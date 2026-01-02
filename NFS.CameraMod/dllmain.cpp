#include <windows.h>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <mutex>
#include "Math.h"
#include "Game.h"
#include "Log.h"
#include "minhook/include/MinHook.h"
#include <d3d9.h>
#include "minhook/include/MinHook.h"

static inline void Orthonormalize(Vec3& r, Vec3& u, Vec3& f)
{
    f = norm(f);
    r = norm(cross(u, f));     // right = up x forward
    u = cross(f, r);           // up   = forward x right
}

// Extract camera basis from a D3D view matrix.
// NOTE: This assumes the matrix you see in SetTransform is a standard view matrix layout used by D3D9.
// We’ll treat rows as basis vectors (common in many engines); if it feels inverted, flip signs below.
static inline void ExtractBasisFromView(const D3DMATRIX& m, Vec3& right, Vec3& up, Vec3& fwd)
{
    // Row-major basis from view:
    right = v3(m._11, m._12, m._13);
    up    = v3(m._21, m._22, m._23);
    fwd   = v3(m._31, m._32, m._33);
    Orthonormalize(right, up, fwd);
}

// Apply roll around forward axis: rotate right/up around fwd by angle
static inline void ApplyRollToView(D3DMATRIX& m, float rollRad)
{
    Vec3 r,u,f;
    ExtractBasisFromView(m, r,u,f);

    float c = std::cos(rollRad);
    float s = std::sin(rollRad);

    // Rodrigues rotation of r and u around f
    Vec3 r2 = add( add(mul(r, c), mul(cross(f, r), s)), mul(f, dot(f,r)*(1.0f-c)) );
    Vec3 u2 = add( add(mul(u, c), mul(cross(f, u), s)), mul(f, dot(f,u)*(1.0f-c)) );

    // Write back basis (keep translation as-is)
    m._11 = r2.x; m._12 = r2.y; m._13 = r2.z;
    m._21 = u2.x; m._22 = u2.y; m._23 = u2.z;
    m._31 = f.x;  m._32 = f.y;  m._33 = f.z;
}

// Compute yaw from forward vector projected to XZ plane
static inline float YawFromForwardXZ(Vec3 f)
{
    // If your yaw feels reversed, swap signs here.
    return std::atan2f(f.x, f.z);
}

static bool IsIdentity(const D3DMATRIX& m)
{
    const float* a = &m._11;
    // strict is fine for your debug dump; if needed, use epsilon
    return a[0]==1 && a[1]==0 && a[2]==0 && a[3]==0 &&
           a[4]==0 && a[5]==1 && a[6]==0 && a[7]==0 &&
           a[8]==0 && a[9]==0 && a[10]==1 && a[11]==0 &&
           a[12]==0 && a[13]==0 && a[14]==0 && a[15]==1;
}

static inline float DWordToFloat(DWORD d)
{
    float f;
    memcpy(&f, &d, sizeof(f));
    return f;
}

// ------------------------
// D3D9 Hooking
// ------------------------
using Direct3DCreate9_t = IDirect3D9* (WINAPI*)(UINT);
static Direct3DCreate9_t g_origDirect3DCreate9 = nullptr;

using CreateDevice_t = HRESULT (STDMETHODCALLTYPE*)(IDirect3D9* self, UINT, D3DDEVTYPE, HWND,
                                                    DWORD, D3DPRESENT_PARAMETERS*, IDirect3DDevice9**);
static CreateDevice_t g_origCreateDevice = nullptr;

// Device vtable hook
using SetTransform_t = HRESULT (STDMETHODCALLTYPE*)(IDirect3DDevice9* self, D3DTRANSFORMSTATETYPE, const D3DMATRIX*);
static SetTransform_t g_origSetTransform = nullptr;

static bool g_projAppliedThisFrame = false;

using Present_t = HRESULT (STDMETHODCALLTYPE*)(
    IDirect3DDevice9*, const RECT*, const RECT*, HWND, const RGNDATA*);

static Present_t g_origPresent = nullptr;
static IDirect3DSurface9* g_backbuffer = nullptr;


static HRESULT STDMETHODCALLTYPE hkSetTransform(
    IDirect3DDevice9* dev,
    D3DTRANSFORMSTATETYPE state,
    const D3DMATRIX* pMat)
{
    if (!pMat) return g_origSetTransform(dev, state, pMat);

    if (state == D3DTS_PROJECTION)
    {
        IDirect3DSurface9* rt = nullptr;
        if (SUCCEEDED(dev->GetRenderTarget(0, &rt)) && rt)
        {
            D3DSURFACE_DESC d{};
            rt->GetDesc(&d);

            // Filter out shadow / small / depth targets
            const bool isMainSceneRT =
                d.Width  >= 800 &&
                d.Height >= 600 &&
                d.Format != D3DFMT_D16 &&
                d.Format != D3DFMT_D24X8 &&
                d.Format != D3DFMT_D24S8;

            if (!isMainSceneRT)
            {
                rt->Release();
                return g_origSetTransform(dev, state, pMat);
            }

            rt->Release();
        }

        D3DMATRIX m = *pMat;

        // HARD TEST (replace later with UG2 roll)
        m._11 *= 0.2f;

        return g_origSetTransform(dev, state, &m);
    }

    return g_origSetTransform(dev, state, pMat);
}

static HRESULT STDMETHODCALLTYPE hkPresent(
    IDirect3DDevice9* dev,
    const RECT* src,
    const RECT* dst,
    HWND wnd,
    const RGNDATA* dirty)
{
    // new frame boundary
    g_projAppliedThisFrame = false;

    return g_origPresent(dev, src, dst, wnd, dirty);
}

using Reset_t = HRESULT (STDMETHODCALLTYPE*)(
    IDirect3DDevice9*, D3DPRESENT_PARAMETERS*);

static Reset_t g_origReset = nullptr;

static HRESULT STDMETHODCALLTYPE hkReset(
    IDirect3DDevice9* dev,
    D3DPRESENT_PARAMETERS* params)
{
    // 🔥 Release dead backbuffer BEFORE reset
    if (g_backbuffer)
    {
        g_backbuffer->Release();
        g_backbuffer = nullptr;
    }

    HRESULT hr = g_origReset(dev, params);

    // Re-acquire AFTER reset
    if (SUCCEEDED(hr))
    {
        IDirect3DSurface9* bb = nullptr;
        if (SUCCEEDED(dev->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb)
        {
            g_backbuffer = bb;
            g_backbuffer->AddRef();
            bb->Release();
        }
    }

    return hr;
}

static void HookDevice(IDirect3DDevice9* dev)
{
    void** vtbl = *(void***)dev;

    constexpr int IDX_SetTransform = 44; // stable
    constexpr int IDX_Present      = 17; // stable

    // Hook SetTransform once
    if (!g_origSetTransform)
    {
        if (MH_CreateHook(vtbl[IDX_SetTransform], &hkSetTransform, (void**)&g_origSetTransform) == MH_OK)
            MH_EnableHook(vtbl[IDX_SetTransform]);
    }

    // Hook Present once
    if (!g_origPresent)
    {
        if (MH_CreateHook(vtbl[IDX_Present], &hkPresent, (void**)&g_origPresent) == MH_OK)
            MH_EnableHook(vtbl[IDX_Present]);
    }
    
    constexpr int IDX_Reset = 16;

    if (!g_origReset)
    {
        if (MH_CreateHook(vtbl[IDX_Reset], &hkReset, (void**)&g_origReset) == MH_OK)
            MH_EnableHook(vtbl[IDX_Reset]);
    }

}

static HRESULT STDMETHODCALLTYPE hkCreateDevice(
    IDirect3D9* self,
    UINT Adapter,
    D3DDEVTYPE DeviceType,
    HWND hFocusWindow,
    DWORD BehaviorFlags,
    D3DPRESENT_PARAMETERS* pPresentationParameters,
    IDirect3DDevice9** ppReturnedDeviceInterface)
{
    HRESULT hr = g_origCreateDevice(
        self,
        Adapter,
        DeviceType,
        hFocusWindow,
        BehaviorFlags,
        pPresentationParameters,
        ppReturnedDeviceInterface);

    if (SUCCEEDED(hr) && ppReturnedDeviceInterface && *ppReturnedDeviceInterface)
    {
        IDirect3DDevice9* dev = *ppReturnedDeviceInterface;

        HookDevice(dev);

        if (!g_backbuffer)
        {
            IDirect3DSurface9* bb = nullptr;

            if (!g_backbuffer)
            {
                IDirect3DSurface9* bb = nullptr;
                if (SUCCEEDED(dev->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb)
                {
                    g_backbuffer = bb;
                    g_backbuffer->AddRef(); // 🔥 REQUIRED
                    bb->Release();          // balance GetBackBuffer
                }
            }

        }
    }

    return hr;
}

static IDirect3D9* WINAPI hkDirect3DCreate9(UINT sdk)
{
    IDirect3D9* d3d9 = g_origDirect3DCreate9(sdk);
    if (!d3d9) return nullptr;

    void** vtbl = *(void***)d3d9;

    constexpr int IDX_CreateDevice = 16;

    if (!g_origCreateDevice)
    {
        if (MH_CreateHook(
                vtbl[IDX_CreateDevice],
                &hkCreateDevice,
                (void**)&g_origCreateDevice) == MH_OK)
        {
            MH_EnableHook(vtbl[IDX_CreateDevice]);
        }
    }

    return d3d9;
}

static DWORD WINAPI InitThread(LPVOID)
{
    // Wait until d3d9 is loaded
    while (!GetModuleHandleA("d3d9.dll"))
        Sleep(50);

    HMODULE hD3D9 = GetModuleHandleA("d3d9.dll");
    auto pCreate9 = (void*)GetProcAddress(hD3D9, "Direct3DCreate9");
    if (!pCreate9) return 0;

    MH_Initialize();
    MH_CreateHook(pCreate9, &hkDirect3DCreate9, (void**)&g_origDirect3DCreate9);
    MH_EnableHook(pCreate9);

    return 0;
}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(hModule);
        CreateThread(nullptr, 0, InitThread, nullptr, 0, nullptr);
    }
    return TRUE;
}
