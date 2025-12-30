#include <algorithm>
#include <windows.h>
#include <cstdint>
#include <cmath>
#include "Game.h"
#include "minhook/include/MinHook.h"

// ==========================================================
// MATH
// ==========================================================

struct Vec3 { float x, y, z; };

static inline Vec3 add(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline Vec3 sub(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline Vec3 mul(Vec3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
static inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static inline Vec3 cross(Vec3 a, Vec3 b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

static inline float len(Vec3 a) { return sqrtf(dot(a, a)); }

static inline Vec3 norm(Vec3 a)
{
    float l = len(a);
    return (l > 1e-6f) ? mul(a, 1.0f / l) : Vec3{ 0,0,0 };
}

static inline Vec3 rotate(Vec3 v, Vec3 axis, float a)
{
    axis = norm(axis);
    float s = sinf(a), c = cosf(a);
    return add(
        add(mul(v, c), mul(cross(axis, v), s)),
        mul(axis, dot(axis, v) * (1.0f - c))
    );
}

static inline float signf(float v)
{
    return (v < 0.0f) ? -1.0f : 1.0f;
}

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}
// ==========================================================
// GLOBAL STRUCTS
// ==========================================================

// Set by wrapper hook, consumed by look-at hook
static volatile LONG gApplyUG2Flag = 0;

struct CamState
{
    bool  inited;
    float lastT;

    Vec3  prevFrom;          // already exists (this is what we will use correctly)

    Vec3  velDirFilt;
    Vec3  prevVelDirFilt;

    float speedFilt;
    float yawRateFilt;

    float yawBias;
    float rollBias;
    float gNorm;
    float horizonLock;
} g_cam;

LONG InterlockedExchange(
    volatile LONG* Target,
    LONG Value
);

// ==========================================================
// TIME
// ==========================================================
using Sim_GetTime_t = float(__cdecl*)();
static Sim_GetTime_t Sim_GetTime = (Sim_GetTime_t)Game::SimGetTimeAddr; // verify in IDA

static float GetTimeSeconds_Safe()
{
    static bool qpcInit = false;
    static LARGE_INTEGER freq{};
    static LARGE_INTEGER t0{};

    // --- Engine time ONLY if stable ---
    if (Sim_GetTime)
    {
        __try
        {
            float t = Sim_GetTime();

            // Reject loading / pre-world values
            if (t > 0.1f && t < 1e7f)
                return t;
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
            // ignore
        }
    }

    // --- QPC fallback ---
    if (!qpcInit)
    {
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&t0);
        qpcInit = true;
        return 0.0f; // IMPORTANT
    }

    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);

    return float(double(now.QuadPart - t0.QuadPart) /
                 double(freq.QuadPart));
}

// ==========================================================
// WRAPPER HOOK (NAKED, SAFE)
// ==========================================================

static void* oCameraWrapper = nullptr; // trampoline

__declspec(naked) void hkCameraWrapper_Naked()
{
    __asm {
        pushfd
        pushad

        mov dword ptr [gApplyUG2Flag], 1

        popad
        popfd

        jmp dword ptr [oCameraWrapper]
    }
}

static bool HookWrapper(uintptr_t wrapperAddr)
{
    if (MH_CreateHook((LPVOID)wrapperAddr, &hkCameraWrapper_Naked, &oCameraWrapper) != MH_OK)
        return false;
    if (MH_EnableHook((LPVOID)wrapperAddr) != MH_OK)
        return false;
    return true;
}

// ==========================================================
// LOOK-AT HOOK
// ==========================================================

using CreateLookAt_t = int(__cdecl*)(void* out, Vec3* from, Vec3* to, Vec3* up);
static CreateLookAt_t oCreateLookAt = nullptr;

static int __cdecl hkCreateLookAt(void* outMatrix, Vec3* from, Vec3* to, Vec3* up)
{
    if (!from || !to || !up)
        return oCreateLookAt(outMatrix, from, to, up);

    // ------------------------------------------------------------
    // 0) SNAPSHOT PHYSICAL CAMERA (ENGINE VALUE)
    // ------------------------------------------------------------
    Vec3 fromPhys = *from;

    // --------------------------------------------------
    // 1) Argument sanity (ALWAYS FIRST)
    // --------------------------------------------------
    if (!from || !to || !up)
        return oCreateLookAt(outMatrix, from, to, up);

    // --------------------------------------------------
    // 2) LOADING / NIS / PRE-WORLD GUARD  <<< HERE
    // --------------------------------------------------
    
    // 2.1) frame gate — ONLY HERE
    if (InterlockedExchange(&gApplyUG2Flag, 0) == 0)
        return oCreateLookAt(outMatrix, from, to, up);
    
    if (!Game::DeltaTime || *Game::DeltaTime <= 0.0f)
        return oCreateLookAt(outMatrix, from, to, up);

    // --------------------------------------------------
    // 3) NOW it is safe to touch time/state
    // --------------------------------------------------
    float t = GetTimeSeconds_Safe();

    if (!g_cam.inited)
    {
        g_cam.inited = true;
        g_cam.lastT = t;

        g_cam.prevFrom = fromPhys;        // PHYSICAL ONLY
        g_cam.velDirFilt = {1,0,0};
        g_cam.prevVelDirFilt = {1,0,0};

        g_cam.speedFilt = 0.0f;
        g_cam.yawRateFilt = 0.0f;
        g_cam.yawBias = 0.0f;
        g_cam.rollBias = 0.0f;
        g_cam.gNorm = 0.0f;
        g_cam.horizonLock = 1.0f;

        return oCreateLookAt(outMatrix, from, to, up);
    }

    float dt = t - g_cam.lastT;
    g_cam.lastT = t;
    dt = clampf(dt, 1.0f / 240.0f, 1.0f / 30.0f);

    // ------------------------------------------------------------
    // 1) VELOCITY FROM PHYSICAL CAMERA ONLY
    // ------------------------------------------------------------
    Vec3 vel = sub(fromPhys, g_cam.prevFrom);
    g_cam.prevFrom = fromPhys;   // IMPORTANT: update immediately

    float speedU = len(vel) / dt;

    Vec3 vdir = vel;
    vdir.y = 0.0f;

    if (len(vdir) > 1e-3f && speedU > 1e-2f)
    {
        vdir = norm(vdir);

        float velResp = 1.0f - expf(-6.0f * dt);
        g_cam.velDirFilt =
            norm(add(mul(g_cam.velDirFilt, 1.0f - velResp),
                     mul(vdir, velResp)));

        g_cam.speedFilt += (speedU - g_cam.speedFilt) * velResp;

        Vec3 prevF = g_cam.prevVelDirFilt;
        Vec3 curF  = g_cam.velDirFilt;
        if (len(prevF) < 0.5f)
            prevF = curF;

        float yawS = (prevF.x * curF.z - prevF.z * curF.x);
        float yawRateRaw = yawS / dt;

        g_cam.prevVelDirFilt = curF;

        float yawResp = 1.0f - expf(-8.0f * dt);
        g_cam.yawRateFilt += (yawRateRaw - g_cam.yawRateFilt) * yawResp;
    }
    else
    {
        float a = 1.0f - expf(-6.0f * dt);
        g_cam.yawRateFilt += (0.0f - g_cam.yawRateFilt) * a;
        g_cam.speedFilt   += (0.0f - g_cam.speedFilt)   * a;
    }

    // ------------------------------------------------------------
    // 2) UG2-STYLE INTENTS (CONSERVATIVE)
    // ------------------------------------------------------------
    float yaw01   = clampf(fabsf(g_cam.yawRateFilt) / 0.6f, 0.0f, 1.0f);
    float speed01 = clampf(g_cam.speedFilt / 45.0f, 0.0f, 1.0f);

    // Bias toward staying locked
    float targetLock =
        1.0f
        - (0.50f * yaw01)     // less sensitive to steering
        - (0.25f * speed01);  // less sensitive to speed

    targetLock = clampf(targetLock, 0.25f, 1.0f);

    // SLOW response ONLY (no snap-back)
    float aLock = 1.0f - expf(-2.5f * dt);
    g_cam.horizonLock += (targetLock - g_cam.horizonLock) * aLock;

    float lateralForce = fabsf(g_cam.yawRateFilt) * g_cam.speedFilt;
    float gTarget = clampf(lateralForce * 0.00006f, 0.0f, 1.0f);
    float aG = 1.0f - expf(-3.0f * dt);
    g_cam.gNorm += (gTarget - g_cam.gNorm) * aG;

    // yaw (HARD CLAMP – prevents 45° ever happening)
    float yawTarget = clampf(g_cam.yawRateFilt * 0.10f, -0.030f, 0.030f);
    float aYaw = 1.0f - expf(-10.0f * dt);
    g_cam.yawBias += (yawTarget - g_cam.yawBias) * aYaw;

    // HARD STATE CLAMP (MANDATORY)
    g_cam.yawBias = clampf(g_cam.yawBias, -0.030f, 0.030f);

    // roll
    float rollTarget =
        signf(g_cam.yawRateFilt) *
        (0.4f * g_cam.gNorm + 0.6f * powf(g_cam.gNorm, 1.3f)) *
        0.30f;

    float speedFade = clampf(g_cam.speedFilt / 18.0f, 0.0f, 1.0f);
    rollTarget *= speedFade;

    float rollResp = (fabsf(rollTarget) < fabsf(g_cam.rollBias)) ? 9.0f : 4.0f;
    float aRoll = 1.0f - expf(-rollResp * dt);
    g_cam.rollBias += (rollTarget - g_cam.rollBias) * aRoll;

    float lateralTarget =
    signf(g_cam.yawRateFilt) *
    g_cam.gNorm * 420.0f;

    float aLat = 1.0f - expf(-4.0f * dt);
    static float lateralFilt = 0.0f;
    lateralFilt += (lateralTarget - lateralFilt) * aLat;

    float lateral = clampf(lateralFilt, -420.0f, 420.0f);

    // --- decay lateral when straight (PUT HERE)
    if (fabsf(g_cam.yawRateFilt) < 0.02f)
    {
        lateral *= expf(-6.0f * dt);
        lateralFilt = lateral;
    }

    // ------------------------------------------------------------
    // 3) APPLY TO VISUAL CAMERA ONLY
    // ------------------------------------------------------------
    const Vec3 worldUp = { 0.0f, -1.0f, 0.0f };

    Vec3 f = norm(sub(*to, fromPhys));

    // project forward onto ground plane
    Vec3 fGround = sub(f, mul(worldUp, dot(f, worldUp)));
    if (len(fGround) < 1e-3f)
        fGround = f;
    fGround = norm(fGround);

    // stable right vector
    Vec3 r = norm(cross(fGround, worldUp));

    // stable up (gravity)
    Vec3 u = worldUp;


    Vec3 fromVis = fromPhys;

    // yaw
    Vec3 offset = sub(fromVis, *to);
    offset = rotate(offset, worldUp, g_cam.yawBias);
    fromVis = add(*to, offset);

    // recompute basis
    f = norm(sub(*to, fromVis));
    r = norm(cross(f, u));
    u = norm(cross(r, f));

    // lateral slide (ground only)
    fromVis = add(fromVis, mul(r, lateral));

    // independent vertical lift (VERY small)
    float lift = g_cam.gNorm * speedFade * 0.35f;
    fromVis = add(fromVis, mul(worldUp, lift));


    // roll
    // --- HORIZON LOCK (UG2 STYLE)
    // base up is ALWAYS gravity
    Vec3 baseUp = worldUp;

    // roll fades out as horizon lock increases
    float effectiveRoll = g_cam.rollBias * (1.0f - g_cam.horizonLock);

    // roll ONLY around forward axis
    *up = rotate(baseUp, f, effectiveRoll);

    // prevent inverted horizon
    if (dot(*up, worldUp) < 0.0f)
        *up = mul(*up, -1.0f);

    *from = fromVis;

    return oCreateLookAt(outMatrix, from, to, up);
}

// ==========================================================
// INSTALL
// ==========================================================

static bool InstallHooks()
{
    if (MH_Initialize() != MH_OK)
        return false;

    // 1) Hook wrapper (authoritative camera update)
    if (!HookWrapper(Game::CameraWrapperAddr))
        return false;

    // 2) Hook CreateLookAt builder
    if (MH_CreateHook((LPVOID)Game::CreateLookAtAddr, &hkCreateLookAt, (LPVOID*)&oCreateLookAt) != MH_OK)
        return false;
    if (MH_EnableHook((LPVOID)Game::CreateLookAtAddr) != MH_OK)
        return false;

    // 3) Optional: disable tilts
    if (Game::DisableTiltsAddr)
    {
        static float DisableTilts = -1000.0f;
        *(float**)Game::DisableTiltsAddr = &DisableTilts;
    }

    OutputDebugStringA("[UG2Cam] Hooks installed.\n");
    return true;
}

// ==========================================================
// DLL ENTRY
// ==========================================================
BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(hModule);

        uintptr_t base = (uintptr_t)GetModuleHandleA(NULL);
        auto* dos = (IMAGE_DOS_HEADER*)base;
        auto* nt  = (IMAGE_NT_HEADERS*)(base + dos->e_lfanew);

        if ((base + nt->OptionalHeader.AddressOfEntryPoint + (0x400000 - base)) == Game::Entry)
        {
            InstallHooks();
        }
        else
        {
            MessageBoxA(NULL, Game::Error, Game::Name, MB_ICONERROR);
            return FALSE;
        }
    }
    return TRUE;
}
