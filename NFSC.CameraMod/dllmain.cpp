// dllmain.cpp - NFS Carbon: UG2-style look-into-turn
// Hook: Camera::SetCameraMatrix @ 0x004822F0

#include <windows.h>
#include <cstdint>
#include <cmath>
#include "minhook/include/MinHook.h"

struct bMatrix4
{
    float m[4][4];
};

// ---------------- hook ----------------
using SetCameraMatrix_t = void(__thiscall*)(void* cam, const bMatrix4* m, float fov);
static SetCameraMatrix_t oSetCameraMatrix = nullptr;

struct v3
{
    float x, y, z;
};

// ---------------- math helpers ----------------
static inline v3 v3_sub(v3 a, v3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline v3 v3_add(v3 a, v3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline v3 v3_mul(v3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline v3 v3_lerp(v3 a, v3 b, float t) { return v3_add(v3_mul(a, 1.0f - t), v3_mul(b, t)); }
static inline float v3_dot(v3 a, v3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float v3_len(v3 a) { return sqrtf(v3_dot(a, a)); }

static inline v3 v3_norm(v3 a)
{
    float l = v3_len(a);
    return (l > 1e-6f) ? v3_mul(a, 1.0f / l) : v3{0, 0, 0};
}

static inline float clampf(float v, float a, float b) { return (v < a) ? a : ((v > b) ? b : v); }
static inline float signf(float v) { return (v < 0.0f) ? -1.0f : 1.0f; }

static inline v3 v3_cross(v3 a, v3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline v3 getRow(const bMatrix4& m, int r) { return {m.m[r][0], m.m[r][1], m.m[r][2]}; }

static inline void setRow(bMatrix4& m, int r, v3 v)
{
    m.m[r][0] = v.x;
    m.m[r][1] = v.y;
    m.m[r][2] = v.z;
}


// ---------------- timing (QPC dt) ----------------
static LARGE_INTEGER g_freq{};
static LARGE_INTEGER g_prev{};

static float GetDt()
{
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);

    if (g_prev.QuadPart == 0)
    {
        g_prev = now;
        return 0.0f;
    }

    double denom = double(g_freq.QuadPart);
    if (denom <= 0.0)
        return 0.0f;

    float dt = (float)(double(now.QuadPart - g_prev.QuadPart) / denom);
    g_prev = now;

    // clamp
    if (dt < 1.0f / 240.0f) dt = 1.0f / 240.0f;
    if (dt > 1.0f / 20.0f) dt = 1.0f / 20.0f;

    return dt;
}

using Sim_GetTime_t = float(__cdecl*)();
static Sim_GetTime_t Sim_GetTime = (Sim_GetTime_t)0x0075CF60; // verify in IDA

static float GetTimeSeconds_Safe()
{
    // 1) Try engine time
    if (Sim_GetTime)
    {
        __try
        {
            float t = Sim_GetTime();
            if (t > 0.001f && t < 1e7f)
                return t;
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
        }
    }

    // 2) Fallback: QPC
    if (g_freq.QuadPart == 0)
    {
        QueryPerformanceFrequency(&g_freq);
        QueryPerformanceCounter(&g_prev);

        if (g_freq.QuadPart == 0)
            return 0.0f;
    }

    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);

    return (float)(
        double(now.QuadPart - g_prev.QuadPart) /
        double(g_freq.QuadPart)
    );
}

// ---------------- look-into-turn state ----------------
struct CamState
{
    bool inited = false;

    // timing
    float lastSimT = 0.0f;
    float lastUpdateT = 0.0f;

    // motion
    v3 prevPos{};
    v3 prevVelDir{}; // normalized horizontal velocity dir
    v3 prevPosReset{};

    v3 velDirFilt{}; // filtered horizontal velocity direction (MOST important)
    v3 prevVelDirFilt{};
    float speedFilt = 0.0f;
    float yawRateFilt = 0.0f;
    float horizonLock = 1.0f; // 1 = locked to horizon, 0 = free bank

    // outputs
    float yawRateRaw = 0.0f; // rad/s (raw, no deadzone)
    float yawBias = 0.0f; // radians (look-into-turn)
    float rollBias = 0.0f; // radians (bank)
    float gNorm = 0.0f; // 0..1 lateral load proxy

    // cached per-frame so second call uses same values
    float cachedYaw = 0.0f;
    float cachedRoll = 0.0f;
    float cachedLateral = 0.0f;
} g_cam;

// ---------------- yaw around LOCAL up (same as your working hook) ----------------
static void ApplyYaw_LocalUp_RowMajor(bMatrix4& m, float yaw)
{
    float s = sinf(yaw);
    float c = cosf(yaw);

    v3 r = v3_norm(getRow(m, 0)); // Right
    v3 f = v3_norm(getRow(m, 1)); // Forward
    v3 u = v3_norm(getRow(m, 2)); // Up

    v3 r2 = v3_add(v3_mul(r, c), v3_mul(f, s));
    v3 f2 = v3_add(v3_mul(f, c), v3_mul(r, -s));

    setRow(m, 0, v3_norm(r2));
    setRow(m, 1, v3_norm(f2));
    setRow(m, 2, u); // unchanged
    // row3 untouched
}

static void ApplyRoll_Horizon_UG2(bMatrix4& m, float roll)
{
    const v3 worldUp = {0.0f, -1.0f, 0.0f};

    // Roll axis (camera forward after yaw)
    v3 f = v3_norm(getRow(m, 1));

    // Camera up (incoming)
    v3 camUp = v3_norm(getRow(m, 2));

    // Gravity-projected up (true horizon)
    v3 gravUp = v3_sub(worldUp, v3_mul(f, v3_dot(worldUp, f)));
    gravUp = v3_norm(gravUp);

    // ---- UG2 horizon bias (THIS is the key) ----
    // Allow roll to fight gravity
    float h = clampf(g_cam.horizonLock * 0.65f, 0.0f, 0.85f);

    v3 u0 = v3_norm(
        v3_add(
            v3_mul(camUp, 1.0f - h),
            v3_mul(gravUp, h)
        )
    );

    // Roll around forward
    float s = sinf(roll);
    float c = cosf(roll);

    v3 u = v3_add(
        v3_mul(u0, c),
        v3_mul(v3_cross(f, u0), s)
    );
    u = v3_norm(u);

    // ---- Correct orthonormal rebuild (critical) ----
    v3 r = v3_norm(v3_cross(u, f)); // RIGHT
    v3 f2 = v3_norm(v3_cross(r, u)); // FORWARD

    setRow(m, 0, r);
    setRow(m, 1, f2);
    setRow(m, 2, u);
}

// ---------------- compute from VELOCITY (UG2-style) ----------------
static void ComputeUG2Like(float dt, const v3& posNow)
{
    // horizontal velocity dir
    v3 vel = v3_sub(posNow, g_cam.prevPos);
    g_cam.prevPos = posNow;

    // if basically not moving, decay effects
    float speedU = v3_len(vel) / dt; // units/sec
    v3 vdir = vel;
    vdir.y = 0.0f;

    if (v3_len(vdir) < 1e-3f || speedU < 1e-2f)
    {
        // decay to neutral
        float a = 1.0f - expf(-6.0f * dt);
        g_cam.yawRateRaw += (0.0f - g_cam.yawRateRaw) * a;
        g_cam.gNorm += (0.0f - g_cam.gNorm) * a;
        g_cam.yawBias += (0.0f - g_cam.yawBias) * a;
        g_cam.rollBias += (0.0f - g_cam.rollBias) * a;

        g_cam.speedFilt += (0.0f - g_cam.speedFilt) * a;
        g_cam.yawRateFilt += (0.0f - g_cam.yawRateFilt) * a;

        // keep direction sane (don’t normalize to zero)
        g_cam.velDirFilt = v3_norm(v3_lerp(g_cam.velDirFilt, v3{1, 0, 0}, a));
        g_cam.prevVelDirFilt = g_cam.velDirFilt;

        // horizon lock back to “default”
        g_cam.horizonLock += (1.0f - g_cam.horizonLock) * a;

        return;
    }

    vdir = v3_norm(vdir);

    // ---------------------------------------------
    // UG2-style velocity filtering
    // ---------------------------------------------
    float velResp = 1.0f - expf(-6.0f * dt); // UG2-ish inertia

    // direction (critical)
    g_cam.velDirFilt = v3_norm(
        v3_add(
            v3_mul(g_cam.velDirFilt, 1.0f - velResp),
            v3_mul(vdir, velResp)
        )
    );

    // speed
    g_cam.speedFilt += (speedU - g_cam.speedFilt) * velResp;

    float speed = g_cam.speedFilt; // use this everywhere below

    // yaw rate from change in horizontal velocity direction
    // signed angle rate around world Y: cross(prev,cur).y / dt

    // yaw rate from change in FILTERED horizontal velocity direction
    v3 prevF = g_cam.prevVelDirFilt;
    v3 curF = g_cam.velDirFilt;

    if (v3_len(prevF) < 0.5f) prevF = curF;

    float yawS = (prevF.x * curF.z - prevF.z * curF.x); // cross(prev,cur).y
    float yawRateRaw = yawS / dt;

    g_cam.prevVelDirFilt = curF;

    // filter yaw rate (optional, but keep it)
    float yawResp = 1.0f - expf(-8.0f * dt);
    g_cam.yawRateFilt += (yawRateRaw - g_cam.yawRateFilt) * yawResp;
    g_cam.yawRateRaw = g_cam.yawRateFilt;

    // ---- deadzone only for VISUAL yaw, not for load ----
    const float kYawDeadzone = 0.02f; // rad/s
    float yawRateFiltered =
        (fabsf(g_cam.yawRateFilt) < kYawDeadzone) ? 0.0f : g_cam.yawRateFilt;

    // ------------------------------------------------------------
    // Horizon lock strength (UG2 behavior)
    // ------------------------------------------------------------

    // yaw intensity (filtered!)
    float yaw01 = clampf(fabsf(g_cam.yawRateRaw) / 0.6f, 0.0f, 1.0f);

    // speed must already be filtered earlier (speedFilt)
    float speed01 = clampf(g_cam.speedFilt / 45.0f, 0.0f, 1.0f);

    // horizon strongly locked when:
    //  - low speed
    //  - small yaw
    float targetLock =
        1.0f - (0.65f * yaw01 + 0.35f * speed01);

    // smooth horizon behavior (UG2 feel)
    float lockResp = (targetLock > g_cam.horizonLock) ? 8.0f : 3.0f;
    float aLock = 1.0f - expf(-lockResp * dt);

    g_cam.horizonLock += (targetLock - g_cam.horizonLock) * aLock;
    g_cam.horizonLock = clampf(g_cam.horizonLock, 0.0f, 0.95f);

    // ---- lateral load proxy (UG2 feel) ----
    // proportional to |yawRate| * speed
    float lateralForce = fabsf(g_cam.yawRateRaw) * speed; // "units/s^2" proxy

    // normalize (tune this ONE number!)
    // start with 1e-5..1e-4 depending on your world scale
    const float kLoadNorm = 0.00006f;
    float gTarget = clampf(lateralForce * kLoadNorm, 0.0f, 1.0f);

    // smooth gNorm (heavy camera)
    float aG = 1.0f - expf(-3.0f * dt);
    g_cam.gNorm += (gTarget - g_cam.gNorm) * aG;

    // ---- yaw look-ahead (UG2-ish) ----
    const float kLookAhead = 0.14f;
    const float kMaxYaw = 0.045f; // ~2.6°
    const float kRespYaw = 10.0f;

    float yawTarget = clampf(yawRateFiltered * kLookAhead, -kMaxYaw, kMaxYaw);
    float aYaw = 1.0f - expf(-kRespYaw * dt);
    g_cam.yawBias += (yawTarget - g_cam.yawBias) * aYaw;

    // ---- roll bank intent from load (UG2-style scalar) ----
    const float kMaxRoll = 0.45f;

    float rollTarget =
        signf(yawRateRaw) *
        (0.35f * g_cam.gNorm + 0.65f * powf(g_cam.gNorm, 1.4f)) *
        kMaxRoll;

    // suppress roll intent at very low speed
    float speedFade = clampf(speed / 18.0f, 0.0f, 1.0f);
    rollTarget *= speedFade;

    // perceptual minimum (motion-based only!)
    if (g_cam.gNorm > 0.05f && speedFade > 0.25f)
    {
        float minRoll = 0.006f; // ~0.35°
        if (fabsf(rollTarget) < minRoll)
            rollTarget = signf(rollTarget) * minRoll;
    }

    // asym response (lean in slow, snap back fast)
    float rollResp = (fabsf(rollTarget) < fabsf(g_cam.rollBias)) ? 9.0f : 4.0f;
    float aRoll = 1.0f - expf(-rollResp * dt);

    g_cam.rollBias += (rollTarget - g_cam.rollBias) * aRoll;
}

// ---------------- hook ----------------
static void __fastcall hkSetCameraMatrix(void* cam, void*, const bMatrix4* m, float fov)
{
    if (!m)
    {
        oSetCameraMatrix(cam, m, fov);
        return;
    }

    // Gameplay-ish FOV filter
    if (fov < 0.015f || fov > 0.040f)
    {
        oSetCameraMatrix(cam, m, fov);
        return;
    }

    // ------------------------------------------------------------
    // Make a LOCAL COPY that we will MODIFY and PASS INTO ENGINE
    // ------------------------------------------------------------
    bMatrix4 mm = *m;
    bMatrix4& live = mm;

    // ------------------------------------------------------------
    // Position
    // ------------------------------------------------------------
    v3 pos = {live.m[3][0], live.m[3][1], live.m[3][2]};

    // ------------------------------------------------------------
    // Time
    // ------------------------------------------------------------
    // if (!Sim_GetTime)
    // {
    //     oSetCameraMatrix(cam, &mm, fov);
    //     return;
    // }

    float t = GetTimeSeconds_Safe();

    // ------------------------------------------------------------
    // First-time init
    // ------------------------------------------------------------
    if (!g_cam.inited)
    {
        g_cam.inited = true;
        g_cam.lastSimT = t;
        g_cam.lastUpdateT = t;

        g_cam.prevPos = pos;
        g_cam.prevPosReset = pos;
        g_cam.prevVelDir = v3{1, 0, 0};

        g_cam.velDirFilt = v3{1, 0, 0};
        g_cam.prevVelDirFilt = g_cam.velDirFilt;
        g_cam.speedFilt = 0.0f;
        g_cam.yawRateFilt = 0.0f;
        g_cam.horizonLock = 1.0f;

        g_cam.yawRateRaw = 0.0f;
        g_cam.yawBias = 0.0f;
        g_cam.rollBias = 0.0f;
        g_cam.gNorm = 0.0f;

        g_cam.cachedYaw = 0.0f;
        g_cam.cachedRoll = 0.0f;
        g_cam.cachedLateral = 0.0f;

        oSetCameraMatrix(cam, &mm, fov);
        return;
    }

    // ------------------------------------------------------------
    // dt
    // ------------------------------------------------------------
    float dt = t - g_cam.lastSimT;
    g_cam.lastSimT = t;
    dt = clampf(dt, 1.0f / 240.0f, 1.0f / 30.0f);

    // ------------------------------------------------------------
    // Teleport / reset detection
    // ------------------------------------------------------------
    v3 dposReset = v3_sub(pos, g_cam.prevPosReset);
    float dpReset = v3_len(dposReset);
    g_cam.prevPosReset = pos;

    if (dpReset > 50000.0f)
    {
        g_cam.prevPos = pos;
        g_cam.yawRateRaw = 0.0f;
        g_cam.yawBias = 0.0f;
        g_cam.rollBias = 0.0f;
        g_cam.gNorm = 0.0f;

        g_cam.prevVelDir = v3{1, 0, 0};
        g_cam.velDirFilt = v3{1, 0, 0};
        g_cam.prevVelDirFilt = g_cam.velDirFilt;
        g_cam.speedFilt = 0.0f;
        g_cam.yawRateFilt = 0.0f;
        g_cam.horizonLock = 1.0f;

        g_cam.cachedYaw = 0.0f;
        g_cam.cachedRoll = 0.0f;
        g_cam.cachedLateral = 0.0f;

        oSetCameraMatrix(cam, &mm, fov);
        return;
    }

    // ------------------------------------------------------------
    // Update ONCE per sim frame
    // ------------------------------------------------------------
    const float kSameFrameEps = 1e-5f;
    if (fabsf(t - g_cam.lastUpdateT) > kSameFrameEps)
    {
        g_cam.lastUpdateT = t;

        ComputeUG2Like(dt, pos);

        const float kMaxLateral = 1200.0f; // Carbon units
        const float kLatFromLoad = 1.10f;

        float lateral =
            signf(g_cam.yawRateRaw) *
            g_cam.gNorm *
            (kLatFromLoad * kMaxLateral);

        g_cam.cachedYaw = g_cam.yawBias;
        g_cam.cachedRoll = g_cam.rollBias;
        g_cam.cachedLateral = clampf(lateral, -kMaxLateral, kMaxLateral);
    }

    float yaw = g_cam.cachedYaw;
    float roll = g_cam.cachedRoll;
    float lateral = g_cam.cachedLateral;

    // ------------------------------------------------------------
    // Translation (lateral + small lift)
    // ------------------------------------------------------------
    v3 right = v3_norm({live.m[0][0], live.m[0][1], live.m[0][2]});
    v3 up = v3_norm({live.m[2][0], live.m[2][1], live.m[2][2]});

    pos = v3_add(pos, v3_mul(right, lateral));
    pos = v3_add(pos, v3_mul(up, fabsf(lateral) * 0.12f));

    live.m[3][0] = pos.x;
    live.m[3][1] = pos.y;
    live.m[3][2] = pos.z;

    // ------------------------------------------------------------
    // Orientation
    // ------------------------------------------------------------
    if (fabsf(yaw) > 1e-6f)
        ApplyYaw_LocalUp_RowMajor(live, yaw);

    if (fabsf(roll) > 1e-6f)
    {
        // 1) Horizon attenuation (CRITICAL)
        float effectiveRoll = roll * (1.0f - g_cam.horizonLock);

        // 2) UG2-style visual exaggeration (OPTIONAL but recommended)
        // simple lerp without helper
        float visualBoost = 1.0f + (1.6f - 1.0f) * g_cam.gNorm;
        effectiveRoll *= visualBoost;

        // 3) Apply
        ApplyRoll_Horizon_UG2(live, effectiveRoll);
    }

    // ------------------------------------------------------------
    // PASS MODIFIED MATRIX INTO ENGINE
    // ------------------------------------------------------------
    oSetCameraMatrix(cam, &mm, fov);
}

// ---------------- init ----------------
static DWORD WINAPI HookThread(LPVOID)
{
    QueryPerformanceFrequency(&g_freq);
    QueryPerformanceCounter(&g_prev); // initialize prev so first dt=0

    constexpr uintptr_t kAddr = 0x004822F0; // YOUR Carbon build

    if (MH_Initialize() != MH_OK) return 0;
    if (MH_CreateHook((LPVOID)kAddr, &hkSetCameraMatrix, (LPVOID*)&oSetCameraMatrix) != MH_OK) return 0;
    if (MH_EnableHook((LPVOID)kAddr) != MH_OK) return 0;

    return 0;
}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(hModule);
        CreateThread(nullptr, 0, HookThread, nullptr, 0, nullptr);
    }
    return TRUE;
}
