#include "CarbonCameraManager.h"
#include "Carbon.h"
#include "ConfigurationManager.h"
#include "TimeManager.h"
#include "CameraManager.h"
#include "minhook/include/MinHook.h"
#include <d3d9.h>

// ==========================================================
// GLOBAL STRUCTS
// ==========================================================

// Set by wrapper hook, consumed by look-at hook
static volatile LONG gApplyUG2Flag = 0;

LONG InterlockedExchange(
    volatile LONG* Target,
    LONG Value
);
// ==========================================================
// WRAPPER HOOK (NAKED, SAFE)
// ==========================================================

static void* oCameraWrapper = nullptr; // trampoline
static volatile LONG gRollArmed = 0;
static float gRollArmTime = 0.0f;

__declspec(naked) void hkCameraWrapper_Naked()
{
    __asm {
        pushfd
        pushad
        mov eax, 1
        lock xchg dword ptr [gApplyUG2Flag], eax   // your old flag, still ok

        popad
        popfd
        jmp dword ptr [oCameraWrapper]
    }
}

// ------------------------------------------------------------
// INIT CAMERA STATE
// ------------------------------------------------------------
static void init_camera(void* outMatrix, Vec3* from, Vec3* to, Vec3* up)
{
    Vec3 fromEngine = *from;
    Vec3 toEngine = *to;
    Vec3 upEngine = norm(*up); // Carbon: Z-up

    Vec3 fEngine = norm(sub(toEngine, fromEngine));
    Vec3 rEngine = norm(cross(fEngine, upEngine)); // engine right
    Vec3 uEngine = norm(cross(rEngine, fEngine)); // re-orthogonalized up

    {
        g_cam.inited = true;
        g_cam.prevFrom = fromEngine;
        g_cam.prevVelDirFilt = fEngine;
        g_cam.velDirFilt = fEngine;
        g_cam.speedFilt = 0.0f;
        g_cam.yawRateFilt = 0.0f;
        g_cam.yawBias = 0.0f;
        g_cam.rollBias = 0.0f;
        g_cam.gNorm = 0.0f;
        g_cam.horizonLock = 1.0f;
    }
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

    // 1) engine builds camera first
    int ret = oCreateLookAt(outMatrix, from, to, up);

    // 2) time ALWAYS advances (prevents director stealing dt)
    const float now = GetTimeSeconds_Safe();

    if (!g_cam.inited)
    {
        init_camera(outMatrix, from, to, up);
        g_cam.lastT = now;              // IMPORTANT: seed time here
        // apply is harmless here (roll=0), but keep consistent
        ApplyRollKeepPos((Mat4*)outMatrix, g_cam.rollBias);
        return ret;
    }

    float rawDt = now - g_cam.lastT;
    g_cam.lastT = now;

    // if dt is garbage, still apply current roll but DO NOT integrate
    if (rawDt <= 0.0f || rawDt > 0.5f)
    {
        ApplyRollKeepPos((Mat4*)outMatrix, g_cam.rollBias);
        return ret;
    }

    float dt = clampf(rawDt, 1.0f / 240.0f, 1.0f / 30.0f);

    // 3) detect dummy/probe cameras (the {0,0,0}->{1,0,0} ones)
    // DO NOT integrate on these.
    const bool isProbe =
        (fabsf(from->x) < 1e-4f && fabsf(from->y) < 1e-4f && fabsf(from->z) < 1e-4f);

    // 4) consume wrapper flag ONCE
    const bool authoritative =
        (InterlockedExchange((volatile LONG*)&gApplyUG2Flag, 0) != 0);

    // =====================================================================
    // INTEGRATE ONLY ON (authoritative && !probe)
    // =====================================================================
    if (authoritative && !isProbe)
    {
        // stable up axis
        Vec3 upW = *up;
        if (len(upW) < 1e-4f) upW = v3(0, 0, 1);
        upW = norm(upW);

        // velocity from camera position
        Vec3 vel = sub(*from, g_cam.prevFrom);
        g_cam.prevFrom = *from;

        // project onto ground plane (Z-up => remove component along upW)
        vel = sub(vel, mul(upW, dot(vel, upW)));

        float speedU = len(vel) / dt;

        Vec3 vdir = vel;
        float vlen = len(vdir);

        float dirResp   = 1.0f - expf(-10.0f * dt);
        float speedResp = 1.0f - expf(-6.0f  * dt);

        if (vlen > 1e-3f && speedU > 1e-2f)
        {
            vdir = mul(vdir, 1.0f / vlen);

            // direction filter (NO lerp helper needed)
            g_cam.velDirFilt = norm(add(
                mul(g_cam.velDirFilt, 1.0f - dirResp),
                mul(vdir,           dirResp)));

            // speed filter
            g_cam.speedFilt += (speedU - g_cam.speedFilt) * speedResp;

            // yaw rate from filtered direction around upW
            Vec3 prevF = g_cam.prevVelDirFilt;
            Vec3 curF  = g_cam.velDirFilt;
            g_cam.prevVelDirFilt = curF;

            float yawS = dot(upW, cross(prevF, curF));   // signed
            float yawRateRaw = yawS / dt;

            float yawResp = 1.0f - expf(-8.0f * dt);
            g_cam.yawRateFilt += (yawRateRaw - g_cam.yawRateFilt) * yawResp;
        }
        else
        {
            // decay when stopped
            g_cam.yawRateFilt += (0.0f - g_cam.yawRateFilt) * speedResp;
            g_cam.speedFilt   += (0.0f - g_cam.speedFilt)   * speedResp;
        }

        // --- UG2 roll intent (configurable) ---
        float yawRate = g_cam.yawRateFilt;
        float speedF  = g_cam.speedFilt;

        if (fabsf(yawRate) < g_rollCfg.yawDeadzone)
            yawRate = 0.0f;

        float latG = fabsf(yawRate) * speedF * g_rollCfg.latGScale;

        float g01 = saturate((latG - g_rollCfg.latGStart) /
                             (g_rollCfg.latGFull  - g_rollCfg.latGStart));
        g01 = g01 * g01 * (3.0f - 2.0f * g01); // smoothstep

        float rollTarget = signf(-yawRate) * g01 * DEG2RAD(g_rollCfg.maxRollDeg);

        // target filter
        float cmdResp = 1.0f - expf(-g_rollCfg.cmdResponse * dt);
        g_cam.yawBias += (rollTarget - g_cam.yawBias) * cmdResp;
        float rollCmd = g_cam.yawBias;

        // rate limit
        float maxStep = DEG2RAD(g_rollCfg.rateLimitDeg) * dt;
        float diff = clampf(rollCmd - g_cam.rollBias, -maxStep, +maxStep);
        rollCmd = g_cam.rollBias + diff;

        // asym damping (wind-in / unwind)
        float k = (fabsf(rollCmd) > fabsf(g_cam.rollBias))
                ? g_rollCfg.windInK
                : g_rollCfg.unwindK;

        g_cam.rollBias += (rollCmd - g_cam.rollBias) * (1.0f - expf(-k * dt));

        // optional clamp
        g_cam.rollBias = clampf(g_cam.rollBias,
                                DEG2RAD(-g_rollCfg.clampRollDeg),
                                DEG2RAD(+g_rollCfg.clampRollDeg));
    }
    else
    {
        // not authoritative OR probe => do not integrate noise; keep roll stable with gentle decay
        float decay = 1.0f - expf(-g_rollCfg.idleDecayK * dt);
        g_cam.rollBias += (0.0f - g_cam.rollBias) * decay;

        // keep prevFrom sane so next real auth frame doesn't see a teleport impulse
        if (!isProbe)
            g_cam.prevFrom = *from;
    }

    // =====================================================================
    // APPLY ON EVERY CALL (this stops Carbon camera fighting)
    // =====================================================================
    ApplyRollKeepPos((Mat4*)outMatrix, g_cam.rollBias);

    return ret;
}

// ==========================================================
// Hook
// ==========================================================
bool InstallHooks_CB()
{
    LoadConfig();

    if (!g_rollCfg.enabled)
        return false;

    if (MH_Initialize() != MH_OK)
        return false;

    // 1) Hook wrapper (authoritative camera update)
    MH_CreateHook(
        (LPVOID)Game::CameraWrapperAddr,
        &hkCameraWrapper_Naked,
        &oCameraWrapper
    );

    // 2) Hook CreateLookAt builder
    MH_CreateHook(
        (LPVOID)Game::CreateLookAtAddr,
        &hkCreateLookAt,
        (LPVOID*)&oCreateLookAt
    );

    // 3) Optional: disable tilts
    if (Game::DisableTiltsAddr)
    {
        static float DisableTilts = -1000.0f;
        *(float**)Game::DisableTiltsAddr = &DisableTilts;
    }

    MH_EnableHook(MH_ALL_HOOKS);

    return true;
}
