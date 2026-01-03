#include "MWCameraManager.h"

#include <cmath>

#include "Math.h"
#include "MostWanted.h"
#include "ConfigurationManager.h"
#include "TimeManager.h"
#include "CameraManager.h"
#include "minhook/include/MinHook.h"

// ==========================================================
// GLOBAL STRUCTS
// ==========================================================
// Set by wrapper hook, consumed by look-at hook
static volatile LONG gApplyUG2Flag = 0;

constexpr float PI = 3.14159265358979323846f;

using CubicUpdate_t = int(__thiscall*)(void* self, float dt);
using CreateLookAt_t = void(__cdecl*)(Mat4* mat, Vec3* eye, Vec3* center, Vec3* up);

static CubicUpdate_t oCubicUpdate = nullptr;
static CreateLookAt_t oCreateLookAt = nullptr;

// ==========================================================
// LOOK-AT HOOK
// ==========================================================
static int __fastcall hkCubicUpdate(void* self, void*, float dt)
{
    InterlockedExchange(&gApplyUG2Flag, 1);
    return oCubicUpdate(self, dt);
}

static void __cdecl hkCreateLookAtMatrix(Mat4* mat, Vec3* eye, Vec3* center, Vec3* up)
{
    // Let MW build camera (director, blends, etc.)
    oCreateLookAt(mat, eye, center, up);

    // Authoritative pass only once per frame
    bool applyRoll = (InterlockedExchange(&gApplyUG2Flag, 0) != 0);

    // ------------------------------------------------------------
    // INIT
    // ------------------------------------------------------------
    if (!g_cam.inited)
    {
        init_camera(eye);
        return;
    }

    // ------------------------------------------------------------
    // TIME
    // ------------------------------------------------------------
    float t = GetTimeSeconds_Safe();

    if (!g_cam.timeValid)
    {
        g_cam.lastT = t;
        g_cam.timeValid = true;
        g_cam.prevFrom = *eye;
        g_cam.viewYawValid = false;
        return;
    }

    float rawDt = t - g_cam.lastT;
    g_cam.lastT = t;

    if (rawDt <= 0.0f || rawDt > 0.5f)
    {
        g_cam.viewYawValid = false;
        return;
    }

    float dt = clampf(rawDt, 1.0f / 240.0f, 1.0f / 30.0f);

    // ------------------------------------------------------------
    // CAMERA POSITION / CUT DETECTION
    // ------------------------------------------------------------
    Vec3 fromNow = *eye;
    Vec3 delta = sub(fromNow, g_cam.prevFrom);

    // Determine vertical axis from 'up' argument (works in all cams)
    Vec3 upW = *up;
    if (len(upW) < 1e-4f) upW = v3(0, 0, 1);
    upW = norm(upW);

    // Flatten delta onto ground plane
    delta = sub(delta, mul(upW, dot(delta, upW)));

    float cutDist = len(delta);

    // Cut threshold: distance on ground plane + dt sanity
    if (cutDist > 60.0f)
    {
        g_cam.prevFrom = fromNow;

        // Reset motion estimates
        g_cam.velDirFilt = v3(0, 0, 1);
        g_cam.prevVelDirFilt = g_cam.velDirFilt;
        g_cam.speedFilt = 0.0f;
        g_cam.yawRateFilt = 0.0f;

        // Reset roll
        g_cam.rollBias = 0.0f;
        g_cam.rollTargetFilt = 0.0f;

        g_cam.viewYawValid = false;
        return;
    }

    g_cam.prevFrom = fromNow;

    // ------------------------------------------------------------
    // SPEED + VELOCITY DIRECTION (GROUND PLANE)
    // ------------------------------------------------------------
    float speed = cutDist / dt;

    Vec3 vdir = delta;
    float vlen = len(vdir);

    float dirResp = 1.0f - expf(-10.0f * dt);
    float speedResp = 1.0f - expf(-6.0f * dt);

    if (vlen > 1e-3f && speed > 1e-2f)
    {
        vdir = mul(vdir, 1.0f / vlen);

        g_cam.velDirFilt = norm(add(
            mul(g_cam.velDirFilt, 1.0f - dirResp),
            mul(vdir, dirResp)));

        g_cam.speedFilt += (speed - g_cam.speedFilt) * speedResp;

        Vec3 prevF = g_cam.prevVelDirFilt;
        Vec3 curF = g_cam.velDirFilt;
        g_cam.prevVelDirFilt = curF;

        // Signed yaw delta on the ground plane around upW:
        // yawS = dot(upW, cross(prevF, curF))
        float yawS = dot(upW, cross(prevF, curF));
        float yawRateRaw = yawS / dt;

        float yawResp = 1.0f - expf(-8.0f * dt);
        g_cam.yawRateFilt += (yawRateRaw - g_cam.yawRateFilt) * yawResp;
    }
    else
    {
        g_cam.yawRateFilt += (0.0f - g_cam.yawRateFilt) * speedResp;
        g_cam.speedFilt += (0.0f - g_cam.speedFilt) * speedResp;
    }

    // Deadzone
    if (fabsf(g_cam.yawRateFilt) < 0.015f)
        g_cam.yawRateFilt = 0.0f;

    // ------------------------------------------------------------
    // ROLL INTENT (UG2 STYLE - CAMERA HEADING BASED)
    // ------------------------------------------------------------
    float yawRate = g_cam.yawRateFilt;
    if (fabsf(yawRate) < g_rollCfg.yawDeadzone)
        yawRate = 0.0f;

    float latG =
        fabsf(yawRate) *
        g_cam.speedFilt *
        g_rollCfg.latGScale;

    float g01 = saturate(
        (latG - g_rollCfg.latGStart) /
        (g_rollCfg.latGFull - g_rollCfg.latGStart));

    g01 = g01 * g01 * (3.0f - 2.0f * g01);

    float rollTarget =
        signf(-yawRate) *
        g01 *
        DEG2RAD(g_rollCfg.maxRollDeg);

    // ------------------------------------------------------------
    // UG2 RESPONSE
    // ------------------------------------------------------------

    // target filter (UG2 has noticeable lag)
    float cmdResp = 1.0f - expf(-g_rollCfg.cmdResponse * dt);
    g_cam.rollTargetFilt += (rollTarget - g_cam.rollTargetFilt) * cmdResp;
    rollTarget = g_cam.rollTargetFilt;

    float maxStep = DEG2RAD(g_rollCfg.rateLimitDeg) * dt;
    float diff = clampf(rollTarget - g_cam.rollBias, -maxStep, +maxStep);
    rollTarget = g_cam.rollBias + diff;

    float k = (fabsf(rollTarget) > fabsf(g_cam.rollBias))
            ? g_rollCfg.windInK
            : g_rollCfg.unwindK;

    g_cam.rollBias += (rollTarget - g_cam.rollBias) *
                      (1.0f - expf(-k * dt));

    // ------------------------------------------------------------
    // HORIZON LOCK (USE SAME YAW SOURCE)
    // ------------------------------------------------------------
    float yaw01   = saturate(fabsf(yawRate) / 0.10f);
    float speed01 = saturate(g_cam.speedFilt / 35.0f);

    g_cam.horizonLock =
        clampf(
            g_rollCfg.horizonBase
            - g_rollCfg.horizonYawK * yaw01
            + g_rollCfg.horizonSpeedK * speed01,
            g_rollCfg.horizonMin,
            g_rollCfg.horizonMax);

    // ------------------------------------------------------------
    // APPLY ROLL
    // ------------------------------------------------------------
    if (!applyRoll)
    {
        // non-authoritative call: just relax toward level (no yaw, no targets)
        float decay = 1.0f - expf(-6.0f * dt);
        g_cam.rollBias += (0.0f - g_cam.rollBias) * decay;
        return;
    }

    // authoritative apply
    if (applyRoll)
        ApplyRollKeepPos(mat, g_cam.rollBias);
}

// ==========================================================
// Hook
// ==========================================================

bool InstallHooks_MW()
{
    LoadConfig();

    if (!g_rollCfg.enabled)
        return false;

    if (MH_Initialize() != MH_OK)
        return false;

    MH_CreateHook(
        (LPVOID)Game::DisableTiltsAddr,
        hkCubicUpdate,
        (LPVOID*)&oCubicUpdate
    );

    MH_CreateHook(
        (LPVOID)Game::CreateLookAtAddr,
        hkCreateLookAtMatrix,
        (LPVOID*)&oCreateLookAt
    );

    MH_EnableHook(MH_ALL_HOOKS);

    return true;
}
