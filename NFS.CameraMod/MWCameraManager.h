#pragma once
#include "Math.h"
#include "MostWanted.h"
#include <d3d9.h>

#include "TimeManager.h"
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

// ------------------------------------------------------------
// INIT CAMERA STATE
// ------------------------------------------------------------
struct CameraState
{
    bool inited;
    float lastT;

    Vec3 prevFrom;
    Vec3 prevF;
    bool hasPrevF;

    Vec3 velDirFilt;
    Vec3 prevVelDirFilt;
    float rollTargetFilt;

    float speedFilt;
    float yawRateFilt;

    float yawBias;
    float rollBias;

    float horizonLock;

    // NEW (UG2)
    Vec3 upVis;
    Vec3 upVel;

    Vec3 upBase;
    bool hasUpBase;

    bool timeValid;
    bool viewYawValid;
} g_cam;

static void init_camera(Vec3* eye)
{
    g_cam.inited = true;

    g_cam.prevFrom = *eye;

    // Seed velocity direction safely (no impulses)
    g_cam.velDirFilt = v3(0.0f, 0.0f, 1.0f); // MW forward (XZ)
    g_cam.prevVelDirFilt = g_cam.velDirFilt;
    g_cam.rollTargetFilt = 0.0f;

    g_cam.speedFilt = 0.0f;
    g_cam.yawRateFilt = 0.0f;
    g_cam.rollBias = 0.0f;
    g_cam.horizonLock = 1.0f;

    // force dt resync next frame
    g_cam.timeValid = false;

    g_cam.viewYawValid = false;
}

// ==========================================================
// LOOK-AT HOOK
// ==========================================================
int __fastcall hkCubicUpdate(void* self, void*, float dt)
{
    InterlockedExchange(&gApplyUG2Flag, 1);
    return oCubicUpdate(self, dt);
}

// MW look-at writes basis into COLUMNS:
// col0 = right, col1 = up, col2 = back (D3D view convention)
static void ApplyRollKeepPos(Mat4* V, float rollRad)
{
    // Reject only truly invalid roll
    if (!_finite(rollRad))
        return;

    // Read original basis (columns)
    Vec3 R0 = v3(V->m[0][0], V->m[1][0], V->m[2][0]);
    Vec3 U0 = v3(V->m[0][1], V->m[1][1], V->m[2][1]);
    Vec3 B0 = v3(V->m[0][2], V->m[1][2], V->m[2][2]);

    if (!isfinite3(R0) || !isfinite3(U0) || !isfinite3(B0))
        return;

    // Recover camera position
    float Tx = V->m[3][0];
    float Ty = V->m[3][1];
    float Tz = V->m[3][2];

    Vec3 camPos =
        add(add(mul(R0, -Tx),
                mul(U0, -Ty)),
                mul(B0, -Tz));

    // Forward (from BACK column)
    Vec3 F = safe_norm(mul(B0, -1.0f), v3(0, 0, 1));

    // --- derive world up from camera up (Z-up MW safe)
    Vec3 UpWorld = U0;
    float ax = fabsf(UpWorld.x);
    float ay = fabsf(UpWorld.y);
    float az = fabsf(UpWorld.z);

    if (az >= ax && az >= ay)
        UpWorld = v3(0, 0, (UpWorld.z >= 0.0f) ? 1.0f : -1.0f);
    else if (ay >= ax)
        UpWorld = v3(0, (UpWorld.y >= 0.0f) ? 1.0f : -1.0f, 0);
    else
        UpWorld = v3((UpWorld.x >= 0.0f) ? 1.0f : -1.0f, 0, 0);

    // --- FORCE FORWARD TO BE PLANAR (THIS WAS THE MISSING PIECE)
    F = sub(F, mul(UpWorld, dot(F, UpWorld)));
    F = safe_norm(F, v3(0, 0, 1));

    // --- horizon lock (stable)
    Vec3 UpProj = sub(UpWorld, mul(F, dot(UpWorld, F)));
    UpProj = safe_norm(UpProj, U0);

    float hLock = clampf(g_cam.horizonLock, 0.65f, 0.95f);
    Vec3 U = safe_norm(
        add(mul(U0, 1.0f - hLock),
            mul(UpProj, hLock)),
        U0);

    // --- build right vector safely
    Vec3 Rn_raw = cross(F, U);
    float rlen = len(Rn_raw);

    Vec3 Rn;
    if (rlen > 1e-5f && _finite(rlen))
    {
        Rn = mul(Rn_raw, 1.0f / rlen);
    }
    else
    {
        Vec3 altR = cross(UpWorld, F);
        if (len(altR) < 1e-5f)
            altR = cross(v3(0, 1, 0), F);

        Rn = safe_norm(altR, R0);
    }

    // --- rebuild up
    Vec3 Un = safe_norm(cross(Rn, F), U0);

    // --- apply roll about forward axis
    Vec3 R2 = rotate(Rn, F, rollRad);
    Vec3 U2 = rotate(Un, F, rollRad);

    R2 = safe_norm(R2, R0);
    U2 = safe_norm(U2, U0);

    Vec3 B2 = safe_norm(cross(R2, U2), B0);

    // --- write back matrix
    V->m[0][0] = R2.x; V->m[1][0] = R2.y; V->m[2][0] = R2.z;
    V->m[0][1] = U2.x; V->m[1][1] = U2.y; V->m[2][1] = U2.z;
    V->m[0][2] = B2.x; V->m[1][2] = B2.y; V->m[2][2] = B2.z;

    V->m[3][0] = -dot(R2, camPos);
    V->m[3][1] = -dot(U2, camPos);
    V->m[3][2] = -dot(B2, camPos);
}

void __cdecl hkCreateLookAtMatrix(Mat4* mat, Vec3* eye, Vec3* center, Vec3* up)
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
    // ROLL INTENT (UG2 STYLE — CAMERA HEADING BASED)
    // ------------------------------------------------------------
    float yawRate = g_cam.yawRateFilt;
    if (fabsf(yawRate) < 0.01f) yawRate = 0.0f;

    float latG = fabsf(yawRate) * g_cam.speedFilt;

    // UG2 knee
    float g01 = saturate((latG - 2.5f) / (8.0f - 2.5f));
    g01 = g01 * g01 * (3.0f - 2.0f * g01);

    // roll target (lean INTO the turn)
    float rollTarget =
        signf(-yawRate) *
        g01 *
        DEG2RAD(8.5f);

    bool yawActive = (fabsf(yawRate) >= 0.01f) && (g_cam.speedFilt >= 2.0f);

    if (!yawActive)
    {
        rollTarget = 0.0f;
        g_cam.rollTargetFilt = 0.0f; // <-- important
    }

    // ------------------------------------------------------------
    // FILTER + RATE LIMIT (UNCHANGED)
    // ------------------------------------------------------------
    float cmdResp = 1.0f - expf(-12.0f * dt);
    g_cam.rollTargetFilt += (rollTarget - g_cam.rollTargetFilt) * cmdResp;
    rollTarget = g_cam.rollTargetFilt;

    float maxStep = DEG2RAD(90.0f) * dt;
    float diff = clampf(rollTarget - g_cam.rollBias, -maxStep, +maxStep);
    rollTarget = g_cam.rollBias + diff;

    float k = (fabsf(rollTarget) > fabsf(g_cam.rollBias)) ? 12.0f : 7.0f;

    g_cam.rollBias += (rollTarget - g_cam.rollBias) *
        (1.0f - expf(-k * dt));

    g_cam.rollBias = clampf(g_cam.rollBias, DEG2RAD(-10.0f), DEG2RAD(10.0f));

    if (g_cam.speedFilt < 6.0f)
    {
        g_cam.rollBias += (0.0f - g_cam.rollBias) *
            (1.0f - expf(-10.0f * dt));
    }

    // ------------------------------------------------------------
    // HORIZON LOCK (USE SAME YAW SOURCE)
    // ------------------------------------------------------------
    float yaw01 = saturate(fabsf(yawRate) / 0.12f);
    float speed01 = saturate(g_cam.speedFilt / 38.0f);

    g_cam.horizonLock =
        clampf(0.95f - 0.35f * yaw01 + 0.10f * speed01,
               0.55f, 0.98f);

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

static bool InstallHooks()
{
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
