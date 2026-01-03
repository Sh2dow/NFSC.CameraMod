#pragma once
#include "Math.h"

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

    float gNorm;
    float horizonLock;

    // NEW (UG2)
    Vec3 upVis;
    Vec3 upVel;

    Vec3 upBase;
    bool hasUpBase;

    bool timeValid;
    bool viewYawValid;
};

extern CameraState g_cam;

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
    V->m[0][0] = R2.x;
    V->m[1][0] = R2.y;
    V->m[2][0] = R2.z;
    V->m[0][1] = U2.x;
    V->m[1][1] = U2.y;
    V->m[2][1] = U2.z;
    V->m[0][2] = B2.x;
    V->m[1][2] = B2.y;
    V->m[2][2] = B2.z;

    V->m[3][0] = -dot(R2, camPos);
    V->m[3][1] = -dot(U2, camPos);
    V->m[3][2] = -dot(B2, camPos);
}

static void ApplyRollKeepPos_Carbon(Mat4* V, float rollRad)
{
    if (!_finite(rollRad)) return;

    // columns
    Vec3 R0 = v3(V->m[0][0], V->m[1][0], V->m[2][0]);
    Vec3 U0 = v3(V->m[0][1], V->m[1][1], V->m[2][1]);
    Vec3 F0 = v3(V->m[0][2], V->m[1][2], V->m[2][2]); // FORWARD

    if (!isfinite3(R0) || !isfinite3(U0) || !isfinite3(F0)) return;

    float Tx = V->m[3][0];
    float Ty = V->m[3][1];
    float Tz = V->m[3][2];

    // camPos = -(Tx*R + Ty*U + Tz*F)
    Vec3 camPos = add(add(mul(R0, -Tx), mul(U0, -Ty)), mul(F0, -Tz));

    Vec3 upW = v3(0.0f, 0.0f, -1.0f); // Carbon Z-up

    Vec3 F = safe_norm(F0, v3(1, 0, 0));

    // (optional) horizon lock: use your g_cam.horizonLock if you want
    Vec3 upProj = sub(upW, mul(F, dot(upW, F)));
    upProj = safe_norm(upProj, U0);

    Vec3 U = upProj; // simplest stable horizon
    Vec3 R = safe_norm(cross(U, F), R0);
    U = safe_norm(cross(F, R), U0);

    // roll around forward axis
    Vec3 R2 = rotate(R, F, rollRad);
    Vec3 U2 = rotate(U, F, rollRad);

    // write back
    V->m[0][0] = R2.x;
    V->m[1][0] = R2.y;
    V->m[2][0] = R2.z;
    V->m[0][1] = U2.x;
    V->m[1][1] = U2.y;
    V->m[2][1] = U2.z;
    V->m[0][2] = F.x;
    V->m[1][2] = F.y;
    V->m[2][2] = F.z;

    V->m[3][0] = -dot(R2, camPos);
    V->m[3][1] = -dot(U2, camPos);
    V->m[3][2] = -dot(F, camPos);
}
