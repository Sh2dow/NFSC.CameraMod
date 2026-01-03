#pragma once
#include "Log.h"

struct CamRollConfig
{
    float enabled = 1.0f;

    // --- lateral load model
    float latGScale = 0.16f; // yawRate * speed multiplier
    float latGStart = 1.0f; // roll starts here
    float latGFull = 5.5f; // full roll here

    // --- roll limits
    float maxRollDeg = 8.0f; // degrees
    float clampRollDeg   = 8.5f;    // UG2-ish

    float idleDecayK     = 2.5f;

    // --- response
    float cmdResponse = 6.0f; // target filter speed
    float rateLimitDeg = 45.0f; // deg/sec
    float windInK = 7.0f;
    float unwindK = 11.0f;

    // --- yaw / speed thresholds
    float yawDeadzone = 0.005f;
    float minSpeed = 2.0f;

    // --- horizon lock
    float horizonBase = 0.92f;
    float horizonYawK = 0.45f;
    float horizonSpeedK = 0.12f;
    float horizonMin = 0.55f;
    float horizonMax = 0.95f;
    
    // optional camera offsets (your lateral/lift)
    float lateralMax     = 120.0f;   // world units
    float liftMax        = 0.35f;    // world units-ish (scaled below)
};

extern CamRollConfig g_rollCfg;

static float ReadIniFloat(
    const char* file,
    const char* section,
    const char* key,
    float def)
{
    char buf[64] = {};
    GetPrivateProfileStringA(
        section, key, "", buf, sizeof(buf), file);

    if (buf[0] == '\0')
        return def;

    return (float)atof(buf);
}

static void LoadConfig()
{
    const char* ini = ".\\NFS.CameraMod.ini";

    g_rollCfg.enabled        = ReadIniFloat(ini, "Main", "Enabled",      g_rollCfg.latGScale);
    
    g_rollCfg.latGStart      = ReadIniFloat(ini, "Roll", "LatGStart",      g_rollCfg.latGStart);
    g_rollCfg.latGFull       = ReadIniFloat(ini, "Roll", "LatGFull",       g_rollCfg.latGFull);

    g_rollCfg.maxRollDeg     = ReadIniFloat(ini, "Roll", "MaxRollDeg",     g_rollCfg.maxRollDeg);

    g_rollCfg.cmdResponse    = ReadIniFloat(ini, "Roll", "CmdResponse",    g_rollCfg.cmdResponse);
    g_rollCfg.rateLimitDeg   = ReadIniFloat(ini, "Roll", "RateLimitDeg",   g_rollCfg.rateLimitDeg);
    g_rollCfg.windInK        = ReadIniFloat(ini, "Roll", "WindInK",        g_rollCfg.windInK);
    g_rollCfg.unwindK        = ReadIniFloat(ini, "Roll", "UnwindK",        g_rollCfg.unwindK);

    g_rollCfg.yawDeadzone    = ReadIniFloat(ini, "Roll", "YawDeadzone",    g_rollCfg.yawDeadzone);
    g_rollCfg.minSpeed       = ReadIniFloat(ini, "Roll", "MinSpeed",       g_rollCfg.minSpeed);

    g_rollCfg.horizonBase    = ReadIniFloat(ini, "Horizon", "Base",        g_rollCfg.horizonBase);
    g_rollCfg.horizonYawK    = ReadIniFloat(ini, "Horizon", "YawK",        g_rollCfg.horizonYawK);
    g_rollCfg.horizonSpeedK  = ReadIniFloat(ini, "Horizon", "SpeedK",      g_rollCfg.horizonSpeedK);
    g_rollCfg.horizonMin     = ReadIniFloat(ini, "Horizon", "Min",         g_rollCfg.horizonMin);
    g_rollCfg.horizonMax     = ReadIniFloat(ini, "Horizon", "Max",         g_rollCfg.horizonMax);
    
    
    g_rollCfg.lateralMax     = ReadIniFloat(ini, "Offsets", "LateralMax",  g_rollCfg.lateralMax);
    g_rollCfg.liftMax        = ReadIniFloat(ini, "Offsets", "LiftMax",     g_rollCfg.liftMax);
}
