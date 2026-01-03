#pragma once

#include <cstdint>

#include "MostWanted.h"
#include "Carbon.h"

enum class GameType {
    MW,
    CB,
    UG2,
    UG,
    PS,
    UC,
    Unknown
};

extern GameType detected_game;

namespace Game
{
    inline const char* Name = "NFS - Camera Mod";
    inline const char* Error = "This .exe is not supported.";

    inline uintptr_t SimGetTimeAddr = 0;
    inline uintptr_t CameraWrapperAddr = 0;
    inline uintptr_t CreateLookAtAddr = 0;
    inline uintptr_t DisableTiltsAddr = 0;

    inline void Init(GameType type)
    {
        switch (type)
        {
        case GameType::MW:
            Name = MW::Name;
            Error = MW::Error;
            SimGetTimeAddr = MW::SimGetTimeAddr;
            CameraWrapperAddr = MW::CameraWrapperAddr;
            CreateLookAtAddr = MW::CreateLookAtAddr;
            DisableTiltsAddr = MW::DisableTiltsAddr;
            break;
        case GameType::CB:
            Name = Carbon::Name;
            Error = Carbon::Error;
            SimGetTimeAddr = Carbon::SimGetTimeAddr;
            CameraWrapperAddr = Carbon::CameraWrapperAddr;
            CreateLookAtAddr = Carbon::CreateLookAtAddr;
            DisableTiltsAddr = Carbon::DisableTiltsAddr;
            break;
        default:
            Name = "NFS - Camera Mod";
            Error = "This .exe is not supported.";
            SimGetTimeAddr = 0;
            CameraWrapperAddr = 0;
            CreateLookAtAddr = 0;
            DisableTiltsAddr = 0;
            break;
        }
    }
}
