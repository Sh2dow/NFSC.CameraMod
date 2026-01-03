#include <windows.h>
#include "Game.h"
#include "MWCameraManager.h"
#include "CarbonCameraManager.h"
// ==========================================================
// DLL ENTRY
// ==========================================================
GameType detected_game = GameType::Unknown;

DWORD WINAPI MainThread(void*)
{
    uintptr_t base = (uintptr_t)GetModuleHandleA(NULL);
    auto* dos = (IMAGE_DOS_HEADER*)base;
    auto* nt = (IMAGE_NT_HEADERS*)(base + dos->e_lfanew);

    uintptr_t entry = base + nt->OptionalHeader.AddressOfEntryPoint;
    bool hooksInstalled = false;

    switch (entry)
    {
    case 0x7C4040: detected_game = GameType::MW;
        {
            Game::Init(detected_game);
            hooksInstalled = InstallHooks_MW();
            break;
        }
    case 0x87E926: detected_game = GameType::CB;
        {
            Game::Init(detected_game);
            hooksInstalled = InstallHooks_CB();
            break;
        }
    case 0x75BCC7: detected_game = GameType::UG2;
        break;
    case 0x670CB5: detected_game = GameType::UG;
        break;
    }

    if (strstr((const char*)(base + (0xA49742 - 0x400000)), "ProStreet08Release.exe"))
        detected_game = GameType::PS;
    else if (detected_game == GameType::Unknown)
        detected_game = GameType::UC;

    if (detected_game != GameType::MW && detected_game != GameType::CB)
        Game::Init(detected_game);

    if (!hooksInstalled)
    {
        MessageBoxA(NULL, Game::Error, Game::Name, MB_ICONERROR);
        return FALSE;
    }
}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(hModule);
        CreateThread(nullptr, 0, MainThread, nullptr, 0, nullptr);
    }
    return TRUE;
}
