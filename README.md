# Motion Bridge

[简体中文](README-ZH.md)

Native Windows bridge that converts real-time game motion into multi-axis device output. Fallen Doll is the first bundled game adapter, while the public `motion-frame/v1` input keeps the application open to additional games. The core is C++20 and does not depend on Qt; the desktop application uses Qt 6/QML.

Motion Bridge has one real-time worker thread for file watching, motion calculation and output. QML only renders copied snapshots at its own cadence; it never blocks the device path.

## Current features

- Real-time L0/L1/L2/R0/R1/R2 processing for OSR2/SR6 workflows.
- USB serial, Wi-Fi UDP (`tcode.local:8000` by default), and Intiface Desktop output.
- Separate SR6/OSR 3D viewer with optional always-on-top mode.
- Per-axis gain and output-range controls, saved beside the executable in portable mode.
- Light and dark themes, plus English, Simplified Chinese, and system-language selection.
- Safe startup, stream-loss hold and smooth return to center. Device output always starts disarmed.

## Quick start with Fallen Doll

1. Install the game-side Mod from [MotionBridge-FallenDoll](https://github.com/Huarch/MotionBridge-FallenDoll).
2. Extract the Motion Bridge portable ZIP and run `MotionBridge.exe`.
3. Enter an HAnime and wait for the stream status to show **ONLINE**.
4. Open the separate 3D preview and check the motion before connecting hardware.
5. Select USB, Wi-Fi, or Intiface, then explicitly choose **ARM OUTPUT**.

The compatibility stream is currently read from `%USERPROFILE%/.f8/studio/games/fallen-doll/runtime/fd-skeleton.ndjson`; F8Studio, `fd_source`, and `fd_pyengine` do not need to be running.

## Development

Build the deterministic core and tests without Qt:

```powershell
cmake -S . -B out/core -DMOTION_BRIDGE_BUILD_GUI=OFF
cmake --build out/core --config Release
ctest --test-dir out/core -C Release --output-on-failure
```

For the desktop application, install Qt 6.8+ with Core, Quick, Quick3D, SerialPort, and WebSockets, then configure with `MOTION_BRIDGE_BUILD_GUI=ON`.

The repository contains two Windows helpers:

```powershell
# One-time development toolchain under .toolchain/qt
.\tools\Install-MotionBridgeToolchain.ps1

# Deterministic core only (does not require Qt)
.\tools\Build-MotionBridge.ps1 -CoreOnly

# Desktop application after Qt installation
.\tools\Build-MotionBridge.ps1 -QtPrefix D:\path\to\Qt\6.8.3\msvc2022_64
```

The game-side UE4SS Mod remains the source of compact functional-bone frames. This project consumes its `fd-skeleton.ndjson` output directly, so F8Studio does not need to be running. File watching is backed by a 50 Hz incremental read to remain reliable while Unreal replaces or keeps the stream file open.

The chosen stream file also accepts the public `motion-frame/v1` NDJSON format in `adapters/`. This is the supported extension point for future games: an adapter writes a completed frame per line. Motion Bridge does not load arbitrary third-party DLLs.

## Portable build

After installing the isolated Qt/MinGW toolchain, create a tested portable directory and ZIP:

```powershell
.\tools\Build-MotionBridge.ps1 -Portable
```

It uses `windeployqt` to copy the Qt runtime beside the executable. The output stays under the ignored `dist/` folder.

## Output safety

- USB sends TCode at 115200 baud.
- Wi-Fi uses the same UDP TCode transport as the existing F8Studio project (`tcode.local:8000` by default).
- Intiface connects to the user's Intiface Desktop WebSocket at `ws://127.0.0.1:12345`. The first implementation only enables a device with a declared Linear feature and maps L0 to that feature; it does not pretend that a generic Intiface device is an SR6.
- A start or imported configuration is always disarmed. Stream loss holds the final value for 250 ms, then returns to center over 600 ms.

## One-time F8Studio migration

The following reads the saved Fallen Doll project from `%USERPROFILE%\.f8\studio\assets.db` and writes `%LOCALAPPDATA%\MotionBridge\motion-bridge.ini`. It imports connection addresses, range and tuning values, but always leaves output disarmed.

```powershell
python .\tools\Import-F8StudioSettings.py
```
