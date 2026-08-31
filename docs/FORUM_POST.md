<!-- Suggested forum title: [Release][Multi-Axis] Motion Bridge + Fallen Doll Mod — TCode, Intiface & Handy (OSR2 / SR6) -->

![PixPin_2026-08-27_03-20-02|690x374](upload://cvGVBFJtcoofbpj2SVQhdvHgOrE.jpeg)
![PixPin_2026-08-27_03-21-04|690x375](upload://xEfgn154cZX6ZGT6s5J4YBJMRUm.jpeg)


## English

This release uses a simple two-part setup for **Operation Lovecraft: Fallen Doll**:

- The **Fallen Doll Mod** runs with the game and writes a compact live motion stream.
- **Motion Bridge** is the new standalone Windows application. It reads that stream, shows the six-axis motion and 3D device preview, and sends output to your hardware.

They are designed to be used together.

Game pages:
https://store.steampowered.com/app/1685960/
https://store.steampowered.com/app/1811180/

### Motion Bridge

- Real-time six-axis `L0/L1/L2/R0/R1/R2` motion for SR6; OSR2 uses `L0`.
- USB TCode, Wi-Fi UDP TCode (default: `tcode.local:8000`), Intiface Desktop, and direct Handy output.
- Compatible with Intiface Central 2.6.
- A separate live SR6/OSR 3D preview, which can stay on top of the game window.
- Per-axis gain and output-range controls, cross-axis smart limits with an editable two-point curve, hard speed limits, output switches, and configurable safe positions. A selected driver axis can dynamically reduce another axis' range or response speed. Axis cards and the 3D preview show the final protected signal even while hardware output is disarmed. Settings are saved with the portable app.
- Safe startup: output always begins disarmed. On stream loss, motion briefly holds and then returns smoothly to centre.
- English and Simplified Chinese UI.
- Use the matching Mod branch for Fallen Doll Demo or Fallen Doll Playtest.
- The Mod also includes additional support for humanoid actions.

### Install and use

1. Download the latest **Fallen Doll Mod release** for your game version, then close the game.
2. Extract the Mod package. Run `Install-Mod.ps1` and select the game folder, or copy the package's `Game` contents into `Paralogue/Binaries/Win64` inside that game folder. The Mod must be enabled so it can write the `fd-skeleton.ndjson` motion stream.
3. Download and extract the latest **Motion Bridge** portable ZIP, then start `MotionBridge.exe`.
4. Start Fallen Doll and enter an HAnime. In Motion Bridge, wait until **STREAM** shows **ONLINE**.
5. Open the 3D preview and verify that the motion looks correct before connecting hardware.
6. Choose one output method: **USB**, **Wi-Fi**, **Intiface**, or **Handy**. Check the port/address, set safe per-axis ranges, then explicitly select **ARM OUTPUT**. Direct Handy output needs its Handyverse connection key; the key is only kept until Motion Bridge closes.

Do not arm output until the preview is correct. Stop/disarm the app before changing hardware or testing a new pose.

If you encounter a problem, please leave a reply in this topic with your game version, the scene/action, and any relevant error details.

### Troubleshooting — STREAM WAITING

MotionBridge and the Mod do not make a network connection to each other. The Mod writes a local motion file and MotionBridge reads the same file:

`%USERPROFILE%\.f8\studio\games\fallen-doll\runtime\fd-skeleton.ndjson`

**STREAM WAITING** means that MotionBridge has not received a new valid bone frame recently. Check these points in order:

1. Use the Mod release matching the exact game edition. Do not assume the repository's Latest release matches Demo, Playtest, or another installation.
2. Confirm the Mod is installed beside the game executable. A working install contains `Paralogue\Binaries\Win64\dwmapi.dll`, `ue4ss\UE4SS.dll`, and `ue4ss\Mods\fd_tcode_probe\Scripts\main.lua`.
3. Open `Paralogue\Binaries\Win64\ue4ss\UE4SS.log` after starting the game. This distinguishes UE4SS not injecting from the Lua Mod not loading.
4. Enter a recognised HAnime, then check whether `fd-skeleton.ndjson` exists and its modified time continues to change while the action is running.
5. Check for old `F8STUDIO_GAMES_DIR` or `FD_TCODE_RUNTIME_DIR` environment variables. Either one can deliberately redirect the Mod to a different runtime folder.

When reporting this issue, include the game edition and version, the complete downloaded ZIP filename, the relevant UE4SS.log lines, and whether the motion file exists and updates during an HAnime.

### Downloads — always use the latest release

- **Motion Bridge (Windows x64 portable):** [latest release and download](https://github.com/Huarch/MotionBridge/releases/latest)
- **Fallen Doll Mod:** [all current releases and downloads](https://github.com/Huarch/MotionBridge-GameMod/releases) — download the newest package labelled **Demo** or **Playtest** for your game version.
- **Discord community:** [join the MotionBridge Discord](https://discord.gg/wc8mn4ejsz)

The relevant SHA-256 is published on each Release page. Verify it against the package you download.

### Discord community

For ongoing support and release discussion, [join the MotionBridge Discord](https://discord.gg/wc8mn4ejsz), then use these channels:

- [Announcements](https://discord.com/channels/1543526971910596649/1543529702558269470) and [download / install help](https://discord.com/channels/1543526971910596649/1543529706769354752)
- [Support and troubleshooting](https://discord.com/channels/1543526971910596649/1543529735399809036)
- [Bug reports](https://discord.com/channels/1543526971910596649/1543529729993482300) and [feature requests](https://discord.com/channels/1543526971910596649/1543529732015136838)
- [English community](https://discord.com/channels/1543526971910596649/1543529710267539496), [中文社区](https://discord.com/channels/1543526971910596649/1543529720413429910), and [showcase](https://discord.com/channels/1543526971910596649/1543546254787612752)

### Notes and limitations

- This is an unofficial community project; it is not affiliated with the game developers or hardware vendors.
- The packages do not include game assets, device drivers, or Intiface Desktop.
- Intiface support currently uses the first declared **Position** feature and maps it from `L0`; it is not a generic SR6 mapping.
- Direct Handy output checks the connection and firmware, enters HDSP mode, and sends timed `/hdsp/xpt` position targets. It coalesces `L0` into the newest target and derives duration from the real cloud dispatch interval, so stale game-motion requests do not accumulate. MultiFunPlayer was used only as a source-code reference and is not required.
- New or unusual scenes should always be checked in the preview first. Set conservative output ranges for your hardware.

---
## 中文

这次更新采用两个组件配合使用：

- **Fallen Doll Mod** 随游戏运行，输出精简的实时动作数据。
- **Motion Bridge** 是新的独立 Windows 软件，读取动作数据，提供六轴数值与 3D 设备预览，并将信号发送到设备。

两者需要配合使用。

游戏页面：
https://store.steampowered.com/app/1685960/
https://store.steampowered.com/app/1811180/

### Motion Bridge

- SR6 实时六轴 `L0/L1/L2/R0/R1/R2`；OSR2 使用 `L0`。
- 支持 USB TCode、Wi-Fi UDP TCode（默认 `tcode.local:8000`）、Intiface Desktop 和 Handy 直连输出。
- 兼容 Intiface Central 2.6。
- 独立的 SR6/OSR 实时 3D 预览窗口，可置顶显示在游戏上方。
- 每轴增益与输出范围、带双点曲线的跨轴智能限制、硬性速度限制、输出开关和安全位置设置；智能限制可根据所选驱动轴的位置，动态缩小另一个轴的行程或响应速度。即使硬件输出未解锁，轴卡和 3D 预览也会显示最终处理后的信号。便携版会保存设置。
- 安全启动：设备输出默认未解锁。数据流中断时会短暂停留，然后平滑回中。
- 支持英文和简体中文界面。
- 请按游戏版本选择 Fallen Doll Demo 或 Fallen Doll Playtest 对应的 Mod 分支。
- Mod 还额外支持类人动作。

### 安装与使用

1. 根据游戏版本下载最新的 **Fallen Doll Mod 发布包**，然后关闭游戏。
2. 解压 Mod 包。运行 `Install-Mod.ps1` 并选择游戏目录，或将包内 `Game` 文件夹中的内容复制到游戏目录下的 `Paralogue/Binaries/Win64`。确认 Mod 已启用，能够写入 `fd-skeleton.ndjson` 动作数据。
3. 下载并解压最新版 **Motion Bridge** 便携包，然后启动 `MotionBridge.exe`。
4. 启动 Fallen Doll 并进入 HAnime。在 Motion Bridge 中等待 **STREAM** 显示为 **ONLINE**。
5. 打开 3D 预览，先确认动作正确，再连接设备。
6. 选择一种输出方式：**USB**、**Wi-Fi**、**Intiface** 或 **Handy**。确认端口/地址，为设备设置安全的各轴范围后，再手动点击 **ARM OUTPUT**。Handy 直连需要填写 Handyverse 连接密钥；密钥只会保留到 Motion Bridge 关闭。

预览未确认正确前，请不要解锁输出。更换硬件或测试新姿势前，请先停止/解除输出。

遇到问题请在本帖留言，并尽量提供游戏版本、场景/动作和相关错误信息。

### 排错 — STREAM WAITING

MotionBridge 和 Mod 之间不是网络连接。Mod 会写入本地动作文件，MotionBridge 读取同一个文件：

`%USERPROFILE%\.f8\studio\games\fallen-doll\runtime\fd-skeleton.ndjson`

**STREAM WAITING** 表示 MotionBridge 最近没有收到新的有效骨骼帧。请按以下顺序检查：

1. 下载与实际游戏版本完全匹配的 Mod 发布包。不要因为它是仓库的 Latest Release 就假定适用于 Demo、Playtest 或另一套安装。
2. 确认 Mod 安装在游戏 EXE 所在目录。正常安装应包含 `Paralogue\Binaries\Win64\dwmapi.dll`、`ue4ss\UE4SS.dll` 和 `ue4ss\Mods\fd_tcode_probe\Scripts\main.lua`。
3. 启动游戏后打开 `Paralogue\Binaries\Win64\ue4ss\UE4SS.log`。它可区分 UE4SS 未注入与 Lua Mod 未加载两种情况。
4. 进入已识别的 HAnime，再检查 `fd-skeleton.ndjson` 是否存在，以及动作运行期间修改时间是否持续变化。
5. 检查旧的 `F8STUDIO_GAMES_DIR` 或 `FD_TCODE_RUNTIME_DIR` 环境变量；它们会让 Mod 有意写入另一个运行目录。

反馈该问题时，请附上游戏版本、下载 ZIP 的完整文件名、相关 UE4SS.log 内容，以及 HAnime 运行时动作文件是否存在并持续更新。

### 下载 — 始终使用最新 Release

- **Motion Bridge（Windows x64 便携版）：** [最新 Release 与下载](https://github.com/Huarch/MotionBridge/releases/latest)
- **Fallen Doll Mod：** [当前全部 Release 与下载](https://github.com/Huarch/MotionBridge-GameMod/releases) —— 按游戏版本下载名称标有 **Demo** 或 **Playtest** 的最新包。
- **Discord 社区：** [加入 MotionBridge Discord](https://discord.gg/wc8mn4ejsz)

每个 Release 页面都会发布对应的 SHA-256；请按实际下载的包进行校验。

### Discord 社区

请先[加入 MotionBridge Discord](https://discord.gg/wc8mn4ejsz)，再按问题类型进入对应频道：

- [公告](https://discord.com/channels/1543526971910596649/1543529702558269470) 与 [下载 / 安装求助](https://discord.com/channels/1543526971910596649/1543529706769354752)
- [支持与排错](https://discord.com/channels/1543526971910596649/1543529735399809036)
- [Bug 反馈](https://discord.com/channels/1543526971910596649/1543529729993482300) 与 [功能建议](https://discord.com/channels/1543526971910596649/1543529732015136838)
- [英文社区](https://discord.com/channels/1543526971910596649/1543529710267539496)、[中文社区](https://discord.com/channels/1543526971910596649/1543529720413429910) 与 [展示区](https://discord.com/channels/1543526971910596649/1543546254787612752)

### 说明与限制

- 这是非官方社区项目，与游戏开发商和设备厂商没有关联。
- 发布包不包含游戏资源、设备驱动或 Intiface Desktop。
- Intiface 当前只使用设备声明的第一个 **Position** 功能，并由 `L0` 驱动；它不是通用的 SR6 映射。
- Handy 直连会先检查连接和固件、进入 HDSP 模式，再发送 `/hdsp/xpt` 定时位置目标。程序只保留最新 `L0`，并按实际云端发送间隔计算到达时长，避免旧的游戏动作请求积压。MultiFunPlayer 仅作为公开源码参考，运行时不需要安装或连接。
- 遇到新动作或特殊姿势，请先在预览中确认，并为自己的设备使用保守的输出范围。
