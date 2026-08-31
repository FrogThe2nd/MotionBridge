# Motion Bridge

这是面向多款游戏的独立 Windows 运动桥接程序，将实时游戏动作转换为 OSR2/SR6 等多轴设备输出。Fallen Doll 是首个内置适配器；正常使用时不需要启动 F8Studio。

## 当前能力

- 独立、低延时的 C++20 运动与安全引擎；界面不阻塞实时输出。
- Fallen Doll Playtest、Demo、0.49 共用的 `fd-skeleton.ndjson` 输入。
- L0/L1/L2/R0/R1/R2、双手/双脚参考、增益（围绕所选中心增减输出行程）、中心、死区、曲线、范围、反向与失联回中逻辑；每个轴都可独立设置偏好区间，将稳定往返的两个端点映射到可调最小值–最大值，只补足短行程并为额外动作保留余量。平移轴自动倍率最高 4×，旋转轴采用更保守的 2× 上限。
- USB 串口、Wi-Fi UDP（默认 `tcode.local:8000`）、Intiface Desktop 与 Handy 云端直连；初始状态始终未解锁输出。
- 四种输出共用最终处理后的轴信号；USB/Wi-Fi 按真实间隔生成 TCode，Handy 则检查连接与固件后进入 HDSP 模式，将实时 L0 合并为最新的 `/hdsp/xpt` 定时位置目标，并按实际云端发送间隔控制到达时长，避免旧动作请求积压。MultiFunPlayer 仅作为公开源码实现参考，运行时不需要安装或连接。
- 高级设置支持软启动、跨轴智能限制、逐轴速度限制、输出开关、安全位置和可选的全局安全距离。Reference 与 Target 尚未靠近时，初始化姿势不会进入六轴处理或偏好区间学习。智能限制可选择驱动轴，并通过两个自由控制点动态调整当前轴保留的行程或响应速度；轴卡数值与 3D 预览始终显示最终处理信号，不受硬件输出是否解锁影响。
- 外部桌面 SR6/OSR 圆柱预览，VR 游戏运行时同样可在桌面查看。
- 顶栏可独立选择界面缩放（跟随系统、75%、90%、100%、110%、125%），设置在下次启动时生效，方便高 DPI 屏幕使用更紧凑的布局。
- `adapters/motion-frame-v1.schema.json` 是未来游戏的公开输入协议。外部适配器只需向所选 NDJSON 文件逐行写入完整帧，不加载第三方 DLL。

## 使用

1. 保持现有 Fallen Doll UE4SS Mod 已安装并启用骨骼流。
2. 启动 Motion Bridge，确认“Fallen Doll 数据流已连接”。
3. 在输出区选择 USB、Wi-Fi、Intiface 或 Handy，并确认端点、端口或 Handyverse 连接密钥。
4. 先观察预览与波形，再手动点击 **ARM OUTPUT**。停止或断流会先保持 250 ms，再在 600 ms 内回中；重新启动始终未解锁。

F8Studio 仍可作为工程编辑和调试工具，与 Motion Bridge 并行，不会被修改。

## 构建便携版

```powershell
.\tools\Install-MotionBridgeToolchain.ps1
.\tools\Build-MotionBridge.ps1 -Portable
```

便携目录和 ZIP 会生成在 `dist/`。安装包脚本将在发布阶段以同一个已部署的便携目录生成；当前不把未测试的设备输出作为发行前置条件。
