# 2026AGV

| 目录 | 内容 |
|---|---|
| `firmware/` | STM32H750 + FreeRTOS 单片机代码（CMake 工程，用 VS Code 打开此目录） |
| `vision/` | Python 视觉识别（6 色物料、圆环靶心）+ 激光雷达障碍物检测，串口发给单片机 |
| `tools/` | `agv_console.html`：车载固件的调试台，用 Chrome 或 Edge 打开，经串口调参数、看曲线、开车（说明见 `firmware/README.md` 4.5 节） |
