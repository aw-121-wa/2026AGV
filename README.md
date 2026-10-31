# 2026AGV

STM32H750 + FreeRTOS 麦轮 AGV 固件：HWT101 陀螺仪（UART5 DMA）、4 台 ZDT_X42S 闭环步进（FDCAN2，X 固件）、USART6 命令/遥测、栅格地图 + A* + 中国邮路巡检。

## 目录

```
Core/ Drivers/ Middlewares/   CubeMX 生成代码（只在 USER CODE 区域内修改）
User/
  Config/      agv_config.h：运行模式、出发区、容差等全局配置
  Common/      user_math.h：角度/限幅等公共工具
  Drivers/     硬件驱动：HWT101、ZDT_X42S
  Chassis/     底盘运动学、里程计、定距/转向闭环
  Comm/        串口命令解析、遥测、HAL UART 回调分发
  Navigation/  地图（构型空间位图）、A*、巡检路线
  App/         巡检状态机 agv_run
tests/
  host/        PC 端单元测试与 HAL 假实现
  run_tests.py 一键构建并运行全部 host 测试
```

依赖方向：App → Navigation/Chassis → Drivers；Comm 依赖 Chassis 与 App；Drivers 不依赖上层。

## 构建

```bash
cmake --preset Release          # 需要 arm-none-eabi-gcc、cmake、ninja
cmake --build build/Release
```

运行模式和出发区默认取 `User/Config/agv_config.h`，也可在配置时覆盖：`cmake --preset Release -DAGV_MODE=1 -DAGV_START_ZONE=2`。

| AGV_MODE | 含义 |
|---|---|
| 0 COMMAND | 串口指令模式（F/B/L/R/A/T/S） |
| 1 SURVEY | 纯底盘跑全图巡检，READY 后 3 s 自动出发 |
| 2 COMPETITION | 预留 |

## 测试

```bash
python3 tests/run_tests.py
```

## RTOS

| 任务 | 优先级 | 栈 | 说明 |
|---|---|---|---|
| IMUTask | High | 1 KB | UART5 中断通过线程标志唤醒，10 ms 超时兜底检查 DMA 重启 |
| ChassisTask | AboveNormal | 2 KB | 5 ms 周期：底盘闭环 → 串口指令/遥测 → 巡检状态机 |

`configCHECK_FOR_STACK_OVERFLOW = 2`，溢出时 `vApplicationStackOverflowHook` 先给四个电机发停止再停机，任务名存在 `stack_overflow_task`。共享数据用 `User/Common/critical.h`（FreeRTOS BASEPRI 临界区）保护。

## 硬件约定

- ZDT_X42S 使用 **X 固件**，校验字节固定 0x6B，速度 0.1 RPM、位置 0.1° 精度。
- FDCAN2 为 **1 Mbps**（120 MHz 内核时钟，Prescaler 6，Seg1 15，Seg2 4，SJW 4，采样点 80%）。驱动器出厂是 500K，**每台都要在菜单里把 `CAN_Baud` 改成 1M**（手册选项 09），否则无法通信；总线两端各接 120Ω 终端电阻。
- 四轮速度命令带同步标志缓存，再广播 `00 FF 66 6B` 同时执行；广播只有地址 1 应答。
- D-Cache/I-Cache 已开启。HWT101 DMA 缓冲区固定在 RAM_D2 起始 256 B，由 `MPU_Config` region1 设为非缓存（配置已写入 `2026AGV.ioc`，重新生成代码会保留）。
