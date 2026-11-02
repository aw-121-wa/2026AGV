# 2026AGV

STM32H750 + FreeRTOS 麦克纳姆轮 AGV 固件。

- 底盘：4 台 ZDT_X42S 二代闭环步进（X 固件协议），经 FDCAN2 以 1 Mbps 驱动，轮速同步缓存 + 广播触发。
- 姿态：HWT101 陀螺仪，UART5 + DMA 循环接收，帧校验后得到航向角。
- 导航：2400 × 2400 mm 栅格地图（构型空间位图）+ 分步 A* + 中国邮路（奇度点匹配 + Hierholzer）生成覆盖 12 条通道的巡检回路；整圈用一条连续路径执行，不在每个航点停车。
- 交互：USART6 一条串口同时做命令输入和文本状态输出，串口助手就能用；调参和看曲线可以用带界面的调试台 `tools/agv_console.html`（4.5 节）。

---

## 1. 硬件与引脚

MCU 时钟：HSE 8 MHz → PLL1（M=2, N=240, P=2）→ **SYSCLK 480 MHz**，HCLK 240 MHz，APB1/2/3/4 均 120 MHz。
Flash 128 KB，DTCM 128 KB，AXI SRAM（`0x24000000`）512 KB，SRAM1~3（`0x30000000`）288 KB。
上电后可用调试器读 `g_hse_ok`（1 = 外部晶振起振）和 `g_sysclk_hz`（应为 480000000）确认时钟。

### 1.1 串口 / 总线一览

| 外设 | 引脚 | 参数 | 用途 | 中断 / DMA |
|---|---|---|---|---|
| **USART6** | PC6 = TX，PC7 = RX | 115200 8N1，无流控 | **命令 + 文本输出**：串口助手或调试台收发（见第 4 节） | USART6_IRQn 优先级 6；收发都用中断（`ReceiveToIdle_IT` / `Transmit_IT`），不占 DMA |
| **UART5** | PC12 = TX，PD2 = RX | 115200 8N1 | **HWT101 陀螺仪**专用，只收不发（TX 预留） | UART5_IRQn 优先级 6 + DMA1_Stream0 循环模式 + 空闲中断；DMA 缓冲 256 B 固定放 RAM_D2 |
| **FDCAN2** | PB12 = RX，PB13 = TX | 经典 CAN，**1 Mbps**，扩展帧 | 4 台 ZDT_X42S 电机；不设硬件滤波器，地址在任务里过滤 | FDCAN2_IT0 优先级 5；RX FIFO0 16 帧、TX FIFO 队列 16 帧 |
| **TIM6** | — | 1 ms 周期 | HAL 时基（`HAL_GetTick()` / `uwTick`） | TIM6_DAC_IRQn 优先级 15 |

> 没有单独的调试串口：`printf` 没有落地（`syscalls.c` 的 `__io_putchar` 是弱符号），调试信息一律走 USART6 文本输出或调试器变量。

### 1.2 电机地址（按实车接线修改 `User/Drivers/ZDT_X42S/zdt_x42s.h`）

| 地址 | 位置 | 说明 |
|---|---|---|
| 1 | LF 左前 | `ZDT_ADDR_LF` |
| 2 | RF 右前 | `ZDT_ADDR_RF` |
| 3 | LR 左后 | `ZDT_ADDR_LR` |
| 4 | RR 右后 | `ZDT_ADDR_RR` |
| 0 | 广播 | 只用于同步触发 `00 FF 66 6B`，不能做节点地址 |

接线要点：

- 驱动器出厂 CAN 波特率是 500 K，**每台都要在菜单里把 `CAN_Baud` 改成 1M**（手册选项 09），否则收不到任何回复。
- 总线两端各接 120 Ω 终端电阻；四台电机并联在同一对 CANH/CANL 上。
- 轮径 77 mm、轴距 210 mm、轮距 230.4 mm 都是卡尺实测值，在 `chassis.h` 顶部修改。

---

## 2. 目录与文件职责

```
Core/ Drivers/ Middlewares/   CubeMX 生成代码，只在 USER CODE 区域内改
User/                         应用代码（可整体移植）
tests/host/                   PC 端单元测试 + 假 HAL
```

### 2.1 User/（手写代码）

| 文件 | 职责 |
|---|---|
| `User/Config/agv_config.h` | 全局配置：运行模式、出发区、巡检圈数、速度/容差/超时、`AGV_RAM` 段宏。带 `#ifndef` 的项可用 CMake `-D` 覆盖 |
| `User/Common/critical.h` | FreeRTOS BASEPRI 临界区封装（`Critical_Enter/Exit`），任务和中断里都能用、可嵌套 |
| `User/Common/user_math.h` | 角度归一化 `Math_WrapDeg`、限幅 `Math_Clamp`（纯 header，inline） |
| `User/Drivers/HWT101/hwt101.c` | 陀螺仪驱动：DMA 循环缓冲 → 软件环形队列（带接收时间戳）→ 11 字节帧校验 → 更新 `imu_data`；处理队列溢出与串口错误重启 |
| `User/Drivers/ZDT_X42S/zdt_x42s.c` | 步进驱动：扩展帧分包打包（每包 ≤ 7 字节数据、末包带 0x6B 校验）、TX FIFO 余量检查、RX 中断入队、功能码解析（速度/位置/状态/应答） |
| `User/Chassis/chassis.c` | 底盘：DWT 微秒时基、四轮 X 型运动学（含等比限幅）、四轮位置差分里程计、状态机（IDLE/READY/FAULT）、速度斜坡、使能与丢帧重发、定距/转向闭环、航向 PI |
| `User/Chassis/chassis_path.inc` | 连续航点跟踪（**被 `chassis.c` include，不单独编译**）：贝塞尔拐角过渡、按剩余距离限速、航点推进、偏线/里程/航向统计 |
| `User/Comm/chassis_cmd.c` | 命令解析：字符状态机（支持一帧拆包、`\r\n`/空格/空闲结束）、整行校验通过才提交、`S` 立即停车/清故障、`W/I/D/H` 即时命令、环形接收队列与溢出处理 |
| `User/Comm/chassis_telemetry.c` | 串口文本输出：2 KB 环形发送缓冲 + 中断分块发送；开机信息、状态变化、命令回执、周期状态行、路径完成统计、单轮测试结果、帮助与诊断、绘图数据流 |
| `User/Comm/console.c` | 串口控制台：行首 `/` 的命令、运行时参数表、台架调试命令（见 4.4 节） |
| `User/Comm/uart_callbacks.c` | HAL 串口回调统一分发到 HWT101 / 命令解析 |
| `User/Navigation/map.c` | 2400 × 2400 mm 栅格（10 mm 一格）、构型空间位图（7.2 KB，`Map_CanStand` O(1)）、圆形障碍、地图版本号 |
| `User/Navigation/astar.c` | 分步 A*：每拍只扩展 32 个节点，先分块清窗口再搜索；四张表放 `.agv_ram`，路径回溯复用堆数组 |
| `User/Navigation/survey.c` | 巡检路线：12 条通道建模成 3×3 网格多重图，奇度点最小权完美匹配补重复边，Hierholzer 求欧拉回路（16 段），给出每段起终点 |
| `User/App/agv_run.c` | 巡检状态机：等 READY → 出库 → 规划覆盖回路 → 整圈 `Chassis_FollowPath` 执行 → 入库；航向/偏线/地图版本/超时保护，`FollowPath` 忙时重试 15 s |

依赖方向：`App → Navigation / Chassis → Drivers`；`Comm` 依赖 `Chassis` 与 `App`；`Drivers` 不依赖上层。

### 2.2 Core/（CubeMX 生成，勿手改生成区）

| 文件 | 职责 |
|---|---|
| `Core/Src/main.c` | 启动顺序：MPU → Cache → HAL → 时钟 → 外设初始化 → 驱动/底盘/命令/巡检初始化 → 启动调度器；时钟自检符号 |
| `Core/Src/freertos.c` | 创建两个任务、栈溢出钩子、IMU 线程标志通知 |
| `Core/Src/usart.c` | UART5 / USART6 初始化、GPIO 复用、UART5 的 DMA 与 NVIC |
| `Core/Src/fdcan.c` | FDCAN2 1 Mbps 位时序、FIFO 深度、GPIO 与 NVIC |
| `Core/Src/dma.c` | DMA1 时钟与 NVIC 使能 |
| `Core/Src/gpio.c` | GPIO 时钟与上电电平 |
| `Core/Src/stm32h7xx_it.c` | 异常与中断向量（转 HAL 处理） |
| `Core/Src/stm32h7xx_hal_msp.c` | 全局 MSP（SYSCFG 等） |
| `Core/Src/stm32h7xx_hal_timebase_tim.c` | TIM6 作为 HAL 时基 |
| `Core/Src/syscalls.c` / `sysmem.c` | newlib 系统调用与 `_sbrk` 堆桩 |
| `Core/Src/system_stm32h7xx.c` | `SystemInit` / `SystemCoreClock` |

### 2.3 tests/host/（PC 端）

| 文件 | 覆盖内容 |
|---|---|
| `main.h` / `critical.h` / `fdcan.h` / `usart.h` | 假 HAL：类型、句柄、DWT、Tick、临界区 |
| `zdt_test.c` | CAN 打包/分包/同步缓存/FIFO 余量/0.1 RPM 与 0.1° 解析/状态位/丢帧计数/解除堵转/Bus-Off 恢复 |
| `imu_dma_test.c` | DMA 环形 + 空闲/半满中断、重复事件、帧校验、溢出、错误重启、时间戳 |
| `map_test.c` | 地图边界、禁区、障碍、构型空间查询 |
| `planner_test.c` | A* 起点终点、走廊、绕障 |
| `survey_test.c` | 中国邮路覆盖全部 12 条通道 |
| `chassis_test.c` | 定距/转向闭环、世界坐标、命令解析、文本输出、反馈丢失、超时、停止重发、DWT 回绕 |
| `bench_test.c` | 单轮测试顺序/判定/中断、堵转保护与掉使能恢复、故障原因文本、DWT 解锁；`-DCHASSIS_USE_IMU=0` 版本验证无陀螺仪航向推算 |
| `path_test.c` | 连续过弯全程不出现零速、偏线在 5 mm 内、直线合并与原地转向、直线跑满设定速度；偏线只量正在走的段（路线重合、原路折返、参考折线丢点） |
| `command_path_test.c` | 运动中命令排队、分片命令、`S` 清队、错误行原子性、`DONE path` 统计只打印一次、长指令流追加 |
| `agv_test.c` | 巡检整圈覆盖/回库/停锁/占领出发区/底盘故障/碰墙/地图失效/航向超限/总超时/BUSY 重试（假底盘） |
| `survey_loop_test.c` | 真实底盘代码 + 巡检状态机跑完整圈：两个出发区都到 `AGV_DONE`、12 条通道全覆盖、偏线和回库残差、调试台读的 `AGV` 行格式 |
| `mode_test.c` | 三种 `AGV_MODE` 下的命令门控、`S`、接收溢出 |
| `console_test.c` | 控制台：改参数后真实控制输出跟着变、错误输入、停车才能改的参数、行结束方式、接收溢出、绘图频率和通道、调试台依赖的输出格式；另编一份验证无陀螺仪版本 |
| `task_test.c` | 任务码解析：规则示例、颜色编号、第二批码垛环号、各类非法码、模块字节流分行 |
| `comp_task_test.c` | 比赛模式任务码通道：扫码模块和相机文本经 `competition.c` 进入 `comp.task`，错码不覆盖好码；控制台的 `/task`、`/vision`、`/obs`、`/pose` 和雷达参数；调试台读的 `COMP` 行格式 |
| `../vision_protocol_test.py` | 上位机颜色编号和 TASK 帧字节（与 `comp_test.c` 解码的帧相同） |

---

## 3. 运行模式与状态机

| AGV_MODE | 含义 |
|---|---|
| 0 COMMAND | 串口指令模式，底盘按 USART6 命令走（默认限速 100 mm/s） |
| 1 SURVEY | 纯底盘跑全图巡检，READY 且等待 3 s 后自动出发（150 mm/s） |
| 2 COMPETITION | C1 雷达障碍识别 + 视觉上位机串口通信 + 扫码模块读任务码（见第 9 节）；取放料流程待接入，目前底盘用串口命令手动开 |

底盘模式（`chassis.mode`）：`VEL` 速度模式 / `MOVE` 定距 / `TURN` 转向 / `PATH` 连续航点。

巡检状态机（`AgvState`）：

```
WAIT ──READY+3s──> DOCK_OUT ──> PLAN(需要时) ──> DRIVE ──> DOCK_IN ──> DONE
  └──15s 超时──┐        └──碰障/位姿/地图/底盘/超时──> FAULT
```

- `DOCK_OUT`：出发区中心 → 通道口（两段 L 形），并为整圈航点做碰撞预检。
- `DRIVE`：逐段规划（通道内无碰撞直接走直线，否则在走廊内做 A*），把航点**累积**成一条路径。
- `DOCK_IN`：航点全部就绪后一次性 `Chassis_FollowPath()` 执行整圈，随后入库。
- `executing` 期间持续检查：偏线 > 8 mm、地图版本变化、位姿越界、整圈总超时。
- 完成后 `agv.covered` 是规划覆盖的通道位图，`agv.home_residual_mm` 是里程计回库残差（只上报，不判故障）。

---

## 4. 串口交互（USART6）

所有输入输出都是可读的 ASCII 文本，任何串口助手（XCOM、SSCOM、MobaXterm、PuTTY、VOFA+ 的串口文本模式等）都能用；要界面就用调试台（4.5 节）。

串口助手设置：**115200、8 数据位、无校验、1 停止位、无流控**；接收区用"文本/ASCII"显示（不要勾 HEX）；发送时勾不勾"加回车换行"都可以（没有换行时靠串口空闲中断判定一条结束）。输出全是英文 ASCII，GBK / UTF-8 都不会乱码。

### 4.1 命令

| 命令 | 含义 | 例子 |
|---|---|---|
| `F<mm>` `B<mm>` `L<mm>` `R<mm>` | 前 / 后 / 左 / 右平移，整数 1~5000 mm | `F300` |
| `T<deg>` | 相对转角，±180°，最多 3 位小数 | `T90`、`T-45.5` |
| `A<deg>` | 转到绝对航向 | `A0` |
| `S` | 立即停车；**也是清故障命令**（自动解除堵转保护、重新使能） | `S` |
| `W` / `W1`~`W4` | 单轮方向测试（全部 / 只测 LF、RF、LR、RR）。**先把车架起来让轮子离地** | `W` |
| `I` | 打印详细诊断：IMU、每个电机的使能/堵转/收帧计数、CAN 计数 | `I` |
| `D<hz>` | 周期状态行频率：`D0` 关闭，`D1`~`D10` Hz（默认 1 Hz） | `D5` |
| `H` 或 `?` | 打印帮助 | `?` |

规则：

- 不分大小写；一行可以写多条，用空格分隔，按顺序执行：`F300 L200 B300 R200`。
- **整行原子**：整行解析通过才提交，有一处错就整行不执行，回 `ERR`。
- 运动中发新的移动命令会**排队追加**到当前路径；`S` 立即生效并丢弃该行剩余内容。
- 指令模式移动速度 100 mm/s（`agv_config.h` 的 `CHASSIS_COMMAND_SPEED_MM_S`）。
- 巡检模式下只接受 `S`、`I`、`D`、`H` 和控制台命令（4.4 节），其余回 `BUSY`。比赛模式还没有比赛流程，移动命令和 `W` 都能用；加上比赛流程后运行中必须锁掉，规则禁止遥控。

### 4.2 串口输出

| 开头 | 什么时候出现 | 含义 |
|---|---|---|
| `====` | 上电 | 开机信息：运行模式、是否用陀螺仪 |
| `EVT` | 状态变化 | `IDLE -> READY`、`READY -> FAULT: <原因>` 等，原因会具体到哪个轮子 |
| `[  12.3]` | 周期（默认每秒一行） | 时间(s) 状态 模式 里程计 x/y(mm) 航向/目标/误差(°) 下发速度 vx/vy(mm/s)/wz(°/s)，四轮目标/实测转速(RPM)；IDLE 时后面跟 `waiting: ...` 说明还在等什么 |
| `OK` / `BUSY` / `ERR` | 每次发命令 | 命令回执；`OK` 只表示已接受 |
| `DONE path` | 一条路径走完 | 用时、里程、最大偏线、最大航向误差 |
| `WHEEL` / `DONE wheel test` | 单轮测试 | 正在转哪个轮子、编码器检查结果 |
| `---- info` | 发 `I` | 详细诊断 |

示例（上电 → 走一段 → 单轮测试 → 堵转）：

```text
==== 2026AGV chassis firmware | mode=COMMAND | IMU=HWT101 | CAN 1M ====
Type H (or ?) + Enter for help. Status line every 1000 ms (D0 = off).
EVT  IDLE: enabling motors, waiting for feedback
[    1.0] IDLE STOP x=0.0 y=0.0 yaw=0.00 ... | waiting: IMU:no-data LF:no-reply LF:disabled ...
EVT  IDLE -> READY: all 4 motors enabled, feedback OK
OK   path queued (100 mm/s)
[    2.0] READY PATH x=49.5 y=0.0 yaw=0.00 tgt=0.00 err=0.00 v=50/0/0.0 rpm(t/m) LF 12/12 RF 12/12 LR 12/12 RR 12/12
DONE path: time=8.11s dist=493.3mm max_cross=2.44mm max_heading=0.00deg
OK   wheel test started, 30 rpm 1000 ms per wheel
WHEEL RF spinning now (addr 2): it must roll FORWARD (top of the wheel moves toward the front). If it rolls backward set CHASSIS_WHEEL_DIR_RF to -1
WHEEL RF encoder: expect +180deg got 179.1deg (30.0 rpm) -> OK
DONE wheel test: encoders all OK. Direction = what you saw: every wheel must have rolled forward.
EVT  READY -> FAULT: feedback lost/motor fault: RF:stall. Send S to clear.
ERR  bad command 'Q' (H for help)
```

### 4.3 现场底盘单测步骤（只接电机，其余硬件没到也能测）

1. **CAN 与电机**：每台驱动器菜单里 `CAN_Baud` 设 1M、地址设 1~4（LF/RF/LR/RR），总线两端 120 Ω。
2. **没装陀螺仪**时编译：`cmake --preset Release -DCHASSIS_USE_IMU=0`（航向用轮子推算，只用于单测）。装了就用默认。
3. 上电看串口：应出现 `EVT IDLE -> READY`。一直停在 IDLE 就看状态行的 `waiting:`：
   - `LF:no-reply` → 这台电机没回帧：查 CAN 线、波特率、地址、电机供电；发 `I` 看 `rx_ok` 是否在涨、`bus_off` 是否在涨。
   - `LF:disabled` → 回帧了但没使能（驱动器 En 引脚/菜单设置）。
   - `IMU:no-data` → 陀螺仪没数据；没装就按第 2 步编译。
4. **车架起来，轮子离地**，发 `W`。四个轮子依次各转 1 秒，**眼睛看每个轮子是不是往"车前进"的方向滚**。往后滚的轮子，在 `chassis.h` 把对应 `CHASSIS_WHEEL_DIR_xx` 改成 `-1`（或 CMake `-DCHASSIS_WHEEL_DIR_RF=-1`），重新编译再测，直到四个都往前。编码器检查给 `NO MOTION` 就查使能/接线。（左右电机一般镜像安装，通常右侧两个要改 -1。）
5. 放回地面，低速试：`F200`、`B200`、`L200`、`R200`，看车是不是往对的方向走；再 `T90`、`T-90` 看转向方向（逆时针为正）。转反了改 `CHASSIS_YAW_SIGN`。
6. 组合：`F500 L500 B500 R500` 走一个方框，看 `DONE path` 的 `max_cross`、回到原点的 x/y。
7. 出现 `FAULT` 时串口会写原因，处理后发 `S` 清除（堵转保护会自动解除，电机会重新使能）。

### 4.4 串口控制台：实时改参数、看曲线

行首是 `/` 的一行是控制台命令，其他行还是上面的单字母命令。三种模式都能用，不用重新烧录。

| 命令 | 作用 |
|---|---|
| `/` 或 `/help` | 控制台帮助 |
| `/list [前缀]` | 列出参数：当前值、范围、默认值。例：`/list heading` |
| `/set <名字> <值>` | 检查范围后立即生效，回显旧值和新值。例：`/set heading_kp 2.5` |
| `/reset` | 全部恢复默认值，只能在停车时用 |
| `/plot <heading\|speed\|wheels> [hz]` | 输出绘图数据，1~100 Hz，默认 50；`/plot off` 停止 |

比赛模式（`AGV_MODE=2`）另有台架调试命令：

| 命令 | 作用 |
|---|---|
| `/task <任务码>` | 当作扫到了这个码，走同一套校验。没有扫码模块时也能测后面的流程 |
| `/vision <模式> [颜色]` | 设置上位机视觉任务：0 关、1 二维码、2 物料、3 圆环；颜色 0 任意，1~6 同规则编号 |
| `/obs`、`/obs clear` | 列出雷达障碍物；停车时清除（同时重建地图、通知上位机） |
| `/pose <x> <y> <航向>` | 把车搬到某处后设定场地位姿（mm、度），需要底盘 READY |

**参数**（`CHASSIS_MAX_WHEEL_RPM`、`CHASSIS_MOVE_MAX_SPEED` 是硬限制，不开放修改）：

| 生效时机 | 名字 | 默认 | 范围 | 含义 |
|---|---|---|---|---|
| 下一拍（5 ms） | `heading_kp` | 2 | 0~20 | 航向比例增益 |
| 下一拍 | `heading_ki` | 1 | 0~20 | 航向积分增益；设成 0 同时清掉积分 |
| 下一拍 | `heading_i_max` | 5 | 0~60 | 积分输出上限，deg/s |
| 下一拍 | `heading_i_zone` | 8 | 0~180 | 误差小于此值才积分，度 |
| 下一拍 | `turn_max_speed` | 90 | 1~240 | 航向环输出限幅，deg/s |
| 下一拍 | `distance_kp` | 1.5 | 0.1~10 | 定距 P 增益，1/s |
| 下一拍 | `distance_tol` | 3 | 0.5~50 | 到位容差，mm |
| 下一拍 | `heading_tol` | 0.1 | 0.01~10 | 航向到位容差，度 |
| 下一拍 | `settle_ms` | 200 | 0~2000 | 停稳多久算完成，ms |
| 下一拍 | `acc_linear` | 2000 | 100~5000 | 线加速度，mm/s² |
| 下一拍 | `acc_angular` | 720 | 30~3000 | 角加速度，deg/s² |
| 下一条路径 | `cmd_speed` | 100 | 10~400 | 串口移动命令的速度，mm/s；正在走的路径保持原限速 |
| 只能停车改 | `wheel_dia` | 77 | 40~150 | 轮径，mm（标定里程计） |
| 只能停车改 | `chassis_k` | 220.2 | 100~400 | 旋转半径系数，(轴距+轮距)/2，mm |
| 只能停车改 | `path_deviation` | 5 | 0.5~50 | 拐角允许偏差，mm |
| 下一个雷达点（比赛模式） | `lidar_x`、`lidar_y` | 0 | ±200 | 雷达安装位置，mm |
| 下一个雷达点 | `lidar_yaw` | 0 | ±180 | 雷达安装角，度 |
| 下一个雷达点 | `lidar_min`、`lidar_max` | 200、1800 | 50~500、500~6000 | 参与识别的距离范围，mm |
| 下一个雷达点 | `obs_gap` | 45 | 5~200 | 同一簇相邻点最大间距，mm |
| 下一个雷达点 | `obs_width` | 95 | 20~300 | 簇宽上限，mm |
| 下一个雷达点 | `obs_match` | 60 | 10~300 | 跨圈关联距离，mm |
| 下一个雷达点 | `obs_confirm` | 4 | 1~50 | 连续命中多少圈才确认 |

**怎么用：**

- **值只在 RAM 里，断电就没了。** H750VB 内部 Flash 只有一个 128 KB 扇区，放的就是程序，擦它存参数会把程序一起擦掉。调试台会记下改过的值，上电后提示写回；不用调试台时把 `/set` 命令存成文本贴回去。定稿后把数值写回 `chassis.h`、`agv_config.h`、`comp_config.h` 的宏。
- **发送方式：** 一次发一整行。串口助手勾不勾“加回车换行”都行，不加时靠串口空闲判定一行结束。PuTTY 这类终端要打开本地行编辑（Terminal → Local line editing = Force on），否则每敲一个字符就算一行。
- **控制台行里的 `s` 不算停车，** 否则 `/set heading_tol ...` 会把车停下。
- **绘图：** 每行是 `plot:a,b,c`，开始时会回复每一列的含义。`heading` 是目标航向、航向、误差、目标角速度、下发角速度；`speed` 是目标、下发、实测的 vx 和 vy；`wheels` 是四轮目标和实测转速（LF RF LR RR）。调参时先发 `D0` 关掉状态行。115200 波特约 11 KB/s，一行约 50 字节，`/plot wheels 100` 也跟得上。这种带标签的逗号分隔文本就是 VOFA+ FireWater 模式的格式（还没用 VOFA+ 实测）。
- **无线调试：** USART6 接一个串口无线模块，车跑起来也能用控制台，固件不用改。规则允许和笔记本通信，但运行中不能遥控，所以比赛流程加上后要在运行时锁掉 `/set`、运动命令和台架命令。

### 4.5 调试台（`tools/agv_console.html`）

一个网页文件，不联网、不用安装。USB 转串口接 USART6，用电脑版 Chrome 或 Edge 打开这个文件，点“连接串口”选端口。

如果浏览器不让这个页面用串口（页面会提示），在 `tools` 目录起一个本地服务，再打开 `http://localhost:8000/agv_console.html`：

```bash
python -m http.server 8000
```

| 区域 | 内容 |
|---|---|
| 顶栏 | 连接、波特率、底盘状态、运行模式；**急停 / 清故障**（快捷键 Esc，发 `S`） |
| 参数 | 从 `/list` 读出来，固件加参数不用改网页。滑块和数字框，超范围当场拒绝，车在动时锁住“只能停车改”的项；导出、导入 JSON |
| 实时曲线 | 航向环、车速、轮速，10~100 Hz，窗口 5~60 s；悬停看数值，点图例隐藏通道，暂停，导出 CSV |
| 终端 | 固件的全部文本，能发任何命令，↑↓ 翻历史；状态行和曲线数据默认隐藏 |
| 底盘状态 | 位姿、速度、四轮转速、故障原因、上一条路径的统计 |
| 运动 | 前后左右、转向、转回 0°、单轮测试（巡检模式下禁用） |
| 巡检（`AGV_MODE=1`） | 巡检状态和错误原因、覆盖通道数、回库残差；场地图上画车的位置。巡检故障时底盘仍是 READY，要看这里和顶栏 |
| 比赛（`AGV_MODE=2`） | 场地图（车、障碍物，点图设定位姿）、任务码含义表（可手动输入）、视觉模式和颜色、雷达 / 上位机 / 扫码模块的健康计数 |

- 模式由固件决定：开机信息里有就用开机信息，中途连上时读完 `/list` 再推断。
- 改过的值记在这台电脑的浏览器里。连接或固件重启后，如果固件里的值和记录不同，页面提示“全部写入”。
- 连接时把状态行调到 5 Hz，正常断开时调回 1 Hz。
- 数据全部来自固件文本，解析规则集中在 `parseLine`；这些格式由 `console_test`、`comp_task_test`、`survey_loop_test` 逐字固定，改固件输出时测试会提醒。
- 网页本身没有自动测试，是接固件仿真（真实固件代码 + 假串口）手工跑的，**还没有接过真实串口和真车**。

## 6. 底盘与导航实现要点

- **运动学**：轮速 = 符号矩阵 × (vx, vy, K·ω)。`Mᵀ·M = 4I`，所以正解取 `Mᵀ/4`。超限时四轮同乘一个系数，保证合成速度方向不歪。
- **同步下发**：四个轮速带同步标志入队（`ZDT_SYNC_CACHE`），再广播一次 `00 FF 66 6B` 同时生效；命令变化才发，另有 100 ms 兜底重发。
- **里程计**：每 20 ms 同时读四轮位置，四轮都更新后才用位置差分累加位移；采样时刻存在 `s_odom_sample_ms[]`，用于 PATH 模式的 40 ms 内预测。
- **航向**：IMU 帧带接收时间戳，超过 500 ms 没新帧就判反馈失效。
- **连续路径**：拐角用二次贝塞尔在两侧各裁 `min(2×容差, 0.25×段长)`，入弯/出弯按向心加速度限速；推进条件是 `距离 ≤ 0.15 mm` 或 `已越过且距离 < 1 mm`，因此 2 mm 间距的航点也不会被跳过，全程不会每段停车。
- **偏线统计**：每个内部点记着自己属于哪个原始航点（`s_path_vertex`），偏线只量控制器正在走的那几段原始折线，含拐角两侧，所以贝塞尔裁掉的部分也算在内。不按位置找最近段：巡检路线会重复经过同一条边，那样会匹配到后面重合的段。参考点满 257 个时丢已走过的点；万一当前段被挤掉，退回量当前弦长并置截断标志。
- **A***：障碍只改构型空间位图（每格 10 mm），搜索窗口只清窗口内的表，每拍 32 节点，控制节拍不会被大块 memset 打断。
- **巡检路线**：12 条通道 = 3×3 网格的 12 条边，奇度点（4 个中点）两两配最短路径补 4 条重复边，得到 16 段欧拉回路，回到出发点。

---

## 7. 配置与编译期校验

`User/Config/agv_config.h` 里的配置（以及 `chassis.h` 里的 `CHASSIS_USE_IMU`、`CHASSIS_WHEEL_DIR_xx`）都可以在 CMake 配置时用 `-D` 覆盖（`CMakeLists.txt` 会把它们加进 `target_compile_definitions`）：

```bash
cmake --preset Release -DAGV_MODE=1 -DAGV_START_ZONE=2 -DAGV_SURVEY_LAPS=2
```

| 宏 | 默认 | 说明 |
|---|---|---|
| `AGV_MODE` | 0 | 运行模式 |
| `AGV_START_ZONE` | 1 | 出发区 1 或 2 |
| `AGV_SURVEY_LAPS` | 1 | 巡检圈数 1~8，只有最后一圈入库 |
| `AGV_STOP_EACH_SEGMENT` | 0 | 1 = 逐段停车（对照用），0 = 连续过弯 |
| `AGV_SURVEY_SPEED_MM_S` | 150 | 巡检速度；可 `-D` 覆盖，运行时钳位到 10~400 |
| `AGV_POSITION_TOL_MM` | 5 | 回库定位目标值，**只作统计** |
| `AGV_CROSS_TRACK_MM` | 8 | 路径跟踪偏线上限 |
| `AGV_HEADING_FAULT_DEG` / `_MS` | 3.0 / 200 | 航向持续超限多久判故障 |
| `CHASSIS_USE_IMU` | 1 | 0 = 不接陀螺仪，航向由轮子推算（只用于底盘单测） |
| `CHASSIS_WHEEL_DIR_LF/RF/LR/RR` | 1 | 电机安装方向，±1；用 `W` 测试后修改 |
| `CHASSIS_COMMAND_SPEED_MM_S` | 100 | 指令模式移动速度 |

文件里的写法是 `#ifndef X → 默认值 → #endif` 加 `#if … #error`：

- `#ifndef` 给**默认值**：命令行传了 `-DAGV_SURVEY_LAPS=2` 就不会重复定义（否则编译告警/报错），没传则用默认。
- `#if … #error` 做**编译期合法性检查**：0 圈、9 圈、逐段停车开关写成 2 这类组合直接编译失败，不会带上车跑。
- `AGV_RAM` 在 ARM 上展开成 `.agv_ram` 段属性，主机测试（x86）展开为空，同一份代码两边都能编。

---

## 8. 构建、烧录、测试

```bash
# 固件（需要 arm-none-eabi-gcc、cmake、ninja）
cmake --preset Release
cmake --build build/Release

# host 测试（需要 gcc、python）
python tests/run_tests.py                 # 默认输出到 build/host
# 输出目录不可写时换一个目录（任选其一）：
#   PowerShell: $env:AGV_HOST_BUILD="$env:TEMP\agv-host"; python tests/run_tests.py
#   bash:       AGV_HOST_BUILD=/tmp/agv-host python tests/run_tests.py
```

格式化（仓库自带 `.clang-format`，LLVM + Allman + 4 空格 + 100 列）：

```bash
build/format_tools/clang_format/data/bin/clang-format.exe -i User tests/host
```

烧录用 CubeIDE / OpenOCD / J-Link 均可，目标 `build/Release/2026AGV.elf`。

---

## 9. 内存与 RTOS

| 区域 | 起点 | 内容 |
|---|---|---|
| FLASH 128 KB | `0x08000000` | 代码与常量。比赛模式 Release 57%、Debug 96%；指令模式 Release 50%、Debug 86%；巡检模式 Release 50%。**Debug 快满了，日常用 Release** |
| DTCM 128 KB | `0x20000000` | `.data/.bss`、主栈、FreeRTOS 堆（15 KB）、任务栈（比赛模式 72%，其他 65%） |
| AXI SRAM 512 KB | `0x24000000` | `.agv_ram`：A* 四张表（`cost/heap/position` 2 B + `parent` 1 B，合计约 394 KB）+ 地图位图 7.2 KB（约 78%） |
| SRAM1 256 B | `0x30000000` | `.imu_dma`：UART5 的 DMA 缓冲，链接脚本断言必须在 `0x30000000` 且 ≤ 256 B |

> A* 表按 57600 格 × 7 B 计算，占满了 RAM 段的大部分，这也是 RAM 78% 的来源；要省 RAM 就减 `MAP_CELL_MM` 或缩小 `MAP_WIDTH_MM/HEIGHT_MM`。

| 任务 | 优先级 | 栈 | 周期 | 说明 |
|---|---|---|---|---|
| IMUTask | High | 1 KB | 事件驱动 | UART5 中断里 `osThreadFlagsSet` 唤醒，10 ms 超时兜底检查 DMA 重启 |
| ChassisTask | AboveNormal | 2 KB | 5 ms（`osDelayUntil`） | 底盘闭环 → 命令/遥测 → 巡检状态机，三者同一拍 |
| Timer/IDLE | 低 | 1 KB / 128 B | — | FreeRTOS 内置 |

`configCHECK_FOR_STACK_OVERFLOW = 2`：溢出时 `vApplicationStackOverflowHook` 先给四轮发停止，再停机，任务名留在 `stack_overflow_task` 供调试器查看。

实测栈占用（`-fstack-usage`，O0）：底盘任务最坏链约 **0.6 KB / 2 KB**（`ChassisTelemetry_Poll → PrintStatus 152 B` + `vsnprintf`，文本缓冲都是静态的），IMU 任务最坏约 0.35 KB / 1 KB（`HAL_UART_AbortReceive` 重启路径）。比赛模式多一条更深的链（Release）：`Comp_Tick 216 B → VisionTick 480 B → AcceptTask 72 B → Printf` + `vsnprintf`，约 1.1 KB / 2 KB。

---

## 10. 调试与已知限制

- 没有单独的调试串口；看状态用 USART6 文本输出（状态行、`I` 命令），或调试器看 `chassis`、`agv`、`chassis_path_result`、`chassis_wheel_result`、`stack_overflow_task`、`g_hse_ok`。
- 电机侧计数：`zdt_motor.rx_frames` / `rx_errors` / `ack_pending`，以及 `ZDT_RxDropped()`、`ZDT_BusOffCount()`（串口 `I` 命令都会打印）。
- 命令接收缓冲 32 B/包、环形队列 256 字符；文本发送缓冲 2 KB，满了丢弃新文本并计数（`I` 里的 `tx_dropped`）。
- 巡检整圈是一条 256 航点上限的路径（`CHASSIS_PATH_CAPACITY`），超出会报 `AGV_ERR_PATH`；8 圈约 132 个航点，仍在范围内。
- `S` 会丢弃本行剩余内容、"接收溢出"会丢弃到下一个结束符，这是有意的：宁可丢一行，也不执行半条路径。
- CAN 必须是 1 Mbps，且电机 `CAN_Baud` 要逐台改；否则轮子完全不动。
- 运动中分批发命令时，两批接缝处的拐角没有贝塞尔过渡，只靠出弯限速，理想模型下偏线约 7 mm，超过 5 mm 的拐角容差。要拐得准就写在同一行：`F300 L200`。
- 车总是停在到位容差的边上：`distance_tol` 默认 3 mm，理想模型下 `F200` 停在 197.2 mm，巡检回库残差约 2.9 mm。要停得更准就调小它。

---

## 11. 调参速查：PID 与限速

### 11.1 固件里一共就两个闭环

| 环 | 位置 | 对象 | 输出 |
|---|---|---|---|
| **航向 PI** | `chassis.c` 的 `Chassis_HeadingControl()` | `heading_error`（°） | `wz`（deg/s），限幅 `CHASSIS_TURN_MAX_SPEED` |
| **定距 P** | `chassis.c` 的 `Chassis_AutoControl()` | 剩余距离（mm） | `vx/vy`（mm/s） |

- 航向 PI 是**唯一**的 PID 型环，MOVE / TURN / PATH 三种模式共用；PATH 模式只是把 `heading_target` 按航段进度插值。
- PATH 模式的位置**不是 PID**：方向直接指向当前航点，速度由几何前馈 + 刹车曲线算出来（`chassis_path.inc`），没有位置积分。
- **轮速环在 ZDT_X42S 驱动器内部**（闭环步进自己的速度/位置环），固件只下发 0.1 RPM 目标，固件里没有它的 PID；要调在驱动器菜单里调。

### 11.2 改 PID：现场用 `/set`，定稿写回 `User/Chassis/chassis.h` 顶部这几个宏

下面的宏是开机默认值，运行时用 `/set` 改（4.4 节），不用重新烧录。

| 宏 | 当前值 | 作用 | 调大的效果 |
|---|---|---|---|
| `CHASSIS_HEADING_KP` | 2.0 | 航向比例增益，`(deg/s)/deg` | 回正更快，过大开始左右抖 |
| `CHASSIS_HEADING_KI` | 1.0 | 航向积分增益 | 消稳态误差更快，过大会超调/来回摆 |
| `CHASSIS_HEADING_I_MAX` | 5.0 | 积分输出上限（deg/s） | 允许更大的积分修正 |
| `CHASSIS_HEADING_I_ZONE` | 8.0 | 只在误差小于该值时才积分（°） | 大角度转向时更早开始积分 |
| `CHASSIS_TURN_MAX_SPEED` | 90 | 航向环输出限幅（deg/s） | 转向更快，也更容易过冲 |
| `CHASSIS_HEADING_TOL_DEG` | 0.1 | 静态"到位"门限（°） | 更容易判到位（不是增益，但影响 `AGV_DONE` 时机） |
| `CHASSIS_DYNAMIC_TOL_DEG` | 0.2 | 平移期航向误差统计门限（°），超了 `I` 里的 `over_0.2deg` 加一 | 只影响指标，不影响控制 |
| `CHASSIS_SETTLE_US` | 200000 | 停稳多久算完成（µs） | 完成判据更宽松 |
| `CHASSIS_DISTANCE_KP` | 1.5 | 定距 P 增益，`1/s` | 定距更快，过大会冲过头 |
| `CHASSIS_DISTANCE_TOL_MM` | 3.0 | 定距到位容差（mm） | 更容易到位 |

要改控制结构（加微分、换 PID、改积分抗饱和方式）就动 `Chassis_HeadingControl()` / `Chassis_AutoControl()` 本体。

调参顺序建议：

1. 先确认方向：手推车头偏左、状态行的 `yaw` 应该是**变大**（逆时针为正）。方向反了先改 `CHASSIS_YAW_SIGN`，这不是增益问题。
2. `CHASSIS_HEADING_KI = 0`，只加 `KP` 直到回正开始等幅抖，再退到一半。
3. 加 `KI` 到稳态误差消失，`I_ZONE` 取"正常平移误差的量级"（现在 8°，先不动）。
4. 发 `I` 看 `heading: peak_dynamic` 和 `over_0.2deg`：目标分别是 ≤0.2° 和 0，超了就是增益或加速度不匹配。
5. 改完跑 `python tests/run_tests.py`，`path_test` 会验证过弯不失速、偏线 ≤5 mm；`chassis_test` 验证 0.3° 小转向和 8° 扰动恢复。

### 11.3 改限速：先确定你要改的是哪一层

| 想要的效果 | 改哪里 |
|---|---|
| **指令模式**整体快慢（F/B/L/R 的默认速度） | 现场 `/set cmd_speed 150`；默认值是 `User/Config/agv_config.h` 的 `CHASSIS_COMMAND_SPEED_MM_S`（100 mm/s） |
| **巡检模式**整体快慢 | `User/Config/agv_config.h` 的 `AGV_SURVEY_SPEED_MM_S`（默认 150），也可以 `cmake -DAGV_SURVEY_SPEED_MM_S=200` 直接从命令行覆盖 |
| **绝对上限**（任何模式都过不去） | `chassis.h` 的 `CHASSIS_MAX_WHEEL_RPM`（400 RPM，解算后四轮等比缩放）；`Chassis_SetMoveSpeedLimit` 会把目标速度钳位到 10~400（`chassis.h` 的 `CHASSIS_MOVE_MAX_SPEED`） |
| **过弯速度**（现在 90° 弯 ≈50 mm/s） | 现场 `/set acc_linear`、`/set path_deviation`；默认值是 `chassis.h` 的 `CHASSIS_ACC_LINEAR`、`CHASSIS_PATH_DEVIATION_MM`。90° 弯的限速 = `sqrt(a·0.25·r)`，r 最大就是拐角容差，所以想更快只能加加速度或放宽容差。直线不受影响，靠刹车曲线减到拐角速度 |
| **起步/刹车手感** | 现场 `/set acc_linear`、`/set acc_angular`；默认值是 `chassis.h` 的 `CHASSIS_ACC_LINEAR`（前向和横向共用）、`CHASSIS_ACC_ANGULAR` |
| **转向角速度** | 现场 `/set turn_max_speed`；默认值是 `chassis.h` 的 `CHASSIS_TURN_MAX_SPEED`（90 deg/s） |
| **VEL 手工遥控**（调试用 `Chassis_SetVelocity`） | `chassis.h` 的 `CHASSIS_MAX_LINEAR` / `CHASSIS_MAX_ANGULAR` |

速度在固件里的合成顺序（改任何一个都要知道它被谁盖住）：

```
s_move_speed_limit（指令 100 / 巡检 150，Chassis_SetMoveSpeedLimit 设置，仅 VEL 模式生效）
  → 直线航点限速 s_path_limit = s_move_speed_limit
  → 拐角航点限速 s_path_limit = min(s_move_speed_limit, sqrt(a·0.25·r))
  → 出弯限速 exit_speed = min(s_move_speed_limit, sqrt(a·dev·c/(1-c)))
  → 刹车曲线 speed = min(s_move_speed_limit, sqrt(exit² + 2·a·剩余距离))
  → 逆解后整体限幅 CHASSIS_MAX_WHEEL_RPM
```

注意：`Chassis_SetMoveSpeedLimit()` 只在 `mode == VEL` 时接受新值，路径执行中改不动，限速表在提交路径时就算好了。`/set cmd_speed` 从下一条路径开始生效；加速度 `acc_linear` 下一拍就生效。

### 11.4 现场调速度的顺序

1. `AGV_STOP_EACH_SEGMENT=1` 先跑逐段停车，确认方向和位置都正常（速度放 100~150）。
2. 改回 `0`，看 `DONE path` 行的 `max_cross` 和 `max_heading`，再逐步加 `AGV_SURVEY_SPEED_MM_S`。
3. 过弯冲出去（偏线 >8 mm 触发 `AGV_ERR_POSE`）就降速度或降 `CHASSIS_ACC_LINEAR`，不要靠放宽 `AGV_CROSS_TRACK_MM` 掩盖。
4. 轮子打滑/丢步先查 `ZDT_RxDropped()` 和驱动器的堵转标志，再谈调参。

---

## 12. 修复记录（2026-11，底盘单测前）

| 问题 | 现象 | 修复 |
|---|---|---|
| Cortex-M7 的 DWT 没解锁（`DWT->LAR`） | 不接调试器时 `CYCCNT` 不计数，`Chassis_Micros()` 恒为 0：定距/转向的超时保护永远不触发，而接着调试器时一切正常 | `Chassis_DwtInit` 写 `LAR = 0xC5ACCE55` |
| FDCAN Bus-Off 后不恢复 | 总线一次严重错误（插拔、短路、电机上电瞬间）后硬件停在 INIT，所有电机命令都发不出，只能断电 | `ZDT_Service()` 每拍检测 Bus-Off，Stop/Start 自动恢复并计数 |
| 堵转保护无法清除 | 驱动器 `Cgp_TF` 置位后一直保持，`S` 永远回不到 READY，只能断电 | 新增 `ZDT_ClearProtection()`（`0E 52 6B`）；`S` 清故障时自动解除保护、重新使能掉线电机，恢复后自动回 READY |
| 故障/未就绪没有原因 | 只能看到 IDLE/FAULT，不知道是哪个电机、哪个传感器 | `Chassis_WaitReason()` 位图 + `chassis.fault/fault_detail`，串口直接写出 `RF:no-reply`、`LR:stall` 等 |
| 电机安装方向写死 | 左右镜像安装的电机无法在软件里修正，只能去驱动器菜单改 | `CHASSIS_WHEEL_DIR_xx`（命令与反馈同时乘方向），配合 `W` 单轮测试 |
| 没有陀螺仪就不能动 | 反馈检查强制要求 IMU，底盘单测时永远停在 IDLE | `CHASSIS_USE_IMU=0` 时航向由四轮里程计推算 |
| 输出只有机器帧 | `$T/$V/$H/...` 20 Hz 刷屏，串口助手里不可读，依赖上位机 | 改为可读文本：事件行、回执、1 Hz 状态行（`D` 可调）、`H` 帮助、`I` 诊断 |
| 拐角限速压住整段直线 | 不管设多快，通往拐角的直线都只跑拐角速度（默认约 50 mm/s）：1000 mm 直线接 90° 弯要 21.9 s | 只在拐角内部的小段全程限速，直线靠刹车曲线减速；同一段路 5.5 s，直线跑满 300 mm/s，拐角 50 mm/s |
| 巡检第一个拐角就报 `AGV_ERR_POSE` | 偏线统计按位置找最近段，路线重复经过同一条边时会跳到后面重合的段（回库段和第一段重合），真正的下一段被排除，车一离开重合段就累计到 8 mm。原来的 `agv_test` 用假底盘，这条闭环没人跑过 | 偏线只量控制器正在走的段（`s_path_vertex`）；新增 `survey_loop_test` 用真实底盘跑完整圈：约 99 s，12 条通道全覆盖，最大偏线 2.5 mm |
| 帮助文字被截断 | `Printf` 缓冲只有 255 字节，超长的帮助被切掉 | 整段文字走 `ChassisTelemetry_Puts` |

---

## 9. 比赛模式：激光雷达与视觉串口

只在 `AGV_MODE=2` 时编译/启用（`main.c`、`freertos.c` 里的调用都包在 `#if AGV_MODE == AGV_MODE_COMPETITION` 中，`ser_port.c` / `competition.c` 在其他模式下编译为空），指令模式和跑图模式行为不变。参数集中在 `User/Config/comp_config.h`。

### 9.1 接线

| 外设 | 引脚 | 参数 | 对端 |
|---|---|---|---|
| USART1 | PA9 = TX → 雷达 RX，PA10 = RX ← 雷达 TX | 460800 8N1 | RPLIDAR C1（5V 单独供电，共地，3.3V 电平） |
| USART2 | PA2 = TX → 上位机 RX，PA3 = RX ← 上位机 TX | 115200 8N1 | 视觉上位机 |
| USART3 | PB10 = TX → 模块 RX，PB11 = RX ← 模块 TX | 9600 8N1（`COMP_SCANNER_BAUD`） | 二维码扫码模块 |

这三路串口不进 `.ioc`：时钟、引脚、NVIC（优先级 6）都在 `SerPort_Init` 里配置，寄存器级中断收发 + 环形缓冲，不用 DMA（不涉及 D-Cache/MPU），CubeMX 重新生成不会覆盖。

### 9.2 雷达障碍物识别（`User/Navigation/lidar_obstacle.c`）

1. 上电 600 ms 后发 STOP，20 ms 后发 SCAN（标准扫描，约 10 Hz、0.72°）；1 s 无数据自动重启。
2. 每个点按**当前位姿**换算到场地坐标（底盘任务 5 ms 处理一次，运动补偿误差可忽略）。
3. 丢弃：距离 < 200 mm（车体自身，270 mm 车身的半对角线是 191 mm）或 > 1800 mm；离围挡 < 60 mm；离已知静态结构（四个岛、暂存/粗加工区、原料台）或已登记障碍 < 40 mm。
4. 沿扫描顺序聚类（相邻点 ≤ 45 mm），簇弦长 ≤ 95 mm 视为 Ø50 圆柱；弧上点重心沿视线外推 2r/π 得圆心。
5. 跨圈关联（60 mm），连续 4 圈命中才确认，确认后 `Map_AddObstacle(x, y, 35)` 写入栅格地图，地图版本号 +1，A* 自动避开。

雷达扫描平面必须低于 100 mm（障碍物高度）。安装位置/朝向现场用 `/set lidar_x`、`lidar_y`、`lidar_yaw`，定稿后写回 `COMP_LIDAR_X_MM / Y_MM / YAW_DEG`。
主机仿真测试（±15/±30 mm 测距噪声、200 圈）：圆心误差 ≤ 10 mm，岛角和围挡无误报。

### 9.3 视觉串口协议

见 `User/Comm/vision_link.h` 文件头（与 `vision/stm32_link.py` 一致）。后续取放料流程通过以下接口使用：

- `comp.x / comp.y / comp.heading_deg`：场地位姿；`comp.anchored` 置位后有效。
- `vision.material[] / ring_*`：上位机最新结果，`*_ms` 为收到时刻，用前检查新鲜度。
- `Comp_SetVisionMode(VISION_MATERIAL, COLOR_RED)` 等：切换上位机识别任务。
- `comp.task / comp.task_valid`：任务码，见 9.5。

台架调试不用写代码：控制台的 `/vision`、`/task`、`/obs`、`/pose` 和雷达参数见 4.4 节。

### 9.4 串口助手输出（USART6，1 Hz）

```
COMP pose=(2250,2250,0.0) lidar=RUN 10.1Hz pts=.. used=.. clu=.. obs=2 rs=0/0 | vision=ON mode=2 mat=3 ring=0 crc=0 | task=156+123+516+231 rej=0 scan=320/0
COMP obstacle #2 at (1203,1698)
COMP task 156+123+516+231 from scanner, batch 2 storage rings 2 1 3
COMP task rejected (format) from scanner: 123+321
```
`rs=a/b`：a = 雷达重启次数，b = 协议失步次数；`crc` 为视觉帧校验失败数；`rej` 为读到但不合法的次数；`scan=a/b`：a = 扫码串口收到的字节数，b = 串口硬件错误数（a 不涨查接线和模块读码模式，b 涨查波特率）。

### 9.5 任务码（`User/App/task_code.c`）

规则格式：四组三位数，以 `+` 连接。第 1、3 组是两批物料的颜色（按搬运顺序），第 2 组是第一批在粗加工区和暂存区的环号，第 4 组是第二批在粗加工区的环号。第二批在暂存区码垛到第一批同色物料上，环号不在码里，解析时推出来。

例：`156+123+516+231`

| 批次 | 搬运顺序 | 粗加工区环号 | 暂存区环号 |
|---|---|---|---|
| 1 | 红 黑 浅蓝 | 1 2 3 | 1 2 3 |
| 2 | 黑 红 浅蓝 | 2 3 1 | 2 1 3（码垛） |

颜色编号：1 红 2 黄 3 蓝 4 绿 5 黑 6 浅蓝，与 `VisionColor`、`vision/config.py` 一致。

- 合法条件：格式完整；每批三种不同颜色；第二批与第一批颜色相同；环号 1、2、3 各一次。不合法的内容打印原文后丢弃，已有的合法码不变。
- 读到新的合法码就更新 `comp.task`。取放料流程要在一键启动时清掉 `comp.task_valid`，并在开始搬运后用自己的副本，避免调试时扫过的码或中途误读影响本轮任务。
- 单片机保存任务并决定下一步；取料时用 `Comp_SetVisionMode(VISION_MATERIAL, comp.task.color[批次][顺序])` 告诉上位机要找的颜色，上位机不保存任务码。相机通道（`VISION_QR`）只是备用，上位机把读到的原文用 TASK 帧发回来，走同一套解析。

扫码模块设置（固件不发配置命令，用模块手册里的设置码配好）：

1. 连续或感应读码模式：固件不发触发命令。
2. 关掉补光灯和瞄准灯：规则只允许垂直向下补光。
3. 串口 8N1，波特率与 `COMP_SCANNER_BAUD` 一致；有 CR/LF 后缀最好，没有也能靠 `COMP_SCANNER_IDLE_MS` 分行。

