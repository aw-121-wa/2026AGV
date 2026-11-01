# vision — 视觉识别 + 雷达避障（上位机，Python）

运行在车载上位机（树莓派 / Jetson / 香橙派等），通过串口与 `../firmware`（STM32H750，比赛模式）通信。
**激光雷达（RPLIDAR C1）直接接在 STM32 的 USART1 上，由单片机识别障碍物，上位机不处理雷达。**
**任务码由接在 STM32 USART3 上的扫码模块读取并由单片机解析，单片机用 MODE 帧逐个告诉上位机要找的颜色，上位机不需要知道任务码。** 模式 1（相机读二维码）只是备用通道，相机要能看到二维码板才有用。

## 任务背景
- 地图：2400 × 2400 mm 正方形；黑色圆柱障碍物 Ø50 × 100 mm 随机摆放。
- 物料：圆台，顶 Ø30 / 底 Ø50 / 高 60 mm；颜色 **红、黄、绿、浅蓝、蓝、黑**。
- 任务：把物料放到圆环靶标**正中间**。靶标为多圈同心实线/虚线圆，最外圈直径 = 物料直径 + 3 mm，所以对中误差必须控制在 ±1.5 mm 以内。

## 文件
| 文件 | 作用 |
|---|---|
| `config.py` | 所有参数：HSV 阈值、目标落点像素、mm/px、雷达串口与安装偏置、单片机串口 |
| `color_detect.py` | 6 色物料识别（HSV 分割 + 形态学 + 圆度过滤），输出颜色、中心、半径 |
| `ring_detect.py` | 同心圆环靶心检测（HoughCircles ALT + 暗线支持度打分 + 圆心聚类），虚线圆也能检出 |
| `stm32_link.py` | 与 STM32 的串口帧协议（见文件头注释） |
| `main.py` | 主循环：按单片机下发的 MODE 做二维码 / 物料 / 圆环识别并回传；10 Hz 心跳 |
| `hsv_tune.py` | 现场调 HSV 阈值的滑条工具，按 `p` 打印区间 |

## 使用
```bash
pip install -r requirements.txt
python hsv_tune.py                          # 1. 现场灯光下逐色标定，填入 config.COLOR_RANGES
python main.py --show --no-mcu --mode 2     # 2. 只看相机识别效果（1 二维码 / 2 物料 / 3 圆环）
python main.py                              # 3. 正式运行
```

## 标定要点
1. **TARGET_PX**：夹爪夹着物料、手动放到圆环正中，记下此时画面里的圆环中心像素，填入 `TARGET_PX`。之后视觉只需让"圆环中心 → TARGET_PX"偏差为 0。
2. **MM_PER_PX**：相机高度固定时，用 Ø50 物料的像素直径算：`50 / 像素直径`。
3. **浅蓝 vs 蓝**：两者靠色相 H 分界（默认 86–100 为浅蓝、101–130 为蓝）以及亮度 V；不同光照一定要重新标定。
4. **黑色**：黑色物料与黑色障碍物在相机里颜色相同，靠位置/尺寸区分：障碍物由雷达给出位置，物料识别只在物料区/暂存区使用。
5. `CAM_TO_BODY`：把物料放在夹爪正前方、正左方各一次，确认上报的 fwd/left 符号正确。

## 通信协议
接线：上位机 TX → STM32 PA3（USART2 RX），上位机 RX ← STM32 PA2（USART2 TX），115200 8N1，3.3V，共地。
权威定义见 `firmware/User/Comm/vision_link.h`，Python 实现见 `stm32_link.py`（两端已做互通测试）。

`AA 55 | LEN | TYPE | SEQ | PAYLOAD | CRC8`（小端，CRC-8 多项式 0x07 初值 0，覆盖 LEN..PAYLOAD）

| TYPE | 方向 | PAYLOAD |
|---|---|---|
| 0x01 HEARTBEAT | 上位机→MCU | state u8（0 正常/1 相机故障）, fps u8；≥5 Hz，500 ms 收不到判离线 |
| 0x02 TASK | 上位机→MCU | text[LEN]，ASCII（相机读到的二维码原文，由单片机解析校验） |
| 0x03 MATERIALS | 上位机→MCU | n u8, n × {color u8, fwd i16, left i16}，0.1 mm |
| 0x04 RING | 上位机→MCU | found u8, aligned u8, fwd i16, left i16，0.1 mm |
| 0x81 POSE | MCU→上位机 | x i16 mm, y i16 mm, yaw i16（0.01°）, valid u8；10 Hz |
| 0x82 MODE | MCU→上位机 | mode u8（0 关/1 二维码/2 物料/3 圆环）, color u8（0 任意）；5 Hz |
| 0x83 OBSTACLES | MCU→上位机 | n u8, n × {x i16, y i16} mm（雷达确认的障碍物） |

fwd/left 是目标相对“夹爪放置点”的偏差，车体坐标（前、左为正）；单片机让车平移 (fwd, left) 即对正。
颜色编号（比赛规则的任务码编号）：1 红 2 黄 3 蓝 4 绿 5 黑 6 浅蓝。
