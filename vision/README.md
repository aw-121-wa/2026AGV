# vision — 视觉识别 + 雷达避障（上位机，Python）

运行在车载上位机（树莓派 / Jetson / 香橙派等），通过串口把结果发给 `../firmware`（STM32H750）。

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
| `lidar_obstacle.py` | LD19/LD06（默认）或 RPLIDAR 驱动；点云聚类，按直径≈50 mm 识别障碍物，输出地图坐标 |
| `stm32_link.py` | 与 STM32 的串口帧协议（见文件头注释） |
| `main.py` | 主循环：相机 → 物料/靶心偏差；雷达 10 Hz → 障碍物列表；发送给单片机 |
| `hsv_tune.py` | 现场调 HSV 阈值的滑条工具，按 `p` 打印区间 |

## 使用
```bash
pip install -r requirements.txt
python hsv_tune.py                          # 1. 现场灯光下逐色标定，填入 config.COLOR_RANGES
python main.py --show --no-mcu --no-lidar   # 2. 只看相机识别效果
python main.py                              # 3. 正式运行
```

## 标定要点
1. **TARGET_PX**：夹爪夹着物料、手动放到圆环正中，记下此时画面里的圆环中心像素，填入 `TARGET_PX`。之后视觉只需让"圆环中心 → TARGET_PX"偏差为 0。
2. **MM_PER_PX**：相机高度固定时，用 Ø50 物料的像素直径算：`50 / 像素直径`。
3. **浅蓝 vs 蓝**：两者靠色相 H 分界（默认 86–100 为浅蓝、101–130 为蓝）以及亮度 V；不同光照一定要重新标定。
4. **黑色**：黑色物料与黑色障碍物在相机里颜色相同，靠位置/尺寸区分：障碍物由雷达给出位置，物料识别只在物料区/暂存区使用。
5. 雷达安装偏置 `LIDAR_OFFSET_*`、`LIDAR_YAW_OFFSET_DEG` 按实车测量。

## 通信协议（小端）
`AA 55 | type | len | payload | sum8(type+len+payload)`

| type | 方向 | payload |
|---|---|---|
| 0x01 物料 | 视觉→MCU | color_id u8（1红 2黄 3绿 4浅蓝 5蓝 6黑）, dx_mm i16, dy_mm i16 |
| 0x02 圆环 | 视觉→MCU | aligned u8, dx_mm i16, dy_mm i16 |
| 0x03 障碍物 | 视觉→MCU | n u8, n × (x_mm i16, y_mm i16)，地图坐标 |
| 0x10 位姿 | MCU→视觉 | x_mm i16, y_mm i16, yaw(0.01°) i16 |

单片机侧需要新增对应的帧解析（建议放在 `firmware/User/Comm/`），收到障碍物后在栅格地图中把圆心周围 (25 mm + 车体半径) 标为占用，再触发 A* 重规划。
