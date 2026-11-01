"""全局参数。现场标定后只改这里。"""

# ---------------- 场地 ----------------
MAP_SIZE_MM = 2400            # 正方形地图边长
OBSTACLE_DIAMETER_MM = 50     # 黑色障碍物：直径 50，高 100
OBSTACLE_HEIGHT_MM = 100

# ---------------- 物料 ----------------
# 圆台：顶 Ø30，底 Ø50，高 60（上下各 8mm 圆柱段）
MATERIAL_TOP_D_MM = 30
MATERIAL_BOTTOM_D_MM = 50
MATERIAL_HEIGHT_MM = 60

# ---------------- 摄像头 ----------------
CAM_INDEX = 0
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 60
# 夹爪正下方/物料应落点在图像中的像素坐标（标定：夹着物料放到圆环正中，记下此时圆环中心像素）
TARGET_PX = (320, 240)
# 圆环中心与 TARGET_PX 偏差小于此值即判定"已对准"
ALIGN_TOL_PX = 4
# 像素 -> mm 比例（俯视、固定高度时近似为常数；用已知 Ø50 物料标定）
MM_PER_PX = 0.5

# ---------------- 颜色阈值 (OpenCV HSV: H 0-179, S/V 0-255) ----------------
# 每种颜色可以给多段区间（红色跨 0/180）。现场用 hsv_tune.py 重新标定。
COLOR_RANGES = {
    "red":        [((0, 100, 70), (8, 255, 255)), ((165, 100, 70), (179, 255, 255))],
    "yellow":     [((20, 100, 100), (34, 255, 255))],
    "green":      [((40, 70, 50), (85, 255, 255))],
    "light_blue": [((86, 60, 120), (100, 255, 255))],   # 浅蓝/青：色相偏低、亮度高
    "blue":       [((101, 120, 40), (130, 255, 255))],  # 深蓝：色相偏高
    "black":      [((0, 0, 0), (179, 255, 55))],
}
# 颜色编号（与单片机协议一致）
COLOR_ID = {"red": 1, "yellow": 2, "green": 3, "light_blue": 4, "blue": 5, "black": 6}

MIN_BLOB_AREA_PX = 400

# ---------------- 雷达 ----------------
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 230400           # LD19 / LD06 / STL-19P
LIDAR_TYPE = "ld19"           # "ld19" 或 "rplidar"
LIDAR_MIN_RANGE_MM = 80
LIDAR_MAX_RANGE_MM = 3500
# 雷达在车体坐标系中的安装位置 (mm) 和朝向偏置 (deg)
LIDAR_OFFSET_X_MM = 0.0
LIDAR_OFFSET_Y_MM = 0.0
LIDAR_YAW_OFFSET_DEG = 0.0
CLUSTER_GAP_MM = 30           # 相邻点距离大于此值则断开聚类
OBSTACLE_D_TOL_MM = 25        # 聚类直径落在 50±25 视为障碍物
WALL_MARGIN_MM = 40           # 离边界太近的点视为围挡，丢弃

# ---------------- 与 STM32 通信 ----------------
MCU_PORT = "/dev/ttyAMA0"
MCU_BAUD = 115200
