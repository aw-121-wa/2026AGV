"""激光雷达障碍物检测。

场地 2400x2400，黑色圆柱障碍 Ø50 x 100 随机摆放。
流程：读一圈点云 → 车体系 → (有位姿时) 地图系 → 去掉围挡附近的点
     → 按相邻点间距聚类 → 拟合圆，直径约 50mm 的保留为障碍物。
默认驱动 LD19/LD06（230400，0x54 帧头），也可选 rplidar。
"""
import math
import struct
import threading

import serial

import config as C


# ---------------- 驱动：LD19 / LD06 ----------------
class LD19:
    PKT_LEN = 47

    def __init__(self, port=C.LIDAR_PORT, baud=C.LIDAR_BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self._scan, self._cur, self._last_angle = [], [], None
        self._lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        buf = bytearray()
        while True:
            buf += self.ser.read(256)
            while len(buf) >= self.PKT_LEN:
                i = buf.find(b"\x54\x2c")
                if i < 0:
                    buf.clear(); break
                if len(buf) - i < self.PKT_LEN:
                    del buf[:i]; break
                self._parse(bytes(buf[i:i + self.PKT_LEN]))
                del buf[:i + self.PKT_LEN]

    def _parse(self, p):
        start = struct.unpack_from("<H", p, 4)[0] / 100.0
        end = struct.unpack_from("<H", p, 42)[0] / 100.0
        span = (end - start) % 360.0
        for k in range(12):
            dist, conf = struct.unpack_from("<HB", p, 6 + 3 * k)
            ang = (start + span * k / 11.0) % 360.0
            if self._last_angle is not None and ang < self._last_angle - 180:
                with self._lock:
                    self._scan = self._cur
                self._cur = []
            self._last_angle = ang
            if conf > 100 and C.LIDAR_MIN_RANGE_MM < dist < C.LIDAR_MAX_RANGE_MM:
                self._cur.append((ang, float(dist)))

    def get_scan(self):
        """[(角度deg, 距离mm)]，LD19 为顺时针角度。"""
        with self._lock:
            return list(self._scan)


def open_lidar():
    if C.LIDAR_TYPE == "rplidar":
        from rplidar import RPLidar  # pip install rplidar-roboticia

        class _RP:
            def __init__(self):
                self.l = RPLidar(C.LIDAR_PORT)
                self.it = self.l.iter_scans()

            def get_scan(self):
                return [(a, d) for _, a, d in next(self.it)]
        return _RP()
    return LD19()


# ---------------- 处理 ----------------
def scan_to_points(scan, pose=None):
    """雷达(角度,距离) → 点坐标。pose=(x_mm, y_mm, yaw_deg) 时输出地图系，否则车体系。
    车体系：x 向前，y 向左，逆时针为正（雷达顺时针角度在这里取负）。"""
    pts = []
    for ang, d in scan:
        a = math.radians(-ang + C.LIDAR_YAW_OFFSET_DEG)
        x = C.LIDAR_OFFSET_X_MM + d * math.cos(a)
        y = C.LIDAR_OFFSET_Y_MM + d * math.sin(a)
        if pose:
            px, py, yaw = pose
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            x, y = px + c * x - s * y, py + s * x + c * y
        pts.append((x, y))
    return pts


def _cluster(pts):
    clusters, cur = [], []
    for p in pts:
        if cur and math.dist(p, cur[-1]) > C.CLUSTER_GAP_MM:
            clusters.append(cur); cur = []
        cur.append(p)
    if cur:
        if clusters and math.dist(cur[-1], clusters[0][0]) <= C.CLUSTER_GAP_MM:
            clusters[0] = cur + clusters[0]   # 首尾跨 0° 合并
        else:
            clusters.append(cur)
    return clusters


def _fit_circle(pts):
    """雷达只能看到圆柱的前半弧：用弦宽估计直径，圆心沿视线外推半径。"""
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    width = math.dist(pts[0], pts[-1])
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    return mx, my, width


def find_obstacles(scan, pose=None):
    """返回 [(x_mm, y_mm)] 障碍物圆心。有 pose 时为地图坐标，并剔除围挡。"""
    pts = scan_to_points(sorted(scan), pose)
    if pose:
        m, L = C.WALL_MARGIN_MM, C.MAP_SIZE_MM
        pts = [p for p in pts if m < p[0] < L - m and m < p[1] < L - m]
    obs = []
    for cl in _cluster(pts):
        if len(cl) < 3:
            continue
        mx, my, w = _fit_circle(cl)
        if abs(w - C.OBSTACLE_DIAMETER_MM) > C.OBSTACLE_D_TOL_MM and w > 10:
            continue
        r = C.OBSTACLE_DIAMETER_MM / 2
        # 沿雷达→弧中点方向外推半径，得到圆心
        ox = pose[0] if pose else C.LIDAR_OFFSET_X_MM
        oy = pose[1] if pose else C.LIDAR_OFFSET_Y_MM
        d = math.hypot(mx - ox, my - oy) or 1
        obs.append((mx + (mx - ox) / d * r, my + (my - oy) / d * r))
    return obs


def to_grid(obs, cell_mm=50):
    """障碍物 → 栅格索引，便于单片机 A* 直接标记。"""
    return [(int(x // cell_mm), int(y // cell_mm)) for x, y in obs]
