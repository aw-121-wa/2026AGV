"""物料颜色识别：红/黄/绿/浅蓝/蓝/黑。

物料是俯视下的圆台（顶 Ø30 / 底 Ø50），俯视看到两个同心圆，
取外轮廓的最小外接圆作为物料中心。
"""
import cv2
import numpy as np

import config as C


class Blob:
    __slots__ = ("color", "cx", "cy", "radius", "area")

    def __init__(self, color, cx, cy, radius, area):
        self.color, self.cx, self.cy, self.radius, self.area = color, cx, cy, radius, area

    def __repr__(self):
        return f"Blob({self.color}, ({self.cx:.0f},{self.cy:.0f}), r={self.radius:.0f})"


_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def color_mask(hsv, name):
    mask = None
    for lo, hi in C.COLOR_RANGES[name]:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
    return mask


def detect(frame_bgr, colors=None, min_area=C.MIN_BLOB_AREA_PX):
    """返回所有物料 Blob，按面积从大到小。"""
    blur = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    out = []
    for name in colors or C.COLOR_RANGES:
        mask = color_mask(hsv, name)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            # 圆度过滤：物料俯视为圆，排除长条反光/地图标线
            if area / (np.pi * r * r + 1e-6) < 0.6:
                continue
            out.append(Blob(name, x, y, r, area))
    out.sort(key=lambda b: b.area, reverse=True)
    return out


DRAW_BGR = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 200, 0),
            "light_blue": (255, 200, 100), "blue": (255, 0, 0), "black": (60, 60, 60)}


def draw(frame, blobs):
    for b in blobs:
        col = DRAW_BGR[b.color]
        cv2.circle(frame, (int(b.cx), int(b.cy)), int(b.radius), col, 2)
        cv2.putText(frame, b.color, (int(b.cx - b.radius), int(b.cy - b.radius - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    return frame
