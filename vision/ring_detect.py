"""圆环（放置靶标）检测。

靶标是若干同心的实线/虚线圆环，最内圈（1 环）外径 = 物料最大径 + 3mm。
虚线圆用轮廓法会断开，所以这里用"灰度暗线 → 边缘 → HoughCircles"，
再把圆心相近的所有圆聚成一组，组内圆越多越可信，取组内圆心中位数作为靶心。
"""
import math

import cv2
import numpy as np

import config as C


def detect(frame_bgr, min_r=15, max_r=200):
    """返回 (cx, cy, r_outer, n_circles) 或 None。"""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.5)
    # HOUGH_GRADIENT_ALT 对同心圆更准、误检少；虚线圆也能投票成功
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT_ALT, dp=1.5, minDist=2,
                               param1=150, param2=0.6, minRadius=min_r, maxRadius=max_r)
    if circles is None:
        return None
    circles = circles[0]

    # 暗线二值图，用于给每个候选圆打分（圆周上落在暗线上的比例）
    dark = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY_INV, 21, 10)
    dark = cv2.dilate(dark, np.ones((3, 3), np.uint8))
    h, w = dark.shape
    th = np.linspace(0, 2 * np.pi, 90, endpoint=False)

    def support(c):
        xs = np.clip((c[0] + c[2] * np.cos(th)).astype(int), 0, w - 1)
        ys = np.clip((c[1] + c[2] * np.sin(th)).astype(int), 0, h - 1)
        return float((dark[ys, xs] > 0).mean())

    good = [c for c in circles if support(c) > 0.45]   # 虚线圆约 0.5~0.7，实线约 1
    if len(good) < 2:
        return None
    good = np.array(good)

    # 按圆心聚类（同心圆圆心距离 < 6px），取总支持度最高的一组
    best, best_score = None, 0.0
    for i in range(len(good)):
        d = np.hypot(good[:, 0] - good[i, 0], good[:, 1] - good[i, 1])
        grp = good[d < 6]
        if len(grp) < 2:
            continue
        score = sum(support(c) for c in grp)
        if score > best_score:
            best, best_score = grp, score
    if best is None:   # 没有同心结构，不是靶标
        return None
    cx, cy = float(np.median(best[:, 0])), float(np.median(best[:, 1]))
    return cx, cy, float(best[:, 2].max()), len(best)


def offset_to_target(ring):
    """圆环中心相对目标落点的偏移（mm，图像 x 向右 / y 向下）以及是否已对准。"""
    cx, cy, _, _ = ring
    dx, dy = (cx - C.TARGET_PX[0]) * C.MM_PER_PX, (cy - C.TARGET_PX[1]) * C.MM_PER_PX
    return dx, dy, math.hypot(dx, dy) <= C.ALIGN_TOL_MM


def draw(frame, ring):
    if ring:
        cx, cy, r, n = ring
        cv2.circle(frame, (int(cx), int(cy)), int(r), (255, 0, 255), 2)
        cv2.drawMarker(frame, (int(cx), int(cy)), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, f"ring n={n}", (int(cx) + 6, int(cy) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    cv2.drawMarker(frame, C.TARGET_PX, (0, 255, 0), cv2.MARKER_TILTED_CROSS, 16, 2)
    return frame
