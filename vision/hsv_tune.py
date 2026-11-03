"""现场 HSV 阈值调节工具：拖滑条，看 mask，按 p 打印结果后填进 config.COLOR_RANGES。"""
import cv2
import numpy as np

import config as C

cap = cv2.VideoCapture(C.CAM_INDEX)
cv2.namedWindow("tune")
for n, v in [("Hmin", 0), ("Hmax", 179), ("Smin", 0), ("Smax", 255), ("Vmin", 0), ("Vmax", 255)]:
    cv2.createTrackbar(n, "tune", v, 255 if n[0] != "H" else 179, lambda _: None)
while True:
    ok, f = cap.read()
    if not ok:
        break
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    g = lambda n: cv2.getTrackbarPos(n, "tune")
    lo, hi = (g("Hmin"), g("Smin"), g("Vmin")), (g("Hmax"), g("Smax"), g("Vmax"))
    m = cv2.inRange(hsv, np.array(lo), np.array(hi))
    cv2.imshow("tune", np.hstack([f, cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)]))
    k = cv2.waitKey(1) & 0xFF
    if k == ord("p"):
        print(f"(({lo[0]}, {lo[1]}, {lo[2]}), ({hi[0]}, {hi[1]}, {hi[2]}))")
    elif k in (27, ord("q")):
        break
