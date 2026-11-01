"""视觉主程序（比赛模式）。

单片机通过 MODE 帧告诉上位机当前要做什么：
  1 二维码  -> 相机读码，发 TASK 原文（备用通道：任务码平时由单片机上的扫码模块读取）
  2 物料    -> 识别物料颜色和位置，发 MATERIALS（可按单片机指定颜色过滤）
  3 圆环    -> 识别靶心，发 RING（偏差 + 是否已对准）
无论什么模式都以 HEARTBEAT_HZ 发心跳。激光雷达由单片机处理，这里不涉及。

用法:
  python main.py                       # 正常运行
  python main.py --show --no-mcu --mode 2   # 只调相机：本地指定模式，不连单片机
"""
import argparse
import time

import cv2

import color_detect
import config as C
import ring_detect
from stm32_link import MCULink, MODE_MATERIAL, MODE_OFF, MODE_QR, MODE_RING

ID_TO_COLOR = {v: k for k, v in C.COLOR_ID.items()}


def to_body(px, py):
    """像素坐标 -> 相对夹爪放置点的车体偏差 (fwd_mm, left_mm)。"""
    dx = (px - C.TARGET_PX[0]) * C.MM_PER_PX
    dy = (py - C.TARGET_PX[1]) * C.MM_PER_PX
    m = C.CAM_TO_BODY
    return m[0][0] * dx + m[0][1] * dy, m[1][0] * dx + m[1][1] * dy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--no-mcu", action="store_true")
    ap.add_argument("--mode", type=int, default=None, help="不连单片机时本地指定模式 1/2/3")
    a = ap.parse_args()

    mcu = None if a.no_mcu else MCULink()
    cap = cv2.VideoCapture(C.CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, C.CAM_FPS)
    qr = cv2.QRCodeDetector()

    last_hb, frames, fps, fps_t = 0.0, 0, 0.0, time.time()
    while True:
        ok, frame = cap.read()
        now = time.time()
        if mcu and now - last_hb >= 1.0 / C.HEARTBEAT_HZ:
            last_hb = now
            mcu.send_heartbeat(ok, fps)
        if not ok:
            time.sleep(0.01)
            continue
        frames += 1
        if now - fps_t >= 1.0:
            fps, frames, fps_t = frames / (now - fps_t), 0, now

        mode = a.mode if a.mode is not None else (mcu.mode if mcu else MODE_OFF)
        want = mcu.mode_color if mcu else 0
        blobs, ring = [], None

        if mode == MODE_QR:
            text, _, _ = qr.detectAndDecode(frame)
            if text and mcu:
                mcu.send_task(text)
            if text and a.show:
                cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        elif mode == MODE_MATERIAL:
            colors = [ID_TO_COLOR[want]] if want in ID_TO_COLOR else None
            blobs = color_detect.detect(frame, colors)[:6]
            if mcu:
                mcu.send_materials([(C.COLOR_ID[b.color], *to_body(b.cx, b.cy)) for b in blobs])
        elif mode == MODE_RING:
            ring = ring_detect.detect(frame)
            if mcu:
                if ring:
                    fwd, left = to_body(ring[0], ring[1])
                    _, _, aligned = ring_detect.offset_to_target(ring)
                    mcu.send_ring(True, aligned, fwd, left)
                else:
                    mcu.send_ring(False)

        if a.show:
            color_detect.draw(frame, blobs)
            ring_detect.draw(frame, ring)
            info = f"mode={mode} fps={fps:.0f}"
            if mcu and mcu.pose:
                info += " pose=(%d,%d,%.1f)" % mcu.pose
            cv2.putText(frame, info, (10, C.CAM_HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1)
            cv2.imshow("agv", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break


if __name__ == "__main__":
    main()
