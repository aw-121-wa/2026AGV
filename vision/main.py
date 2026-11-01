"""视觉 + 雷达主程序。

用法:
  python main.py                 # 正常运行，连单片机和雷达
  python main.py --show --no-mcu --no-lidar   # 只调摄像头
"""
import argparse
import time

import cv2

import color_detect
import config as C
import ring_detect


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--no-mcu", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    a = ap.parse_args()

    mcu = None
    if not a.no_mcu:
        from stm32_link import MCULink
        mcu = MCULink()
    lidar = None
    if not a.no_lidar:
        import lidar_obstacle
        lidar = lidar_obstacle.open_lidar()

    cap = cv2.VideoCapture(C.CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, C.CAM_FPS)

    last_lidar = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01); continue

        blobs = color_detect.detect(frame)
        ring = ring_detect.detect(frame)

        if mcu:
            if blobs:
                b = blobs[0]
                dx = (b.cx - C.TARGET_PX[0]) * C.MM_PER_PX
                dy = (b.cy - C.TARGET_PX[1]) * C.MM_PER_PX
                mcu.send_material(C.COLOR_ID[b.color], dx, dy)
            if ring:
                dx, dy, aligned = ring_detect.offset_to_target(ring)
                mcu.send_ring(aligned, dx, dy)

        if lidar and time.time() - last_lidar > 0.1:   # 10 Hz
            last_lidar = time.time()
            import lidar_obstacle
            obs = lidar_obstacle.find_obstacles(lidar.get_scan(), mcu.pose if mcu else None)
            if mcu and mcu.pose:
                mcu.send_obstacles(obs)

        if a.show:
            color_detect.draw(frame, blobs)
            ring_detect.draw(frame, ring)
            cv2.imshow("agv", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break


if __name__ == "__main__":
    main()
