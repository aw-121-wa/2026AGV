"""与 STM32 的串口协议（比赛模式，对应 firmware/User/Comm/vision_link.h）。

帧：AA 55 | LEN | TYPE | SEQ | PAYLOAD[LEN] | CRC8      小端
    CRC-8 多项式 0x07、初值 0，覆盖 LEN TYPE SEQ PAYLOAD

上位机 -> STM32
  0x01 HEARTBEAT  state u8, fps u8
  0x02 TASK       text[LEN] ASCII                            相机读到的二维码原文，单片机解析校验
  0x03 MATERIALS  n u8, n*{color u8, fwd i16, left i16}      0.1 mm，车体系（前/左为正）
  0x04 RING       found u8, aligned u8, fwd i16, left i16    0.1 mm
STM32 -> 上位机
  0x81 POSE       x i16 mm, y i16 mm, yaw i16 0.01deg, valid u8
  0x82 MODE       mode u8 (0 关/1 二维码/2 物料/3 圆环), color u8 (0 = 任意)
  0x83 OBSTACLES  n u8, n*{x i16, y i16} mm
"""
import struct
import threading
import time

import serial

import config as C

HEARTBEAT, TASK, MATERIALS, RING = 0x01, 0x02, 0x03, 0x04
POSE, MODE, OBSTACLES = 0x81, 0x82, 0x83
MODE_OFF, MODE_QR, MODE_MATERIAL, MODE_RING = 0, 1, 2, 3
MAX_PAYLOAD = 64


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _clamp16(v):
    return max(-32768, min(32767, int(round(v))))


class Parser:
    """逐字节状态机，与单片机端解析逻辑一致。"""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        frames = []
        for b in data:
            n = len(self.buf)
            if n == 0 and b != 0xAA:
                continue
            if n == 1 and b != 0x55:
                self.buf = bytearray([0xAA]) if b == 0xAA else bytearray()
                continue
            if n == 2 and b > MAX_PAYLOAD:
                self.buf = bytearray()
                continue
            self.buf.append(b)
            if len(self.buf) >= 6 and len(self.buf) == 6 + self.buf[2]:
                f, self.buf = bytes(self.buf), bytearray()
                ln = f[2]
                if crc8(f[2:5 + ln]) != f[5 + ln]:
                    self.crc_errors += 1
                    continue
                frames.append((f[3], f[5:5 + ln]))
        return frames


class MCULink:
    def __init__(self, port=C.MCU_PORT, baud=C.MCU_BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.seq = 0
        self.lock = threading.Lock()
        self.parser = Parser()
        self.pose = None            # (x_mm, y_mm, yaw_deg)，valid 才更新
        self.mode = MODE_OFF        # 单片机要求的视觉任务
        self.mode_color = 0
        self.obstacles = []
        self.last_rx = 0.0
        threading.Thread(target=self._rx, daemon=True).start()

    # ---------- 发送 ----------
    def _send(self, t, payload=b""):
        with self.lock:
            body = bytes([len(payload), t, self.seq]) + payload
            self.seq = (self.seq + 1) & 0xFF
            self.ser.write(b"\xAA\x55" + body + bytes([crc8(body)]))

    def send_heartbeat(self, ok=True, fps=0):
        self._send(HEARTBEAT, bytes([0 if ok else 1, max(0, min(255, int(fps)))]))

    def send_task(self, text):
        """text: 相机读到的二维码原文。任务码的解析和校验都在单片机（task_code.c）。"""
        self._send(TASK, text.encode("ascii", "replace")[:MAX_PAYLOAD])

    def send_materials(self, items):
        """items: [(color_id, fwd_mm, left_mm)]，最多 6 个。"""
        items = items[:6]
        p = bytes([len(items)]) + b"".join(
            struct.pack("<Bhh", c, _clamp16(f * 10), _clamp16(l * 10)) for c, f, l in items)
        self._send(MATERIALS, p)

    def send_ring(self, found, aligned=False, fwd_mm=0.0, left_mm=0.0):
        self._send(RING, struct.pack("<BBhh", int(found), int(aligned),
                                     _clamp16(fwd_mm * 10), _clamp16(left_mm * 10)))

    # ---------- 接收 ----------
    def online(self, timeout=1.0):
        return time.time() - self.last_rx < timeout

    def _rx(self):
        while True:
            data = self.ser.read(64)
            if not data:
                continue
            for t, p in self.parser.feed(data):
                self.last_rx = time.time()
                if t == POSE and len(p) == 7:
                    x, y, yaw, valid = struct.unpack("<hhhB", p)
                    if valid:
                        self.pose = (x, y, yaw / 100.0)
                elif t == MODE and len(p) == 2:
                    self.mode, self.mode_color = p[0], p[1]
                elif t == OBSTACLES and len(p) >= 1 and len(p) == 1 + 4 * p[0]:
                    self.obstacles = [struct.unpack_from("<hh", p, 1 + 4 * i) for i in range(p[0])]
