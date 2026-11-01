"""与 STM32 的串口协议。

帧: 0xAA 0x55 | type(1) | len(1) | payload(len) | sum8(type+len+payload)
  type 0x01 物料:   color_id(u8) dx_mm(i16) dy_mm(i16)          —— 相对目标落点
  type 0x02 圆环:   aligned(u8)  dx_mm(i16) dy_mm(i16)
  type 0x03 障碍物: n(u8) + n*(x_mm(i16), y_mm(i16))           —— 地图坐标
  type 0x10 位姿(下行, 单片机→视觉): x_mm(i16) y_mm(i16) yaw_cdeg(i16)
小端。
"""
import struct
import threading

import serial

import config as C

T_MATERIAL, T_RING, T_OBSTACLE, T_POSE = 0x01, 0x02, 0x03, 0x10


def pack(t, payload: bytes) -> bytes:
    body = bytes([t, len(payload)]) + payload
    return b"\xAA\x55" + body + bytes([sum(body) & 0xFF])


class MCULink:
    def __init__(self, port=C.MCU_PORT, baud=C.MCU_BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.pose = None
        threading.Thread(target=self._rx, daemon=True).start()

    def send_material(self, color_id, dx, dy):
        self.ser.write(pack(T_MATERIAL, struct.pack("<Bhh", color_id, int(dx), int(dy))))

    def send_ring(self, aligned, dx, dy):
        self.ser.write(pack(T_RING, struct.pack("<Bhh", int(aligned), int(dx), int(dy))))

    def send_obstacles(self, obs):
        obs = obs[:30]
        p = bytes([len(obs)]) + b"".join(struct.pack("<hh", int(x), int(y)) for x, y in obs)
        self.ser.write(pack(T_OBSTACLE, p))

    def _rx(self):
        buf = bytearray()
        while True:
            buf += self.ser.read(64)
            while len(buf) >= 5:
                i = buf.find(b"\xAA\x55")
                if i < 0:
                    buf.clear(); break
                del buf[:i]
                if len(buf) < 4 or len(buf) < 5 + buf[3]:
                    break
                t, n = buf[2], buf[3]
                body, ck = bytes(buf[2:4 + n]), buf[4 + n]
                del buf[:5 + n]
                if sum(body) & 0xFF == ck and t == T_POSE and n == 6:
                    x, y, yaw = struct.unpack("<hhh", body[2:])
                    self.pose = (x, y, yaw / 100.0)
