#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026AGV 地面站 · 桥接服务

浏览器不能直接开 TCP 或串口（Web Serial 只支持 USB），所以需要一个本机进程
把「无线 DAPLink 的 TCP 透传口」或「USB DAPLink 的虚拟串口」翻译成 WebSocket。

用法：
    # 无线 DAPLink：假定它把调试串口透传到 192.168.4.1:8080
    python bridge.py --tcp 192.168.4.1:8080

    # USB DAPLink 的 CDC 虚拟串口
    python bridge.py --serial COM5 --baud 115200

    # 换 WebSocket 端口，并开启原始数据回显（排查用）
    python bridge.py --tcp 192.168.4.1:8080 --port 9000 -v

    # 列出本机串口
    python bridge.py --list

TCP 模式零第三方依赖；串口模式需要 pyserial（pip install pyserial）。
"""

import argparse
import base64
import hashlib
import socket
import struct
import sys
import threading
import time

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

_clients = []
_clients_lock = threading.Lock()
_verbose = False


# --------------------------------------------------------------------------
# WebSocket 最小实现（服务端，只处理文本帧）
# --------------------------------------------------------------------------
def ws_handshake(conn):
    """完成 HTTP Upgrade。返回 True 表示握手成功。"""
    conn.settimeout(5.0)
    buf = b""
    while b"\r\n\r\n" not in buf:
        try:
            chunk = conn.recv(4096)
        except socket.timeout:
            return False
        if not chunk:
            return False
        buf += chunk
        if len(buf) > 16384:
            return False

    head = buf.split(b"\r\n\r\n", 1)[0].decode("latin1", "replace")
    key = None
    for line in head.split("\r\n")[1:]:
        if line.lower().startswith("sec-websocket-key:"):
            key = line.split(":", 1)[1].strip()
            break
    if not key:
        return False

    accept = base64.b64encode(
        hashlib.sha1((key + WS_GUID).encode("ascii")).digest()
    ).decode("ascii")

    conn.sendall((
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + accept + "\r\n"
        "\r\n"
    ).encode("ascii"))
    conn.settimeout(None)
    return True


def ws_send(conn, payload, opcode=0x1):
    """发送一帧。payload 为 bytes。opcode: 1=文本 2=二进制 8=关闭 10=pong"""
    if isinstance(payload, str):
        payload = payload.encode("utf-8")
    n = len(payload)
    if n < 126:
        header = struct.pack("!BB", 0x80 | opcode, n)
    elif n < 65536:
        header = struct.pack("!BBH", 0x80 | opcode, 126, n)
    else:
        header = struct.pack("!BBQ", 0x80 | opcode, 127, n)
    try:
        conn.sendall(header + payload)
    except OSError:
        pass


def ws_recv(conn):
    """接收一帧。返回 (opcode, bytes) 或 (None, None) 表示连接结束。"""
    def read_exact(n):
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    h = read_exact(2)
    if h is None:
        return None, None
    b0, b1 = h[0], h[1]
    opcode = b0 & 0x0F
    masked = (b1 & 0x80) != 0
    ln = b1 & 0x7F
    if ln == 126:
        ext = read_exact(2)
        if ext is None:
            return None, None
        ln = struct.unpack("!H", ext)[0]
    elif ln == 127:
        ext = read_exact(8)
        if ext is None:
            return None, None
        ln = struct.unpack("!Q", ext)[0]

    mask = read_exact(4) if masked else b"\x00\x00\x00\x00"
    if mask is None:
        return None, None

    data = read_exact(ln) if ln else b""
    if data is None:
        return None, None

    if masked:
        data = bytes(c ^ mask[i & 3] for i, c in enumerate(data))
    return opcode, data


# --------------------------------------------------------------------------
# 下行：链路 → 所有浏览器
# --------------------------------------------------------------------------
def broadcast(payload):
    if isinstance(payload, bytes):
        try:
            text = payload.decode("utf-8", "replace")
        except Exception:
            return
    else:
        text = payload

    with _clients_lock:
        dead = []
        for c in _clients:
            try:
                ws_send(c, text, 0x1)
            except Exception:
                dead.append(c)
        for c in dead:
            _clients.remove(c)


def log(msg):
    print(time.strftime("[%H:%M:%S] ") + msg, flush=True)


# --------------------------------------------------------------------------
# 链路后端：TCP 透传 / 串口
# --------------------------------------------------------------------------
class TcpLink:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.settimeout(None)
        self.name = "TCP %s:%d" % (host, port)

    def write(self, data):
        self.sock.sendall(data)

    def read_loop(self):
        while True:
            try:
                chunk = self.sock.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            if _verbose:
                log("RX %d B: %r" % (len(chunk), chunk[:48]))
            broadcast(chunk)
        log("链路断开")
        broadcast("\n[bridge] 链路断开\n")

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class SerialLink:
    def __init__(self, port, baud):
        try:
            import serial  # pyserial
        except ImportError:
            sys.exit("串口模式需要 pyserial：pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.name = "SERIAL %s @ %d" % (port, baud)

    def write(self, data):
        self.ser.write(data)

    def read_loop(self):
        while True:
            try:
                chunk = self.ser.read(4096)
            except Exception:
                break
            if chunk:
                if _verbose:
                    log("RX %d B: %r" % (len(chunk), chunk[:48]))
                broadcast(chunk)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# 每个浏览器连接的处理线程
# --------------------------------------------------------------------------
def client_thread(conn, addr, link, link_lock):
    try:
        if not ws_handshake(conn):
            conn.close()
            return
    except Exception:
        try:
            conn.close()
        except Exception:
            pass
        return

    with _clients_lock:
        _clients.append(conn)
    log("浏览器接入 %s:%d（当前 %d 个）" % (addr[0], addr[1], len(_clients)))
    ws_send(conn, "$L,sys,bridge 已连接 " + link.name + "\n", 0x1)

    try:
        while True:
            opcode, data = ws_recv(conn)
            if opcode is None:
                break
            if opcode == 0x8:          # close
                break
            if opcode == 0x9:          # ping
                ws_send(conn, data, 0xA)
                continue
            if opcode == 0xA:          # pong
                continue
            if opcode in (0x0, 0x1, 0x2):
                if _verbose:
                    log("TX %d B: %r" % (len(data), data[:64]))
                with link_lock:
                    try:
                        link.write(data)
                    except Exception as e:
                        log("写入链路失败: %s" % e)
    except (OSError, ConnectionResetError):
        pass
    finally:
        with _clients_lock:
            if conn in _clients:
                _clients.remove(conn)
        try:
            ws_send(conn, b"", 0x8)
            conn.close()
        except Exception:
            pass
        log("浏览器断开（剩余 %d 个）" % len(_clients))


def list_serial_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        print("需要 pyserial 才能列出串口：pip install pyserial")
        return
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口")
        return
    for p in ports:
        print("%-8s %s" % (p.device, p.description))


def main():
    global _verbose
    ap = argparse.ArgumentParser(description="2026AGV 地面站桥接：DAPLink/TCP → WebSocket")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--tcp", metavar="HOST:PORT", help="无线 DAPLink 的 TCP 透传地址")
    src.add_argument("--serial", metavar="PORT", help="串口名，如 COM5")
    src.add_argument("--list", action="store_true", help="列出本机串口")
    ap.add_argument("--baud", type=int, default=115200, help="串口波特率，默认 115200")
    ap.add_argument("--port", type=int, default=8766, help="WebSocket 监听端口，默认 8766")
    ap.add_argument("--host", default="127.0.0.1", help="WebSocket 监听地址，默认只监听本机")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印收发的原始字节")
    args = ap.parse_args()

    _verbose = args.verbose

    if args.list:
        list_serial_ports()
        return

    if not args.tcp and not args.serial:
        ap.print_help()
        print("\n必须指定 --tcp 或 --serial")
        return

    # 先占住 WebSocket 端口：端口冲突时立刻退出，不用先去占用串口
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind((args.host, args.port))
    except OSError as e:
        srv.close()
        sys.exit("WebSocket 端口 %d 绑定失败：%s\n"
                 "多半是端口被别的程序占用（8765 常被桌面客户端占）。\n"
                 "换个端口重试，例如加 --port 8766，网页地址也改成同一个端口。"
                 % (args.port, e.strerror))
    srv.listen(8)

    # 建链路
    try:
        if args.tcp:
            host, _, port = args.tcp.partition(":")
            if not port:
                srv.close()
                sys.exit("--tcp 格式应为 HOST:PORT，例如 192.168.4.1:8080")
            link = TcpLink(host, int(port))
        else:
            link = SerialLink(args.serial, args.baud)
    except Exception as e:
        srv.close()
        sys.exit("连接链路失败：%s" % e)

    log("链路就绪：" + link.name)

    link_lock = threading.Lock()
    threading.Thread(target=link.read_loop, daemon=True).start()

    log("WebSocket 监听 ws://%s:%d" % (args.host, args.port))
    log("在网页里填入上面的地址，点「连接」。Ctrl+C 退出。")

    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=client_thread,
                             args=(conn, addr, link, link_lock), daemon=True).start()
    except KeyboardInterrupt:
        log("退出")
    finally:
        link.close()
        srv.close()


if __name__ == "__main__":
    main()
