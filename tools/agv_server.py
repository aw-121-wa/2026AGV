#!/usr/bin/env python3
"""2026AGV 调试台的本地助手：打开调试台，并替网页编译、烧录固件。

    python tools/agv_server.py                 # 在 http://localhost:8765/ 打开调试台
    python tools/agv_server.py --port 9000 --no-browser

网页（agv_console.html）不能自己运行本机的程序，“固件”页的编译和烧录由这个助手来做。它只做固定的
几件事，不接受任意命令：
  - 按模式编译：cmake + ninja + arm-none-eabi-gcc，Release，-DAGV_MODE=0..3。每个模式一个目录
    firmware/build/console-<模式>，来回切模式只编改过的文件，也不动你手动用的 build/Release。
  - 烧录：OpenOCD + CMSIS-DAP 或 ST-Link（target/stm32h7x.cfg），烧完校验、复位运行。
只听本机 127.0.0.1；接口要带 X-AGV-Console 头，别的网站的页面调不动它。
串口（USART6）还是网页自己用 Web Serial 连，和烧录（SWD）互不影响：烧完单片机复位，网页认出新模式。
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent.parent
TOOLS = ROOT / 'tools'
FIRMWARE = ROOT / 'firmware'
TOOLCHAIN = FIRMWARE / 'cmake' / 'gcc-arm-none-eabi.cmake'

# 和 firmware/User/Config/agv_config.h 的 AGV_MODE 一致
MODES = {0: 'command', 1: 'survey', 2: 'competition', 3: 'arm'}
PROBES = {'cmsis-dap': 'interface/cmsis-dap.cfg', 'stlink': 'interface/stlink.cfg'}
ACTIONS = ('build', 'flash', 'build-flash')
SWD_KHZ = 1000  # 和 .vscode/launch.json 的 CMSIS-DAP 调试配置一样


def build_dir(mode):
    return FIRMWARE / 'build' / f'console-{MODES[mode]}'


def elf_path(mode):
    return build_dir(mode) / '2026AGV.elf'


def find_tools():
    return {name: shutil.which(name) for name in ('cmake', 'ninja', 'arm-none-eabi-gcc', 'openocd')}


def decode(raw):
    for encoding in ('utf-8', 'gbk'):
        try:
            return raw.decode(encoding)
        except UnicodeDecodeError:
            pass
    return raw.decode('utf-8', 'replace')


class Job:
    """一次编译或烧录：一步一步跑外部程序，输出一行一行攒着给网页取。"""

    def __init__(self, ident, action, mode, probe):
        self.id, self.action, self.mode, self.probe = ident, action, mode, probe
        self.lines, self.done, self.ok, self.proc, self.cancelled = [], False, None, None, False
        self.started = time.time()
        self.lock = threading.Lock()

    def say(self, text):
        with self.lock:
            self.lines.append(text)

    def run_step(self, title, command):
        self.say(f'$ {title}')
        flags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == 'nt' else 0
        self.proc = subprocess.Popen(command, cwd=FIRMWARE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     creationflags=flags)
        for raw in iter(self.proc.stdout.readline, b''):
            self.say(decode(raw).rstrip('\r\n'))
        code = self.proc.wait()
        self.proc = None
        if self.cancelled:
            self.say('已停止。')
            return False
        if code:
            self.say(f'失败（返回 {code}）。')
        return code == 0

    def cancel(self):
        self.cancelled = True
        proc = self.proc
        if not proc:
            return
        if os.name == 'nt':  # ninja 底下还有编译器进程，整棵树一起结束
            subprocess.run(['taskkill', '/F', '/T', '/PID', str(proc.pid)], capture_output=True)
        else:
            proc.kill()

    def run(self):
        tools = find_tools()
        ok = True
        try:
            if self.action in ('build', 'build-flash'):
                ok = self.build(tools)
            if ok and self.action in ('flash', 'build-flash'):
                ok = self.flash(tools)
        except OSError as e:
            self.say(f'运行失败：{e}')
            ok = False
        with self.lock:
            self.ok, self.done = ok and not self.cancelled, True

    def build(self, tools):
        missing = [n for n in ('cmake', 'ninja', 'arm-none-eabi-gcc') if not tools[n]]
        if missing:
            self.say('找不到 ' + '、'.join(missing) + '：装好并加进 PATH（README 8 节）。')
            return False
        out = build_dir(self.mode)
        configure = [tools['cmake'], '-S', str(FIRMWARE), '-B', str(out), '-G', 'Ninja',
                     f'-DCMAKE_TOOLCHAIN_FILE={TOOLCHAIN.as_posix()}', '-DCMAKE_BUILD_TYPE=Release',
                     f'-DCMAKE_MAKE_PROGRAM={Path(tools["ninja"]).as_posix()}', f'-DAGV_MODE={self.mode}']
        return (self.run_step(f'配置 AGV_MODE={self.mode}（{MODES[self.mode]}）', configure) and
                self.run_step('编译', [tools['cmake'], '--build', str(out)]))

    def flash(self, tools):
        if not tools['openocd']:
            self.say('找不到 openocd：装好并加进 PATH（scoop install openocd）。')
            return False
        elf = elf_path(self.mode)
        if not elf.exists():
            self.say(f'还没编译过这个模式（{elf}），先点“编译”。')
            return False
        command = [tools['openocd'], '-f', PROBES[self.probe], '-f', 'target/stm32h7x.cfg',
                   '-c', f'adapter speed {SWD_KHZ}', '-c', f'program {{{elf.as_posix()}}} verify reset exit']
        return self.run_step(f'烧录（{self.probe}）', command)

    def view(self, start=0):
        with self.lock:
            return {'id': self.id, 'action': self.action, 'mode': self.mode, 'lines': self.lines[start:],
                    'next': len(self.lines), 'done': self.done, 'ok': self.ok}


class State:
    job = None
    next_id = 1
    lock = threading.Lock()


def builds():
    out = []
    for mode, name in MODES.items():
        elf = elf_path(mode)
        info = {'mode': mode, 'name': name, 'built': None}
        if elf.exists():
            info['built'] = int(elf.stat().st_mtime * 1000)
        out.append(info)
    return out


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(TOOLS), **kwargs)

    def log_message(self, fmt, *args):  # 网页每 300 ms 问一次进度，不刷屏
        if not self.path.startswith('/api/'):
            super().log_message(fmt, *args)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')  # 改了网页刷新就生效
        super().end_headers()

    def allowed(self):
        """只认本机地址（防 DNS 重绑定）和自己网页带的头（别的网站跨域发不了自定义头）。"""
        host = (self.headers.get('Host') or '').split(':')[0]
        return host in ('localhost', '127.0.0.1') and self.headers.get('X-AGV-Console') == '1'

    def reply(self, status, body):
        data = json.dumps(body, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == '/':
            self.send_response(HTTPStatus.FOUND)
            self.send_header('Location', '/agv_console.html')
            self.end_headers()
            return
        if not url.path.startswith('/api/'):
            super().do_GET()
            return
        if not self.allowed():
            self.reply(HTTPStatus.FORBIDDEN, {'error': 'forbidden'})
            return
        if url.path == '/api/info':
            job = State.job
            self.reply(HTTPStatus.OK, {'version': 1, 'root': str(ROOT), 'tools': find_tools(), 'builds': builds(),
                                       'probes': list(PROBES), 'job': job.view(len(job.lines)) if job else None})
        elif url.path == '/api/job':
            job, query = State.job, parse_qs(url.query)
            if not job or str(job.id) != query.get('id', [''])[0]:
                self.reply(HTTPStatus.NOT_FOUND, {'error': 'no such job'})
                return
            start = int(query.get('from', ['0'])[0] or 0)
            self.reply(HTTPStatus.OK, job.view(max(0, start)))
        else:
            self.reply(HTTPStatus.NOT_FOUND, {'error': 'unknown'})

    def do_POST(self):
        url = urlparse(self.path)
        if not self.allowed():
            self.reply(HTTPStatus.FORBIDDEN, {'error': 'forbidden'})
            return
        length = min(int(self.headers.get('Content-Length') or 0), 4096)
        try:
            body = json.loads(self.rfile.read(length) or b'{}')
        except ValueError:
            body = {}
        if url.path == '/api/job':
            action, mode, probe = body.get('action'), body.get('mode'), body.get('probe', 'cmsis-dap')
            if action not in ACTIONS or mode not in MODES or probe not in PROBES:
                self.reply(HTTPStatus.BAD_REQUEST, {'error': 'bad request'})
                return
            with State.lock:
                if State.job and not State.job.done:
                    self.reply(HTTPStatus.CONFLICT, {'error': 'busy', 'id': State.job.id})
                    return
                State.job = Job(State.next_id, action, mode, probe)
                State.next_id += 1
                threading.Thread(target=State.job.run, daemon=True).start()
                self.reply(HTTPStatus.OK, {'id': State.job.id})
        elif url.path == '/api/cancel':
            job = State.job
            if job and not job.done:
                job.cancel()
            self.reply(HTTPStatus.OK, {'ok': True})
        else:
            self.reply(HTTPStatus.NOT_FOUND, {'error': 'unknown'})


def main():
    parser = argparse.ArgumentParser(description='2026AGV 调试台的本地助手（编译、烧录）')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true', help='不自动打开浏览器')
    args = parser.parse_args()
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except AttributeError:
        pass
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    url = f'http://localhost:{args.port}/agv_console.html'
    tools = find_tools()
    print(f'调试台：{url}（用 Chrome 或 Edge 打开，串口要用 Web Serial）')
    for name, path in tools.items():
        print(f'  {name:18} {path or "没找到（编译/烧录要用它）"}')
    print('Ctrl+C 退出。')
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        job = State.job
        if job and not job.done:
            job.cancel()


if __name__ == '__main__':
    main()
