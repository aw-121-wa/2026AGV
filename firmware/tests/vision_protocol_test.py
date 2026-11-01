"""上位机协议与固件、比赛规则的一致性，不需要相机和串口硬件。

TASK 帧的期望字节与 tests/host/comp_test.c 里单片机解码的那一帧相同，两边共用才算互通。
"""
from pathlib import Path
import sys
import time
import types

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'vision'))


class FakeSerial:
    """只替换串口硬件：记录写出的字节，读不到数据（MCULink 的接收线程会一直读）。"""

    def __init__(self, *args, **kwargs):
        self.sent = bytearray()

    def write(self, data):
        self.sent += data

    def read(self, size):
        time.sleep(0.01)
        return b''


sys.modules['serial'] = types.SimpleNamespace(Serial=FakeSerial)

import config  # noqa: E402
import stm32_link  # noqa: E402


def test_color_numbers_match_rules():
    assert config.COLOR_ID == {'red': 1, 'yellow': 2, 'blue': 3, 'green': 4, 'black': 5,
                               'light_blue': 6}


def test_task_frame_matches_firmware():
    link = stm32_link.MCULink()
    link.send_task('156+123+516+231')
    assert bytes(link.ser.sent) == bytes.fromhex(
        'aa 55 0f 02 00 31 35 36 2b 31 32 33 2b 35 31 36 2b 32 33 31 d6')


def test_task_text_fits_frame():
    link = stm32_link.MCULink()
    link.send_task('x' * 100)
    assert link.ser.sent[2] == stm32_link.MAX_PAYLOAD
    assert len(link.ser.sent) == 6 + stm32_link.MAX_PAYLOAD


def test_non_ascii_qr_text_does_not_raise():
    link = stm32_link.MCULink()
    link.send_task('任务156')
    assert bytes(link.ser.sent[5:-1]) == b'??156'


def main():
    test_color_numbers_match_rules()
    test_task_frame_matches_firmware()
    test_task_text_fits_frame()
    test_non_ascii_qr_text_does_not_raise()
    print('PASS: vision colour numbers, TASK frame bytes, long and non-ASCII QR text')


if __name__ == '__main__':
    main()
