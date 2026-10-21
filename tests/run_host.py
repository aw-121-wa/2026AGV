"""Compile the production chassis and command sources against fake HAL/motors."""
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
for name in ('chassis_test', 'imu_dma_test'):
    out = root / 'build' / ('host_' + name + '.exe')
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
        '-Itests/host', '-IAPP/Chassis_motor', '-IAPP/HWT101', '-IAPP/ZDT_X42S',
        'tests/host/' + name + '.c', '-lm', '-o', str(out)], cwd=root, check=True)
    subprocess.run([str(out)], cwd=root, check=True)
