"""Build and run all host-side tests (gcc, fake HAL in tests/host).

Usage: python3 tests/run_tests.py
"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build' / 'host'
INCLUDES = ['tests/host', 'User/Config', 'User/Common', 'User/Drivers/HWT101',
            'User/Drivers/ZDT_X42S', 'User/Chassis', 'User/Comm', 'User/Navigation', 'User/App']
CFLAGS = ['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror'] + ['-I' + i for i in INCLUDES]
NAV = ['User/Navigation/map.c', 'User/Navigation/astar.c', 'User/Navigation/survey.c']

SUITES = {
    'zdt_test': [],
    'chassis_test': [],
    'imu_dma_test': [],
    'map_test': ['User/Navigation/map.c'],
    'planner_test': NAV[:2],
    'survey_test': ['User/Navigation/survey.c'],
    'agv_test': NAV + ['User/App/agv_run.c'],
}


def run(name, sources, defines=()):
    exe = BUILD / name
    subprocess.run(CFLAGS + list(defines) + ['tests/host/' + name.split('@')[0] + '.c'] + sources +
                   ['-lm', '-o', str(exe)], cwd=ROOT, check=True)
    subprocess.run([str(exe)], cwd=ROOT, check=True, timeout=60)


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    for name, sources in SUITES.items():
        run(name, sources)
    for mode in range(3):
        run('mode_test@' + str(mode), [], ['-DAGV_MODE=' + str(mode)])
    print('ALL HOST TESTS PASSED')


if __name__ == '__main__':
    main()
