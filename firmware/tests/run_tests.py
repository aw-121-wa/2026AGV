"""Build and run all host-side tests (gcc, fake HAL in tests/host).

Usage: python3 tests/run_tests.py
Set AGV_HOST_BUILD to build the binaries somewhere else (e.g. a writable temp directory).
"""
import os
from pathlib import Path
import subprocess
import sys

ROOT =Path(__file__).resolve().parents[1]
BUILD = Path(os.environ.get('AGV_HOST_BUILD') or ROOT / 'build' / 'host')
INCLUDES = ['tests/host', 'User/Config', 'User/Common', 'User/Drivers/HWT101',
            'User/Drivers/ZDT_X42S', 'User/Drivers/RPLidar', 'User/Drivers/SerPort',
            'User/Drivers/Servo', 'User/Chassis', 'User/Arm',
            'User/Comm', 'User/Navigation', 'User/App']
CFLAGS = ['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror'] + ['-I' + i for i in INCLUDES]
NAV = ['User/Navigation/map.c', 'User/Navigation/astar.c', 'User/Navigation/survey.c']

SUITES = {
    'zdt_test': [],
    'chassis_test': [],
    'path_test': [],
    'command_path_test': [],
    'imu_dma_test': [],
    'map_test': ['User/Navigation/map.c'],
    'planner_test': NAV[:2],
    'survey_test': ['User/Navigation/survey.c'],
    'agv_test': NAV + ['User/App/agv_run.c'],
    'task_test': ['User/App/task_code.c'],
    'console_test': [],
    'comp_test': ['User/Navigation/map.c', 'User/Navigation/lidar_obstacle.c',
                  'User/Drivers/RPLidar/rplidar.c', 'User/Comm/vision_link.c'],
}


def run(name, sources, defines=(), args=()):
    exe = BUILD / name
    subprocess.run(CFLAGS + list(defines) + ['tests/host/' + name.split('@')[0] + '.c'] + sources +
                   ['-lm', '-o', str(exe)], cwd=ROOT, check=True)
    subprocess.run([str(exe)] + list(args), cwd=ROOT, check=True, timeout=60)


def node_tests(fixture):
    """调试台的调参算法（tools/pid_tune.js）：用 tune_log_test 记下的假车日志检查。"""
    try:
        subprocess.run(['node', '--version'], capture_output=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        print('SKIP tools/pid_tune_test.js: node not found')
        return
    subprocess.run(['node', str(ROOT.parent / 'tools' / 'pid_tune_test.js'), str(fixture)],
                   cwd=ROOT, check=True, timeout=300)


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    for name, sources in SUITES.items():
        run(name, sources)
    run('bench_test@imu', [])
    run('bench_test@noimu', [], ['-DCHASSIS_USE_IMU=0'])
    run('console_test@noimu', [], ['-DCHASSIS_USE_IMU=0'])
    fixture = BUILD / 'tune_fixture.log'
    run('tune_log_test', [], args=[str(fixture)])
    node_tests(fixture)
    for mode in range(4):
        run('mode_test@' + str(mode), [], ['-DAGV_MODE=' + str(mode)])
    run('survey_loop_test', NAV + ['User/App/agv_run.c'], ['-DAGV_MODE=1'])
    run('comp_task_test', ['User/App/competition.c', 'User/App/task_code.c',
                           'User/Comm/console.c', 'User/Comm/tune_log.c',
                           'User/Navigation/map.c', 'User/Navigation/astar.c',
                           'User/Navigation/lidar_obstacle.c',
                           'User/Drivers/RPLidar/rplidar.c', 'User/Comm/vision_link.c'],
        ['-DAGV_MODE=2'])
    run('comp_route_test', NAV + ['User/App/agv_run.c', 'User/App/competition.c',
                                  'User/App/task_code.c', 'User/Navigation/lidar_obstacle.c',
                                  'User/Drivers/RPLidar/rplidar.c', 'User/Comm/vision_link.c'],
        ['-DAGV_MODE=2'])
    run('arm_test', ['User/Comm/console.c', 'User/App/arm_app.c', 'User/Comm/vision_link.c'],
        ['-DAGV_MODE=3'])
    subprocess.run([sys.executable, 'tests/vision_protocol_test.py'], cwd=ROOT, check=True,
                   timeout=60)
    print('ALL HOST TESTS PASSED')


if __name__ == '__main__':
    main()
