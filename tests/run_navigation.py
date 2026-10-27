"""Compile real navigation modules on host; no hardware access."""
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
common = ['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itests/host',
          '-IAstar','-IAPP/Chassis_motor','-IAPP/ZDT_X42S']
cases = {
 'map_test': ['Astar/map.c'],
 'planner_test': ['Astar/map.c','Astar/astar.c'],
 'survey_test': ['Astar/survey.c'],
 'agv_test': ['Astar/map.c','Astar/astar.c','Astar/survey.c','Astar/agv_run.c'],
}
for name,sources in cases.items():
    out = root/'build'/('host_'+name+'.exe')
    subprocess.run(common+['tests/host/'+name+'.c']+sources+['-lm','-o',str(out)],cwd=root,check=True)
    subprocess.run([str(out)],cwd=root,check=True,timeout=30)
for mode in range(3):
    out=root/'build'/('host_mode_'+str(mode)+'.exe')
    subprocess.run(common+['-DAGV_MODE='+str(mode),'tests/host/mode_test.c','-o',str(out)],cwd=root,check=True)
    subprocess.run([str(out)],cwd=root,check=True,timeout=10)
