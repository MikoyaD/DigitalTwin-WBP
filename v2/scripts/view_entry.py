"""Start/reuse V2 scene, then one interactive viewer and optional RViz; no planning outputs."""
import argparse
import os
import subprocess
import sys
from pathlib import Path
import yaml
from .runtime_v2 import V2,start
from ..adapters.trajectory_viewer import interactive,REGISTRY

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',default=str(V2/'configs/v2_workspace_single_goal.yaml'))
    parser.add_argument('--no-rviz',action='store_true',help='reuse an already open RViz')
    parser.add_argument('--wait-subscriber',type=float,default=20.)
    parser.add_argument('--gazebo-csv',help='Replay a standard single-segment CSV on an already-running isolated Gazebo master')
    parser.add_argument('--replay-output',help='Directory for replay observations')
    parser.add_argument('--replay-open-rviz',action='store_true',help='explicitly open an additional RViz for Gazebo replay')
    parser.add_argument('--isaac-read', help='read a P1 recommended CSV or its run directory')
    parser.add_argument('--isaac-map', help='optional Isaac scene USD path, recorded only')
    args=parser.parse_args()
    if args.isaac_map and not args.isaac_read: parser.error('--isaac-map requires --isaac-read')
    if args.isaac_read:
        from .isaac_trajectory import read_and_print
        return read_and_print(args.isaac_read, args.isaac_map)
    if args.gazebo_csv:
        from .gazebo_csv_replay import run
        if args.replay_open_rviz and args.no_rviz:parser.error('--replay-open-rviz conflicts with --no-rviz')
        return run(args.gazebo_csv,args.replay_output,args.replay_open_rviz)
    # Preserve all legacy argument-bearing invocations; the new menu is the
    # no-argument entry only, before rospy connects to either master.
    if len(sys.argv)==1:
        while True:
            try: choice=input('展示模式：1 RViz参考轨迹 | 2 Gazebo+RViz实际状态 | 3 Isaac Sim | q 退出 > ').strip().lower()
            except EOFError:return 0
            if choice in ('q','quit'):return 0
            if choice=='1':break
            if choice=='2':
                from .gazebo_interactive import interactive_gazebo
                interactive_gazebo()
                continue
            if choice=='3':
                from .isaac_trajectory import interactive_isaac
                interactive_isaac()
                continue
            print('请选择 1、2、3 或 q。',flush=True)
    start(yaml.safe_load(Path(args.config).read_text()),for_display=True)
    rviz=None
    try:
        if not args.no_rviz:
            if not os.environ.get('DISPLAY'):raise RuntimeError('DISPLAY_MISSING: use graphical terminal or --no-rviz')
            rviz=subprocess.Popen(['rviz','-d',str(V2/'rviz/v2_g.rviz')],cwd=str(V2.parent))
        args.registry=REGISTRY
        interactive(args)
    finally:
        if rviz is not None and rviz.poll() is None:
            rviz.terminate()
            try:rviz.wait(timeout=5)
            except subprocess.TimeoutExpired:print('RVIZ_STILL_RUNNING: only this viewer child may be stopped manually')

if __name__=='__main__':main()
