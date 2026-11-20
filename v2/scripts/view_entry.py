"""Start/reuse V2 scene, then one interactive viewer and optional RViz; no planning outputs."""
import argparse
import os
import subprocess
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
    args=parser.parse_args()
    if args.gazebo_csv:
        from .gazebo_csv_replay import run
        if args.replay_open_rviz and args.no_rviz:parser.error('--replay-open-rviz conflicts with --no-rviz')
        return run(args.gazebo_csv,args.replay_output,args.replay_open_rviz)
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
