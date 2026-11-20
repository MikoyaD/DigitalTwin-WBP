#!/usr/bin/env python3
"""Derive Gazebo's live-state RViz layout from the existing V2 preview layout."""
import os
from pathlib import Path
import subprocess
import tempfile
import yaml

V2 = Path(__file__).resolve().parents[1]


def live_config():
    with (V2 / 'rviz/v2_g.rviz').open() as source:
        config = yaml.safe_load(source)
    displays = config['Visualization Manager']['Displays']
    robot = next(d for d in displays if d.get('Class') == 'rviz/RobotModel')
    robot.update(Name='Gazebo actual state', Enabled=True, Value=True)
    robot['Visual Enabled'] = True
    camera = next(d for d in displays if d.get('Topic') == '/realsense/depth/points')
    camera.update(Enabled=False, Value=False)
    octomap = next(d for d in displays if d.get('Topic') == '/octomap_point_cloud_centers')
    octomap.update(Name='V2 warehouse occupied centers', Enabled=True, Value=True)
    displays.insert(displays.index(octomap) + 1, {'Class':'rviz/MarkerArray','Name':'Fixed warehouse OctoMap',
                    'Enabled':False,'Value':False,'Marker Topic':'/occupied_cells_vis_array'})
    # Preview-only animation plugins belong to RViz-only, not the live robot view.
    displays[:] = [d for d in displays if d.get('Class') not in
                   ('moveit_rviz_plugin/Trajectory', 'moveit_rviz_plugin/MotionPlanning')]
    config['Visualization Manager']['Global Options']['Fixed Frame'] = 'world'
    return config


def main():
    # The child inherits ROS master and display from the caller. It is the sole RViz node.
    with tempfile.TemporaryDirectory(prefix='v2_replay_rviz_') as folder:
        config = Path(folder) / 'gazebo_live.rviz'
        with config.open('w') as f: yaml.safe_dump(live_config(),f,sort_keys=False,allow_unicode=True)
        return subprocess.call(['rviz','-d',str(config)] + os.sys.argv[1:])

if __name__ == '__main__': raise SystemExit(main())
