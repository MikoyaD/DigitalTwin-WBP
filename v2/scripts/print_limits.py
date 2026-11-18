"""Print the effective backend LimitSet of a configuration without starting ROS."""
import argparse,json
from dataclasses import asdict
from pathlib import Path
import yaml
from ..core.json_io import compatible
from .stage_v2 import limits_for


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);args=p.parse_args()
    config=yaml.safe_load(Path(args.config).read_text())
    print(json.dumps(compatible(asdict(limits_for({'config':config}))),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
