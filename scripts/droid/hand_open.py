"""Open the Franka Hand to the commissioned open width before an episode (Hand motion only).

The runtime never moves the Hand on its own; an episode that ended with a grasp leaves it
closed. Uses the pinned native Franky artifact; constructs only a Gripper.
"""
import argparse
import json
from pathlib import Path
from franka_runtime.droid.native import verify_native_artifact
from franka_runtime.droid.physical_config import PhysicalDroidConfig


def open_hand(*,config,authorize_hand_motion=False):
    if authorize_hand_motion is not True:raise ValueError('Explicit Hand motion authorization required')
    c=PhysicalDroidConfig.load(config);d=c.require_approved();g=d['gripper']
    sdk=verify_native_artifact(c.path(d['artifacts']['native_pin']))
    gripper=sdk.Gripper(d['robot']['address'])
    before=float(gripper.state.width);ok=gripper.move(g['open_width_m'],g['speed_m_s']);after=float(gripper.state.width)
    return dict(before_width_m=before,after_width_m=after,move_returned=ok,open_width_m=g['open_width_m'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--authorize-hand-motion',action='store_true')
    print(json.dumps(open_hand(**vars(p.parse_args()))))


if __name__=='__main__':main()
