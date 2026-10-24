"""Home the Franka Hand once (fingers move to both ends) and record the result.

Hand motion: run only under a stage authorization. Uses the pinned native Franky
artifact; constructs only a Gripper (no arm connection, no arm motion).
"""
import argparse
import json
from pathlib import Path
import time
from franka_runtime.droid.native import verify_native_artifact
from franka_runtime.droid.physical_config import PhysicalDroidConfig


def home(*,config,output,authorize_hand_motion=False):
    if authorize_hand_motion is not True:raise ValueError('Explicit Hand motion authorization required')
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    c=PhysicalDroidConfig.load(config);d=c.require_approved()
    sdk=verify_native_artifact(c.path(d['artifacts']['native_pin']))
    gripper=sdk.Gripper(d['robot']['address'])
    def state():
        s=gripper.state;return dict(width=float(s.width),max_width=float(s.max_width),grasped=bool(s.is_grasped),
                                    temperature=float(s.temperature),time_ms=int(s.time.to_msec()))
    before=state();start=time.monotonic_ns();ok=gripper.homing();elapsed=time.monotonic_ns()-start
    after=state()
    result=dict(schema_version=1,address=d['robot']['address'],homing_returned=ok,duration_s=elapsed/1e9,
                before=before,after=after,open_width_m=d['gripper']['open_width_m'],
                width_error_m=after['max_width']-d['gripper']['open_width_m'])
    output.write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--authorize-hand-motion',action='store_true')
    print(json.dumps(home(**vars(p.parse_args())),indent=2))


if __name__=='__main__':main()
