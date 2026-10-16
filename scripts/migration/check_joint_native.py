"""Offline PPO native constructors; explicit existing modified wheel only."""
import argparse
import json
from pathlib import Path
import yaml
import franky as f
from franka_runtime.common.backends.franky_m4 import verify_native_capabilities
from franka_runtime.common.backends.franky_joint import FrankyBackend,FrankyVelocityImpedanceBackend

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--runtime-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    def forbidden(*args,**kwargs):raise AssertionError('Robot construction forbidden')
    f.Robot=forbidden;results=[]
    for kind,cls in [('position',FrankyBackend),('velocity-impedance',FrankyVelocityImpedanceBackend)]:
        c=yaml.safe_load((a.runtime_root/f'configs/fr3/ppo.{kind}.pending.yaml').read_text());verify_native_capabilities(c,f)
        # Constructor kernels only. No connection, threads or scheduler changes.
        b=cls.__new__(cls);b._franky=f;b.impedance=c['robot']['impedance_controller'];b.soft_lower=c['safety']['soft_lower_rad'];b.soft_upper=c['safety']['soft_upper_rad'];b.stop_config=c['robot']['torque_stop']
        motion=b._create_motion();stop=b._torque_stop_motion()
        assert motion is not None and stop is not None
        results.append({'route':kind,'constructor_passed':True,'stiffness':list(motion.params.stiffness),'damping':list(motion.params.damping)})
    result={'hardware_accessed':False,'native_rebuilt':False,'routes':results}
    with a.output.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
