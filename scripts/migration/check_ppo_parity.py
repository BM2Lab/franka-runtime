"""Explicit offline bundle/reference parity; never constructs a robot."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import yaml
from learning.artifacts.franka_bundle.bundle import verify
from learning.artifacts.franka_bundle.contract import action_mapping
from learning.integrations.onnx.bridge import WorkerBridge,raw_ns
from franka_runtime.common.motion.legacy import assemble_policy_observation,action_to_target,action_to_velocity
from contracts.franka_policy import VELOCITY_ID


def check(root,python):
    digest=hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()
    report=verify(root,digest);c=yaml.safe_load((root/'policy_contract.yaml').read_text())
    mapping={'action_mapping':{'type':c['action']['type'],'policy_joint_count':6,**c['action']},'safety':{k:c['action'][k] for k in ('soft_lower_rad','soft_upper_rad')}}
    with np.load(root/'test_vectors.npz',allow_pickle=False) as v:
        assembled=np.asarray([assemble_policy_observation(q,dq,target,hand,ref,prev,c['default_joint_position_rad'],c['action']['soft_lower_rad'],c['action']['soft_upper_rad'],6)
                              for q,dq,target,hand,ref,prev in zip(v['q'],v['dq'],v['target'],v['hand'],v['reference'],v['previous_action'])],dtype=np.float32)
        observation_error=float(np.max(abs(assembled-v['observation'])))
        np.testing.assert_allclose(assembled,v['observation'],rtol=0,atol=1e-6)
        mapped=[]
        for action,ref in zip(v['action'],v['reference']):
            if c['contract_id']==VELOCITY_ID:
                velocity=action_to_velocity(mapping,action.tolist())
                assert velocity[6]==0
                value=velocity[:6]  # Legacy q_policy key stores velocities for this contract.
            else:value=action_to_target(mapping,action.tolist(),ref.tolist())
            mapped.append(value)
        np.testing.assert_allclose(mapped,v['q_policy'],rtol=0,atol=1e-6)
        mapping_error=float(np.max(abs(np.asarray(mapped)-v['q_policy'])))
        # Live IPC handles all zero/per-input sentinel vectors; batch verification
        # above also covers all stored seeded-random/replay vectors.
        bridge=WorkerBridge(str(python),str(root),digest)
        errors=[]
        try:
            for index,obs in enumerate(v['observation'][:30]):
                now=raw_ns();request=dict(request_id=index+1,trial_id=1,observation_sequence=index+1,observation_monotonic_ns=now,session_id='offline-parity',episode_id='sentinels',observation=obs.tolist())
                bridge.submit(request);deadline=time.monotonic()+2
                while True:
                    reply=bridge.poll()
                    if reply is not None:break
                    if time.monotonic()>deadline:raise RuntimeError('Offline parity worker timeout')
                    time.sleep(.0005)
                errors.append(float(np.max(abs(np.asarray(reply['action'])-v['action'][index]))))
        finally:bridge.close()
        assert max(errors)<=1e-5
    return {**report,'bundle_id':root.name,'runtime_observation_max_abs_error':observation_error,'runtime_action_mapping_max_abs_error':mapping_error,'worker_sentinel_count':len(errors),'worker_max_abs_error':max(errors)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,action='append',required=True);p.add_argument('--worker-python',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result={'schema_version':1,'hardware_accessed':False,'bundles':[check(root,a.worker_python) for root in a.bundle]}
    with a.output.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
