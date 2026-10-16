"""Explicit offline native API check; Robot creation and scheduling are forbidden."""
from pathlib import Path
from types import SimpleNamespace
from contextlib import nullcontext
import json
import sys
import numpy as np
import franky as f
from franka_runtime.common.backends import franky_m4 as h
from franka_runtime.common.motion.dls_tracking import resolved_rate

root=Path(sys.argv[1]);output=Path(sys.argv[2]);routes=[]
def forbidden(*args,**kwargs):raise AssertionError('Robot constructor forbidden in offline smoke')
f.Robot=forbidden
h.preserve_caller_scheduler=lambda **kwargs:nullcontext()
class RobotDouble:
    model=object()
    def move(self,motion,**kwargs):
        assert kwargs=={'asynchronous':True,'limit_rate':False,'cutoff_frequency':100.}
        self.kwargs=kwargs
snapshot=SimpleNamespace(q=[0.,-.7853981633974483,0.,-2.356194490192345,0.,1.5707963267948966,0.],
                         flange_position_m=[.30689056659294117,0.,.5902820523028393],
                         flange_orientation_xyzw=[1.,0.,0.,0.])
for name in ('hold','tracking','dls','posture','z_line','yz_circle'):
    c=h.load_hardware_config(root/f'configs/fr3/m4.{name}.pending.yaml')
    # In-memory fake execution approval only; move() is a Python recorder above.
    c.update(status='approved',executable=True,motion_authorized=True)
    c['approval']={k:'OFFLINE_TEST_DOUBLE_ONLY' for k in ('approved_by','approved_at','physical_stop_procedure','workspace_review')}
    robot=RobotDouble();b=h.FrankyMotionBackend(c,robot=robot,franky_module=f)
    b.start(snapshot)
    assert b.motion.fault==0 and b.sequence==1 and not b.motion.drain()
    b.motion.request_stop()
    routes.append({'route':name,'native_constructor_passed':True,'robot_created':False,'move_kwargs':robot.kwargs})
rng=np.random.default_rng(7);cases=[]
for kind in ('full_rank','rank_deficient','velocity_scaled'):
    j=rng.normal(size=(6,7))
    if kind=='rank_deficient':j[5]=j[4]
    p=f.M4DlsParams();p.task_scale=[1,1,1,.2,.2,.2]
    e=np.arange(6)*.001;v=e*.2;posture=np.arange(7)*.01;bounds=np.ones(7)*(.001 if kind=='velocity_scaled' else .4)
    actual=f.compute_m4_dls_step(j,e,v,posture,bounds,p)
    expected=resolved_rate(j,e,v,posture,bounds,pose_gain=p.pose_gain,task_scale=p.task_scale,
        lambda_value=p.lambda_value,rank_tolerance=p.rank_tolerance,posture_gain=p.posture_gain)
    np.testing.assert_allclose(actual[0],expected[0],rtol=0,atol=1e-12)
    np.testing.assert_allclose(actual[1],expected[1],rtol=0,atol=1e-12)
    assert actual[3]==expected[3]
    cases.append({'case':kind,'velocity_max_error':float(np.max(abs(actual[0]-expected[0]))),'rank':int(actual[3])})
with output.open('x') as stream:json.dump({'hardware_accessed':False,'native_rebuilt':False,'routes':routes,'dls':cases},stream,indent=2);stream.write('\n')
print('Offline native API passed: six routes, three DLS cases; no Robot or RT scheduler changes.')
