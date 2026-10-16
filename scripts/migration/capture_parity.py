#!/usr/bin/env python3
"""Capture offline golden outputs from an explicitly selected legacy checkout.

Only the fixture-generation tool loads legacy Python sources. The installed
runtime and ordinary fixture tests do not depend on that checkout.
"""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import subprocess
import sys
import types


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def generate(root):
    import numpy as np
    # Deliberate source inspection, not a deployed package path fallback.
    sys.path[:0] = [str(root/'deployment/franky_runtime'), str(root/'source/franka_motion')]
    from franky_experiment.core import action_to_target, action_to_velocity, assemble_policy_observation, SessionSupervisor
    from franky_experiment.backend import EncoderForwardKinematics, _encoder_pose_feedback
    from franka_motion.impedance import JointImpedanceController, TorqueCommandProcessor, dynamic_torque_projector
    from franka_motion.kinematics import UrdfKinematicsProvider, nullspace_projector
    from franka_motion.virtual_fixture import CurveVirtualFixture
    cfg={'action_mapping': {'type':'normalized_position_increment','policy_joint_count':6,
           'max_position_increment_rad':[.0087]*4+[.01044]*2},
         'safety':{'soft_lower_rad':[-1.]*7,'soft_upper_rad':[1.]*7}}
    action=[1,-1,.5,-.5,0,1]; prior=[.999,-.999,0,0,0,0,.37]
    pos=action_to_target(cfg,action,prior)
    cfgv={'action_mapping':{'type':'normalized_joint_velocity','policy_joint_count':6,
                           'max_reference_velocity_rad_s':[.435]*4+[.522]*2}}
    vel=action_to_velocity(cfgv,action)
    obs_inputs=dict(q=[.1]*7,dq=[.2]*7,target=[.5,.1,.4],flange=[.4,.08,.35],reference=prior,
                    previous=action,default=[0]*7,lower=[-1]*7,upper=[1]*7,controlled_joint_count=6)
    joint_inputs=dict(q=[0]*7,dq=[.1]*7,q_ref=[.01]*7,dq_ref=[.2]*7,coriolis=[.1]*7,gravity=[2]*7)
    jc=JointImpedanceController([100]*7,[20]*7)
    joint_out=jc.compute(**joint_inputs)
    proc=TorqueCommandProcessor(); samples=[]
    for requested in ([100]*7,[100]*7,[-100]*7):
        out=proc.compute(requested,[2]*7)
        samples.append({'requested':requested, 'gravity':[2]*7, 'output':out})
    # Reset one instance, preserve the other: later batched reset must match this.
    other=TorqueCommandProcessor(); other.compute([100]*7,[0]*7)
    proc.reset(); reset_a=proc.compute([100]*7,[0]*7); retained_b=other.compute([100]*7,[0]*7)
    j=np.c_[np.eye(6),np.ones(6)*.2]; m=np.diag([1,2,3,4,5,6,7])
    projectors={'jacobian':j,'mass':m,'kinematic':nullspace_projector(j), 'dynamic':dynamic_torque_projector(j,m)}
    fixtures=[]
    for kind in ('z_line','yz_circle'):
        f=CurveVirtualFixture(kind,[.4,0,.4],.1); p=[.42,.08,.49]; v=[.01,.02,.03]
        projected,tangent=f.project(p)
        fixtures.append({'kind':kind,'origin':[.4,0,.4],'radius':.1,'position':p,'velocity':v,
                         'projected':projected,'tangent':tangent,'gains':[100,20,10],
                         'force':f.force(p,v,100,20,10)})
    urdf=root/'deployment/hardware_control_audit/2026-09-25/robot_returned_model.urdf'
    fk=UrdfKinematicsProvider(urdf)
    q=[0,-.785398,0,-2.356194,0,1.570796,.785398]
    p,quat,jac=fk.evaluate(q)
    # Exercise the real backend's measured-q/model-frame arguments with a fake model.
    class Affine:
        def __init__(self): self.matrix=np.eye(4)
    class Model:
        def pose(self,frame,q_arg,a,b):
            self.call={'frame':frame,'q':list(q_arg),'F_T_EE':a.matrix,'EE_T_K':b.matrix}
            return types.SimpleNamespace(matrix=np.block([[np.eye(3),np.array([[.4],[.1],[.5]])],[np.zeros((1,3)),np.ones((1,1))]]),quaternion=[0,0,0,1])
    model=Model(); module=types.SimpleNamespace(Frame=types.SimpleNamespace(Flange='Frame.Flange'),Affine=Affine)
    encoder=EncoderForwardKinematics(types.SimpleNamespace(model=model),module)
    reported=np.eye(4);reported[:3,3]=[.4,.1,.6]
    tool=np.eye(4);tool[2,3]=.1
    state=types.SimpleNamespace(q=q,O_T_EE=reported,F_T_EE=tool)
    feedback=_encoder_pose_feedback(state,encoder)
    supervisor=SessionSupervisor({}); supervisor.state='ready'; supervisor.start(); supervisor.fault('callback_gap')
    supervisor.stop();supervisor.complete();supervisor.fault('later_error')
    sources=['deployment/franky_runtime/franky_experiment/'+n for n in ('core.py','backend.py','path_runtime.py')]
    sources += ['source/franka_motion/franka_motion/'+n for n in ('impedance.py','kinematics.py','virtual_fixture.py','contracts.py')]
    sources += [str(urdf.relative_to(root))]
    return {'schema_version':1,'hardware_accessed':False,'native_equivalence_claimed':False,
        'source_commit':subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip(),
        'source_hashes':{p:sha(root/p) for p in sources},
        'tolerance':{'atol':1e-10,'purpose':'numerical regression only; not operating limits'},
        'reference':{'action':action,'prior':prior,'position_mapping':cfg,'position_target':pos,
                     'velocity_mapping':cfgv,'velocity_target':vel},
        'observation':{'inputs':obs_inputs,'values':assemble_policy_observation(**obs_inputs)},
        'joint_controller':{'stiffness':[100]*7,'damping':[20]*7,'inputs':joint_inputs,'output':joint_out},
        'torque_processing':samples,'selective_reset':{'reset_instance':reset_a,'untouched_instance':retained_b},
        'projectors':projectors,'curve_fixtures':fixtures,
        'urdf_fk':{'model_sha256':sha(urdf),'joint_names':fk.joint_names,'q':q,'position':p,'quaternion_xyzw':quat,'jacobian':jac,
                   'base_link':'link0','tip_link':'link8'},
        'encoder_fk_adapter':{'model_call':model.call,'position':feedback.position_m,'quaternion_xyzw':feedback.orientation_xyzw,
                              'reported_position_error_m':feedback.reported_position_error_m,
                              'qualification':'fake model adapter semantics only; not calibrated FK validation'},
        'fault_latch':{'operations':['ready','start','fault:callback_gap','stop','complete','fault:later_error'],
                       'final_state':supervisor.state,'final_reason':supervisor.reason},
        'stop_fixture_scope':'Native stop behavior is covered by the separately captured native test receipt; no Python stop substitute.'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    def convert(value):
        if hasattr(value,'tolist'):return value.tolist()
        raise TypeError(type(value).__name__)
    result=generate(a.source.resolve())
    with a.output.open('x') as f:json.dump(result,f,indent=2,sort_keys=True,default=convert,allow_nan=False);f.write('\n')

if __name__=='__main__':main()
