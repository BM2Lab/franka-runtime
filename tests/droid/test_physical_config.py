"""Synthetic commissioning fixtures; never connect to hardware."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
from contracts.droid import DroidIOSpec
from contracts.sessions import payload_sha256
SPEC=DroidIOSpec.from_json((Path(__file__).resolve().parents[2]/"configs/fr3/droid_fr3_jointpos_v1.contract.json").read_text())
import pytest
from franka_runtime.droid.physical_config import PhysicalDroidConfig,pending_template,digest
from franka_runtime.droid.physical_factory import PhysicalDeviceFactory,build_motion


def complete(tmp_path):
    d=pending_template()
    for section,items in d.items():
        if isinstance(items,dict):
            for key in items:items[key]=1
    d['robot'].update(robot_id=SPEC.robot.robot_id,address='test.invalid',model='test',firmware_version='5.9.2',fci_version=10,arm_serial='test-arm',control_serial='test-control',identity_max_age_ns=1_000_000_000)
    d['contract'].update(allowed_sha256=[payload_sha256(SPEC)],capability_prefix='test')
    d['joint_bounds'].update(lower_rad=[-2.]*7,upper_rad=[2.]*7,margin_rad=[.1]*7)
    d['acceptance']={k:[.5]*7 for k in d['acceptance']}
    d['controller'].update(kind='droid_hybrid_joint_impedance',joint_stiffness=[50.]*7,joint_damping=[10.]*7,cartesian_stiffness=[1.]*6,cartesian_damping=[1.]*6,error_clip_rad=[.5]*7,gain_time_constant_s=.1,compensate_coriolis=True,libfranka_limit_rate=True)
    d['guards'].update(max_measured_joint_velocity_rad_s=[1.]*7,max_command_torque_nm=[10.]*7,max_callback_gap_ns=5_000_000,publisher_timeout_ns=50_000_000)
    d['stop'].update(damping=[10.]*7,ramp_duration_s=.2,max_duration_s=2.,rest_velocity_rad_s=.01,compensate_coriolis=True)
    d['timing'].update(control_hz=100,startup_timeout_ns=1_000_000_000,consumption_timeout_ns=50_000_000,stop_timeout_ns=3_000_000_000,state_timeout_ns=50_000_000,native_log_capacity=1000)
    d['gripper'].update(open_width_m=.08,close_width_m=0.,poll_hz=50,max_state_age_ns=100_000_000,moving_epsilon_m_s=.001)
    d['tool'].update(F_T_EE=[[float(i==j) for j in range(4)] for i in range(4)],I_ee=[0.]*9,I_load=[0.]*9,F_x_Cee=[0.]*3,F_x_Cload=[0.]*3,scalar_tolerance=.001,transform_tolerance=.001)
    urdf=tmp_path/'model.urdf'
    urdf.write_text('<robot name="test">'+''.join(f'<joint name="panda_joint{i}" type="revolute"><parent link="l{i-1}"/><child link="l{i}"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/></joint>' for i in range(1,8))+'</robot>')
    w=d['workspace'];w.update(model_urdf=str(urdf),model_sha256=digest(urdf),base_link='l0',flange_link='l7',joint_names=list(SPEC.robot.joint_names),link_points=['l7'],tool_points_m=[[0.,0.,0.]],table_z_m=-1.,clearance_m=.01,box_min_m=[-2.]*3,box_max_m=[2.]*3)
    artifact=tmp_path/'artifact.json';artifact.write_text('{}')
    d['artifacts'].update(native_pin=str(artifact),native_pin_sha256=digest(artifact),collision_model=str(artifact),collision_model_sha256=digest(artifact))
    d['network'].update(bind_host='127.0.0.1',port=12345,allowed_peers=['127.0.0.1'],token_env='TEST_TOKEN')
    d['recording'].update(directory=str(tmp_path/'recording'),queue_capacity=128,max_record_bytes=16384,max_event_bytes=1_000_000,min_free_bytes=1,fsync_interval_s=.01,close_timeout_s=1.)
    d['approval']={k:'synthetic test fixture' for k in d['approval']}
    d.update(status='reviewed',executable=True,motion_authorized=True)
    return d


def test_pending_is_nonexecutable():
    c=PhysicalDroidConfig(pending_template())
    assert len(c.missing())>80
    with pytest.raises(ValueError,match='Uncommissioned'):c.require_approved()
    with pytest.raises(ValueError,match='disabled'):c.require_runnable()


def test_complete_and_deepcopy(tmp_path):
    d=complete(tmp_path);c=PhysicalDroidConfig(d);d['robot']['address']='changed'
    assert c.require_approved()['robot']['address']=='test.invalid'
    with pytest.raises(ValueError,match='disabled'):c.require_runnable()


@pytest.mark.parametrize('section,key,value',[
 ('contract','allowed_sha256','bad'),('workspace','joint_names',[{}]*7),('workspace','link_points',1),
 ('controller','joint_stiffness',[float('nan')]*7),('controller','libfranka_limit_rate',1),
 ('guards','max_measured_joint_velocity_rad_s',[0.]*7),('robot','fci_version',6),
 ('tool','F_T_EE',[[0.]*4]*4),('network','bind_host','0.0.0.0'),('timing','stop_timeout_ns',1)])
def test_reject_malformed(tmp_path,section,key,value):
    d=complete(tmp_path);d[section][key]=value
    with pytest.raises(ValueError):PhysicalDroidConfig(d).validated()


def test_artifact_change(tmp_path):
    d=complete(tmp_path);Path(d['workspace']['model_urdf']).write_text('changed')
    with pytest.raises(ValueError,match='Artifact mismatch'):PhysicalDroidConfig(d).validated()


def test_factory_admission_before_constructor(tmp_path):
    d=complete(tmp_path);calls=[]
    factory=PhysicalDeviceFactory(PhysicalDroidConfig(d),clock=lambda:100,clock_id='test',identity_reader=lambda a:{},collision_checker=None,artifact_verifier=lambda p:calls.append(p))
    with pytest.raises(ValueError,match='authorization'):factory.connect(None,spec=SPEC)
    with pytest.raises(ValueError,match='collision'):factory.connect(None,spec=SPEC,authorize_device_setup=True)
    assert calls==[]


def test_identity_rejects_ip_only_and_stale(tmp_path):
    d=complete(tmp_path);v={k:d['robot'][k] for k in ('address','model','firmware_version','fci_version','arm_serial','control_serial')}
    v.update(clock_id='test',observed_ns=100)
    f=PhysicalDeviceFactory(PhysicalDroidConfig(d),clock=lambda:101,clock_id='test',identity_reader=lambda a:v,collision_checker=None)
    f._identity(d);v['arm_serial']='wrong'
    with pytest.raises(ValueError,match='arm_serial'):f._identity(d)
    v['arm_serial']=d['robot']['arm_serial'];v['observed_ns']=102
    with pytest.raises(ValueError,match='stale'):f._identity(d)


def test_native_parameter_mapping_without_devices(tmp_path):
    d=complete(tmp_path);args=[]
    sdk=NS(JointImpedanceParams=lambda:NS(safety=NS()),DroidJointLimits=NS,TorqueStopParams=NS,
        CartesianImpedanceGains=NS(diagonal=lambda k,b:(k,b)),DroidJointImpedanceMotion=lambda *a:args.append(a))
    build_motion(sdk,PhysicalDroidConfig(d))
    p,limits,stop,capacity,tau=args[0]
    assert limits.lower==[-1.9]*7 and limits.velocity==[1.]*7
    assert p.damping==[10.]*7 and p.constant_torque_offset==[0.]*7
    assert stop.velocity_epsilon==.01 and capacity==1000 and tau==.1


def test_factory_connect_is_passive_until_start_and_refreshes(tmp_path):
    d=complete(tmp_path);c=PhysicalDroidConfig(d);calls=[]
    state=NS(q=[0.]*7,dq=[.01]*7,robot_mode='idle',current_errors=NS(joint_reflex=False),F_T_EE=NS(matrix=d['tool']['F_T_EE']),
             **{k:d['tool'][k] for k in ('m_ee','m_load','I_ee','I_load','F_x_Cee','F_x_Cload')})
    robot=NS(state=state)
    count=[0]
    class Gripper:
        @property
        def state(self):
            count[0]+=1
            return NS(width=.08,is_grasped=False,time=NS(to_msec=lambda:count[0]))
        def move(self,*args):raise AssertionError('No commands before start')
        def grasp(self,*args):raise AssertionError('No commands before start')
        def stop(self):return True
    sdk=NS(Robot=lambda *a,**kw:(calls.append(kw) or robot),Gripper=lambda a:Gripper(),RobotMode=NS(Idle='idle'),RealtimeConfig=NS(Enforce='enforce'),CommandException=RuntimeError)
    clock=[100]
    def now():clock[0]+=1;return clock[0]
    def identity(a):
        return dict(**{k:d['robot'][k] for k in ('address','model','firmware_version','fci_version','arm_serial','control_serial')},clock_id='test',observed_ns=clock[0])
    checker=NS(artifact_sha256=d['artifacts']['collision_model_sha256'],check_configuration=lambda q:None)
    f=PhysicalDeviceFactory(c,clock=now,clock_id='test',identity_reader=identity,collision_checker=checker,artifact_verifier=lambda p:sdk)
    b=f.connect(None,spec=SPEC,authorize_device_setup=True)
    assert b.driver.thread is None and b.hand._thread is None and b.arm.dq==(.01,)*7
    assert calls==[dict(default_torque_threshold=1,default_force_threshold=1,realtime_config='enforce')]
    state.q=[.1]*7;b.estimate=object();assert b.preflight()['q']==(.1,)*7
    np=pytest.importorskip('numpy')  # Franky's shapes: I_* 3x3, F_x_C* 3x1, column-major like libfranka
    state.I_ee=np.array(d['tool']['I_ee'],dtype=float).reshape(3,3,order='F');state.F_x_Cee=np.array(d['tool']['F_x_Cee'],dtype=float).reshape(3,1)
    state.I_load=np.array(d['tool']['I_load'],dtype=float).reshape(3,3,order='F');assert b.preflight()['q']==(.1,)*7
    state.I_ee=state.I_ee+1e-2
    with pytest.raises(ValueError,match='I_ee'):b.preflight()
    state.I_ee=np.array(d['tool']['I_ee'],dtype=float).reshape(3,3,order='F')
    state.m_load=99
    with pytest.raises(ValueError,match='tool/load'):b.preflight()


def test_physical_recorder_construction_is_passive_and_bound(tmp_path):
    from franka_runtime.droid.recording import from_physical_config
    c=PhysicalDroidConfig(complete(tmp_path))
    r=from_physical_config(c,clock_id='test:monotonic',contract_sha256=payload_sha256(SPEC))
    assert r._thread is None and not r.directory.exists()
    assert r.metadata['configuration_sha256']==c.sha256
    with pytest.raises(ValueError,match='contract/clock'):
        from_physical_config(c,clock_id='test',contract_sha256='f'*64)
