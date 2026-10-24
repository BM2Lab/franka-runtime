"""D7 configuration: physical candidate is a source of checks, never motion approval."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from .physical_config import PhysicalDroidConfig,integer,number,vector,text
from .workspace import from_settings


class ShadowRuntimeConfig:
    def __init__(self,data,config_dir):self.data=deepcopy(data);self.config_dir=Path(config_dir)
    @classmethod
    def load(cls,path):
        p=Path(path);return cls(json.loads(p.read_text()),p.parent)
    def path(self,value):
        p=Path(value);return p if p.is_absolute() else self.config_dir/p
    def require_runnable(self):
        d=self.data
        if set(d)!={'schema_version','runtime','backend','executable','motion_authorized','read_only','physical_candidate','source','recording_directory'}:
            raise ValueError('Unknown/missing shadow config fields')
        if d['schema_version']!=1 or d['runtime']!='droid_shadow_v1' or d['backend']!='shadow_libfranka':raise ValueError('Expected D7 read-only schema')
        if d['motion_authorized'] is not False or d['read_only'] is not True:raise ValueError('Shadow must forbid motion')
        if d['executable'] is not True:raise ValueError('Review/enable read-only shadow config first')
        source=d['source']
        if set(source)!={'address','helper','helper_sha256','poll_hz','server_version','initial_timeout_s'}:raise ValueError('Explicit reader settings required')
        text(source['address'],'source.address');text(source['helper'],'source.helper');text(source['helper_sha256'],'source.helper_sha256')
        if len(source['helper_sha256'])!=64 or any(c not in '0123456789abcdef' for c in source['helper_sha256']):raise ValueError('Invalid reader SHA256')
        if source['server_version']!=10:raise ValueError('Only server10 reader qualified for this candidate')
        integer(source['poll_hz'],'poll_hz',50)
        if source['poll_hz']>1000:raise ValueError('Reader rate exceeds 1000 Hz')
        number(source['initial_timeout_s'],'initial_timeout_s',strict=True)
        text(d['recording_directory'],'recording_directory')
        p=PhysicalDroidConfig.load(self.path(d['physical_candidate']));r=p.data
        b,t,g,net=r['joint_bounds'],r['timing'],r['gripper'],r['network']
        lo=vector(b['lower_rad'],7,'lower',signed=True);hi=vector(b['upper_rad'],7,'upper',signed=True);margin=vector(b['margin_rad'],7,'margin')
        if any(a+m>=z-m for a,z,m in zip(lo,hi,margin)):raise ValueError('Invalid soft bounds')
        for k in ('control_hz','hold_timeout_ms','heartbeat_timeout_ms','command_timeout_ms','max_lease_s','state_timeout_ns'):integer(t[k],k)
        if t['control_hz']>1000 or t['state_timeout_ns']<=1e9/source['poll_hz']:raise ValueError('Invalid supervisor/freshness cadence')
        for k in ('max_uncertainty_us','min_probes'):integer(r['clock'][k],k)
        number(g['open_width_m'],'open_width',strict=True);number(g['moving_epsilon_m_s'],'moving_epsilon',strict=True)
        integer(g['max_state_age_ns'],'hand_state_age')
        if g['max_state_age_ns']<=1e9/source['poll_hz']:raise ValueError('Hand freshness below source period')
        if net['bind_host'] in ('0.0.0.0','::','') or not isinstance(net['bind_host'],str):raise ValueError('Dedicated bind interface required')
        integer(net['port'],'port');text(net['token_env'],'token_env')
        if net['port']>65535 or not isinstance(net['allowed_peers'],list) or not net['allowed_peers']:raise ValueError('Explicit peers/port required')
        for peer in net['allowed_peers']:text(peer,'peer')
        workspace=deepcopy(r['workspace']);workspace['model_urdf']=str(p.path(workspace['model_urdf']).resolve())  # absolute: from_settings must not re-join config_dir
        if hashlib.sha256(Path(workspace['model_urdf']).read_bytes()).hexdigest()!=workspace['model_sha256']:raise ValueError('Workspace model hash mismatch')
        for key in ('box_min_m','box_max_m'):vector(workspace[key],3,key,signed=True)
        if any(a>=b for a,b in zip(workspace['box_min_m'],workspace['box_max_m'])):raise ValueError('Invalid workspace box')
        number(workspace['table_z_m'],'table_z',minimum=-float('inf'));number(workspace['clearance_m'],'clearance')
        integer(workspace['interpolation_steps'],'interpolation_steps')
        if not isinstance(workspace['tool_points_m'],list) or not workspace['tool_points_m']:raise ValueError('Tool points required')
        for point in workspace['tool_points_m']:vector(point,3,'tool point',signed=True)
        self.workspace=from_settings(workspace,p.config_dir)
        if list(self.workspace.chain.joint_names)!=workspace['joint_names']:raise ValueError('Model joint ordering mismatch')
        self.reference=r;self.reference_sha256=p.sha256
        return dict(robot_id=r['robot']['robot_id'],allowed_contracts=tuple(r['contract']['allowed_sha256']),capability_prefix=r['contract']['capability_prefix'],
            lower=lo,upper=hi,margin=margin,max_velocity=vector(r['acceptance']['max_consecutive_row_speed_rad_s'],7,'row_speed',positive=True),
            max_target_lead=vector(r['acceptance']['max_target_lead_rad'],7,'lead',positive=True),
            measured_velocity=vector(r['guards']['max_measured_joint_velocity_rad_s'],7,'measured_speed',positive=True),
            state_timeout_ns=t['state_timeout_ns'],hand_state_timeout_ns=g['max_state_age_ns'],moving_epsilon_m_s=g['moving_epsilon_m_s'],
            open_width_m=g['open_width_m'],control_period_ns=1_000_000_000//t['control_hz'],hold_timeout_ns=t['hold_timeout_ms']*1_000_000,
            heartbeat_timeout_ns=t['heartbeat_timeout_ms']*1_000_000,command_timeout_ns=t['command_timeout_ms']*1_000_000,
            max_lease_ns=t['max_lease_s']*1_000_000_000,max_uncertainty_ns=r['clock']['max_uncertainty_us']*1000,min_probes=r['clock']['min_probes'],
            workspace=workspace,config_dir=p.config_dir,recorder_close_timeout_s=r['recording']['close_timeout_s'],
            **{k:tuple(v) if k=='allowed_peers' else v for k,v in net.items()})
