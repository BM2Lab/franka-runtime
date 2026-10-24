"""Strict physical DROID schema v2. Inspection/validation never imports a vendor.

Separate from v1 mock configuration: no fictitious impedance acceleration/jerk
trajectory guarantees. Device connection requires additional explicit admission.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

FIELDS = {
 'robot': 'robot_id address model firmware_version fci_version arm_serial control_serial identity_max_age_ns collision_torque_nm collision_force_n',
 'contract': 'allowed_sha256 capability_prefix',
 'joint_bounds': 'lower_rad upper_rad margin_rad',
 'acceptance': 'max_target_lead_rad max_consecutive_row_speed_rad_s predicted_torque_fraction',
 'controller': 'kind joint_stiffness joint_damping cartesian_stiffness cartesian_damping error_clip_rad gain_time_constant_s compensate_coriolis max_delta_nm_per_callback libfranka_limit_rate lowpass_cutoff_hz',
 'guards': 'max_measured_joint_velocity_rad_s max_command_torque_nm max_callback_gap_ns publisher_timeout_ns',
 'stop': 'damping ramp_duration_s rest_velocity_rad_s max_duration_s compensate_coriolis max_delta_nm_per_callback',
 'timing': 'control_hz hold_timeout_ms heartbeat_timeout_ms command_timeout_ms max_lease_s startup_timeout_ns consumption_timeout_ns stop_timeout_ns state_timeout_ns native_log_capacity',
 'clock': 'max_uncertainty_us min_probes',
 'gripper': 'open_width_m close_width_m speed_m_s grasp_force_n epsilon_inner_m epsilon_outer_m poll_hz max_state_age_ns moving_epsilon_m_s cancel_interval_ms cancel_attempts history_capacity',
 'tool': 'F_T_EE m_ee I_ee F_x_Cee m_load I_load F_x_Cload scalar_tolerance transform_tolerance',
 'workspace': 'model_urdf model_sha256 base_link flange_link joint_names link_points tool_points_m body_points_m table_z_m clearance_m box_min_m box_max_m interpolation_steps',
 'network': 'bind_host port allowed_peers token_env',
 'artifacts': 'native_pin native_pin_sha256 collision_model collision_model_sha256',
 'recording': 'directory queue_capacity max_record_bytes max_event_bytes min_free_bytes fsync_interval_s close_timeout_s',
 'approval': 'approved_by approved_at physical_stop_procedure workspace_review hand_review collision_review',
}
FIELDS={key:tuple(value.split()) for key,value in FIELDS.items()}
# May be commissioned as an explicit empty list (e.g. no rig geometry beyond the Hand).
OPTIONAL_LISTS={('workspace','body_points_m')}
TOP={'schema_version','runtime','backend','status','executable','motion_authorized','provenance',*FIELDS}


def number(v, name, *, minimum=0., strict=False):
    if (isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v)
            or (v<=minimum if strict else v<minimum)):
        raise ValueError(f'{name}: expected finite number {">" if strict else ">="} {minimum}')
    return float(v)


def integer(v,name,minimum=1):
    if type(v) is not int or v<minimum:raise ValueError(f'{name}: expected integer >= {minimum}')
    return v


def vector(v,n,name,*,signed=False,positive=False):
    if not isinstance(v,list) or len(v)!=n:raise ValueError(f'{name}: expected {n}-vector')
    return tuple(number(x,name,minimum=-math.inf if signed else 0,strict=positive) for x in v)


def text(v,name):
    if not isinstance(v,str) or not v.strip():raise ValueError(f'{name}: nonempty text required')
    return v


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class PhysicalDroidConfig:
    def __init__(self,data,source='<memory>',config_dir=None):
        if not isinstance(data,dict) or set(data)!=TOP:
            raise ValueError('Physical config has missing/unknown top-level fields')
        if data['schema_version']!=2 or data['runtime']!='droid_jointpos_v1' or data['backend']!='franky':
            raise ValueError('Expected physical schema v2, droid_jointpos_v1, franky')
        for section,keys in FIELDS.items():
            if not isinstance(data[section],dict) or set(data[section])!=set(keys):
                raise ValueError(f'Missing/unknown fields in {section}')
        for flag in ('executable','motion_authorized'):
            if type(data[flag]) is not bool:raise ValueError(f'{flag}: explicit boolean required')
        self.data,self.source,self.config_dir=deepcopy(data),source,Path(config_dir) if config_dir else None

    @classmethod
    def load(cls,path):
        path=Path(path);return cls(json.loads(path.read_text()),str(path),path.parent)

    def missing(self):
        return [f'{s}.{k}' for s,keys in FIELDS.items() for k in keys
                if self.data[s][k] is None or self.data[s][k]=='' or (self.data[s][k]==[] and (s,k) not in OPTIONAL_LISTS)]

    @property
    def sha256(self):
        return hashlib.sha256(json.dumps(self.data,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

    def path(self,value):
        p=Path(value)
        if p.is_absolute():return p
        if self.config_dir is None:raise ValueError('Relative artifacts require a config directory')
        return self.config_dir/p

    def validated(self,*,check_files=True):
        d=self.data
        missing=[x for x in self.missing() if not x.startswith('approval.')]
        if missing:raise ValueError('Uncommissioned fields: '+', '.join(missing))
        r=d['robot'];[text(r[k],'robot.'+k) for k in ('robot_id','address','model','firmware_version','arm_serial','control_serial')]
        if r['fci_version']!=10:raise ValueError('This native candidate supports only server10; no compatibility bypass')
        integer(r['fci_version'],'fci_version');integer(r['identity_max_age_ns'],'identity_max_age_ns')
        for k in ('collision_torque_nm','collision_force_n'):number(r[k],k,strict=True)
        lower=vector(d['joint_bounds']['lower_rad'],7,'lower',signed=True)
        upper=vector(d['joint_bounds']['upper_rad'],7,'upper',signed=True)
        margin=vector(d['joint_bounds']['margin_rad'],7,'margin')
        if any(lo+m>=hi-m for lo,hi,m in zip(lower,upper,margin)):raise ValueError('No usable joint interval')
        for k in ('max_target_lead_rad','max_consecutive_row_speed_rad_s'):vector(d['acceptance'][k],7,k,positive=True)
        f=number(d['acceptance']['predicted_torque_fraction'],'predicted_torque_fraction',strict=True)
        if f>1:raise ValueError('predicted_torque_fraction must be in (0, 1]')
        c=d['controller']
        if c['kind']!='droid_hybrid_joint_impedance':raise ValueError('Unknown physical control law')
        for k,n in [('joint_stiffness',7),('joint_damping',7),('cartesian_stiffness',6),('cartesian_damping',6)]:vector(c[k],n,k)
        vector(c['error_clip_rad'],7,'error_clip_rad',positive=True)
        for k in ('gain_time_constant_s','max_delta_nm_per_callback','lowpass_cutoff_hz'):number(c[k],k,strict=True)
        if c['lowpass_cutoff_hz']>1000:raise ValueError('libfranka cutoff must be <=1000 Hz')
        for k in ('compensate_coriolis','libfranka_limit_rate'):
            if type(c[k]) is not bool:raise ValueError(f'{k}: explicit bool required')
        for k in ('max_measured_joint_velocity_rad_s','max_command_torque_nm'):vector(d['guards'][k],7,k,positive=True)
        for k in ('max_callback_gap_ns','publisher_timeout_ns'):integer(d['guards'][k],k)
        stop=d['stop'];vector(stop['damping'],7,'stop.damping',positive=True)
        for k in ('ramp_duration_s','rest_velocity_rad_s','max_duration_s','max_delta_nm_per_callback'):
            number(stop[k],k,strict=k!='ramp_duration_s')
        if type(stop['compensate_coriolis']) is not bool:raise ValueError('Explicit stop coriolis bool required')
        if stop['max_duration_s']<=stop['ramp_duration_s']+.1:raise ValueError('Stop must permit ramp and sustained rest')
        for section in ('timing','clock'):
            for k,v in d[section].items():integer(v,section+'.'+k)
        period=1_000_000_000//d['timing']['control_hz']
        if not 0<d['timing']['control_hz']<=1000:raise ValueError('Invalid supervisor frequency')
        for v in (d['guards']['publisher_timeout_ns'],d['timing']['consumption_timeout_ns'],d['timing']['state_timeout_ns']):
            if v<=period:raise ValueError('Watchdog/freshness must exceed supervisor period')
        if d['guards']['max_callback_gap_ns']<1_000_000:raise ValueError('Callback threshold below 1 kHz period')
        if d['timing']['startup_timeout_ns']<d['timing']['consumption_timeout_ns']:raise ValueError('Startup timeout shorter than consumption timeout')
        if d['timing']['stop_timeout_ns']<=stop['max_duration_s']*1e9:raise ValueError('Stop deadline must allow native completion/join')
        g=d['gripper']
        for k in ('open_width_m','speed_m_s','grasp_force_n','poll_hz','moving_epsilon_m_s','cancel_interval_ms'):number(g[k],k,strict=True)
        for k in ('close_width_m','epsilon_inner_m','epsilon_outer_m'):number(g[k],k)
        if not g['close_width_m']<g['open_width_m'] or g['poll_hz']<50:raise ValueError('Invalid hand widths or feedback target rate')
        for k in ('max_state_age_ns','cancel_attempts','history_capacity'):integer(g[k],k)
        if g['max_state_age_ns']<=1e9/g['poll_hz']:raise ValueError('Hand freshness must exceed its polling period')
        tool=d['tool']
        for k in ('m_ee','scalar_tolerance','transform_tolerance'):number(tool[k],k,strict=True)
        number(tool['m_load'],'m_load')
        for k,n in [('I_ee',9),('I_load',9),('F_x_Cee',3),('F_x_Cload',3)]:vector(tool[k],n,k,signed=True)
        t=tool['F_T_EE']
        if not isinstance(t,list) or len(t)!=4:raise ValueError('F_T_EE must be 4x4')
        for row in t:vector(row,4,'F_T_EE',signed=True)
        if any(abs(a-b)>1e-9 for a,b in zip(t[3],[0,0,0,1])):raise ValueError('Invalid homogeneous transform')
        for i in range(3):
            for j in range(3):
                if abs(sum(t[k][i]*t[k][j] for k in range(3))-(i==j))>1e-6:raise ValueError('Nonorthonormal tool rotation')
        det=sum(t[0][i]*(t[1][(i+1)%3]*t[2][(i+2)%3]-t[1][(i+2)%3]*t[2][(i+1)%3]) for i in range(3))
        if abs(det-1)>1e-6:raise ValueError('Improper tool rotation')
        w=d['workspace']
        for k in ('base_link','flange_link','model_urdf'):text(w[k],k)
        for k in ('joint_names','link_points','tool_points_m','body_points_m'):
            if not isinstance(w[k],list):raise ValueError(f'{k}: list required')
        for v in w['joint_names']+w['link_points']:text(v,'workspace link/joint')
        if not isinstance(w['joint_names'],list) or len(w['joint_names'])!=7 or len(set(w['joint_names']))!=7:raise ValueError('Seven distinct physical joint names required')
        for v in w['joint_names']+w['link_points']:text(v,'workspace link/joint')
        if not w['link_points'] or not w['tool_points_m']:raise ValueError('Workspace/link geometry required')
        for row in w['tool_points_m']:vector(row,3,'tool point',signed=True)
        for row in w['body_points_m']:vector(row,3,'body point',signed=True)
        number(w['table_z_m'],'table_z',minimum=-math.inf);number(w['clearance_m'],'clearance')
        lo=vector(w['box_min_m'],3,'box_min',signed=True);hi=vector(w['box_max_m'],3,'box_max',signed=True)
        if any(a>=b for a,b in zip(lo,hi)):raise ValueError('Invalid workspace box')
        integer(w['interpolation_steps'],'interpolation_steps')
        log=d['recording']
        text(log['directory'],'recording.directory')
        for k in ('queue_capacity','max_record_bytes','max_event_bytes','min_free_bytes'):integer(log[k],'recording.'+k)
        if log['max_record_bytes']>log['max_event_bytes']:raise ValueError('Record bound exceeds file bound')
        for k in ('fsync_interval_s','close_timeout_s'):number(log[k],'recording.'+k,strict=True)
        if log['close_timeout_s']>=60:raise ValueError('Recorder close timeout must be below 60s')
        n=d['network'];text(n['token_env'],'token_env')
        if n['bind_host'] in ('0.0.0.0','::','') or not isinstance(n['bind_host'],str):raise ValueError('Bind a specific dedicated interface')
        integer(n['port'],'port')
        if n['port']>=65536 or not isinstance(n['allowed_peers'],list) or not n['allowed_peers']:raise ValueError('Invalid network peers/port')
        for p in n['allowed_peers']:text(p,'peer')
        if not isinstance(d['contract']['allowed_sha256'],list) or not d['contract']['allowed_sha256']:
            raise ValueError('Nonempty contract hash list required')
        for h in d['contract']['allowed_sha256']+[w['model_sha256'],d['artifacts']['native_pin_sha256'],d['artifacts']['collision_model_sha256']]:
            if not isinstance(h,str) or len(h)!=64 or any(x not in '0123456789abcdef' for x in h):raise ValueError('Expected SHA256 hash')
        text(d['contract']['capability_prefix'],'capability_prefix')
        if check_files:
            for filename,h in [(w['model_urdf'],w['model_sha256']), (d['artifacts']['native_pin'],d['artifacts']['native_pin_sha256']), (d['artifacts']['collision_model'],d['artifacts']['collision_model_sha256'])]:
                if digest(self.path(filename))!=h:raise ValueError(f'Artifact mismatch: {filename}')
        return deepcopy(d)

    def require_approved(self):
        d=self.validated()
        if d['status']!='reviewed' or not d['executable'] or not d['motion_authorized']:
            raise ValueError('Physical configuration is not approved/executable')
        for k,v in d['approval'].items():text(v,'approval.'+k)
        return d

    def require_runnable(self):
        raise ValueError('Physical service entry point remains disabled pending qualification; use offline inspection')


def pending_template():
    return dict(schema_version=2,runtime='droid_jointpos_v1',backend='franky',status='pending_lab_review',
                executable=False,motion_authorized=False,provenance='No commissioning values supplied; v1 mock limits are not physical guarantees',
                **{s:{k:None for k in keys} for s,keys in FIELDS.items()})


def main():
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True);a=p.parse_args()
    c=PhysicalDroidConfig.load(a.config)
    print(json.dumps(dict(schema_version=2,physical_entry_enabled=False,config_sha256=c.sha256,missing=c.missing()),indent=2))


if __name__=='__main__':main()
