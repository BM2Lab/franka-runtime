"""Populate an offline DROID draft from pinned DROID and archived lab pipelines.

Requires PyYAML only for reading saved configs. Never imports a robot SDK, loads
source code from the archives, or copies historical approvals into a new run.
"""
import argparse
import ast
import hashlib
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
from franka_runtime.droid.commissioning import prepare_review, digest
from franka_runtime.droid.workspace import SerialChain


HAND_FRESHNESS_NS=250_000_000  # measured-period basis below; at least two Hand periods of margin required


def populate(*,repo,m4_run,legacy_reference,droid_sources,mock_link,observed_state,output_dir,recording_dir,hand_timing=None,
             fresh_state=None,collision_bundle=None,unit_observation=None,payload=None,workspace_observation=None,
             operator_decisions=None):
    import yaml
    repo,m4_run,legacy_reference,droid_sources,mock_link,observed_state,output_dir=map(Path,
        (repo,m4_run,legacy_reference,droid_sources,mock_link,observed_state,output_dir))
    if output_dir.exists():raise FileExistsError(output_dir)
    sources={}
    def load(label,path,kind='json'):
        raw=path.read_bytes();sources[label]=dict(path=str(path.resolve()),sha256=hashlib.sha256(raw).hexdigest())
        return yaml.safe_load(raw) if kind=='yaml' else json.loads(raw)
    result=load('completed_report',m4_run/'report.json')
    if not result.get('hardware_executed') or result.get('terminal_state')!='complete' or result.get('smooth_stop_ok') is not True:
        raise ValueError('Expected a completed archived hardware run with confirmed stop')
    m4=load('completed_m4_config',m4_run/'config.yaml','yaml')
    if result['config_sha256']!=sources['completed_m4_config']['sha256']:
        raise ValueError('Archived hardware config/report hash mismatch')
    legacy=load('legacy_reference',legacy_reference,'yaml')
    ppo=load('ppo_candidate',repo/'configs/fr3/ppo.velocity-impedance.pending.yaml','yaml')
    mock=load('mock_link',mock_link);state=load('archived_state',observed_state)
    manifest=load('droid_source_manifest',droid_sources/'sources.json')
    for item in manifest:
        p=(droid_sources/item['file']).resolve();p.relative_to(droid_sources.resolve())
        if digest(p)!=item['sha256']:raise ValueError('Pinned DROID source changed: '+item['file'])
    droid=load('droid_fr3',droid_sources/'droid/config/fr3/franka_hardware.yaml','yaml')
    sources['droid_robot']=dict(path=str((droid_sources/'droid/droid/franka/robot.py').resolve()),
                                sha256=digest(droid_sources/'droid/droid/franka/robot.py'))
    tree_robot=ast.parse((droid_sources/'droid/droid/franka/robot.py').read_text())
    update=next(n for n in ast.walk(tree_robot) if isinstance(n,ast.FunctionDef) and n.name=='update_gripper')
    calls=[n for n in ast.walk(update) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='goto']
    if len(calls)!=1:raise ValueError('DROID gripper call changed; audit required')
    wrapper={kw.arg:ast.literal_eval(kw.value) for kw in calls[0].keywords if kw.arg in ('speed','force')}
    if set(wrapper)!={'speed','force'}:raise ValueError('DROID gripper constants changed')
    d6=repo/'docs/droid-d6-rt-host.md';sources['archived_d6_summary']=dict(path=str(d6.resolve()),sha256=digest(d6))
    urdf=repo/'tests/parity/fixtures/robot_returned_model.urdf'
    sources['arm_reference_urdf']=dict(path=str(urdf.resolve()),sha256=digest(urdf))
    contract=repo/'configs/fr3/droid_fr3_jointpos_v1.contract.json'
    spec=json.loads(contract.read_text());logical=spec['robot']['joint_names']
    raw_chain=SerialChain(urdf,'link0','link8');original=list(raw_chain.joint_names)
    tree=ET.parse(urdf);renames=dict(zip(original,logical))
    for joint in tree.getroot().findall('joint'):
        if joint.get('name') in renames:joint.set('name',renames[joint.get('name')])
    aliased=ET.tostring(tree.getroot(),encoding='utf-8',xml_declaration=True)+b'\n'
    entries=[];ledger={}
    decided={}
    if operator_decisions is not None:
        decided=load('operator_decisions',Path(operator_decisions))
        if (decided.get('schema_version')!=1 or not decided.get('basis') or not isinstance(decided.get('fields'),dict)
                or not decided['fields'] or any(f.startswith('approval.') for f in decided['fields'])):
            raise ValueError('Operator decisions need schema 1, basis and non-approval fields')
    def put(field,value,source,basis):
        if field in decided.get('fields',{}) and source!='operator_decisions':
            return  # replaced by the operator decision below
        s=sources[source]
        entries.append(dict(field=field,value=value,source=s['path'],sha256=s['sha256'],basis=basis))
        ledger[field]=dict(value=value,source=source,basis=basis)
    def proposal(field,value,basis):
        sources['population_script']=dict(path=str(Path(__file__).resolve()),sha256=digest(__file__))
        put(field,value,'population_script','ENGINEERING PROPOSAL, not hardware evidence: '+basis)
    if unit_observation is not None:
        unit=load('unit_observation',Path(unit_observation))
        if unit.get('schema_version')!=1 or not isinstance(unit.get('address'),str) or not unit.get('basis'):
            raise ValueError('Unit observation needs schema 1, address and basis')
        put('robot.address',unit['address'],'unit_observation',unit['basis'])
        for key in ('arm_serial','control_serial','identity_max_age_ns'):  # operator-stated label values / policy
            if key in unit:
                if not unit.get('basis_'+key):raise ValueError('Unit observation needs basis_'+key)
                put('robot.'+key,unit[key],'unit_observation',unit['basis_'+key])
    else:
        put('robot.address',droid['robot_client']['executable_cfg']['robot_ip'],'droid_fr3','DROID reference address; confirm against the current unit before connection')
    put('robot.model',ET.parse(urdf).getroot().get('name'),'arm_reference_urdf','Archived FR3 model identity; not a fresh physical-unit identification')
    put('robot.firmware_version','5.9.2','archived_d6_summary','Archived October 23 read-only summary; must be reconfirmed with unit serials')
    put('robot.fci_version',10,'archived_d6_summary','Selected server10 family; not a new FCI query')
    for field,key in [('collision_torque_nm','collision_torque_nm'),('collision_force_n','collision_force_n')]:
        put('robot.'+field,m4['robot'][key],'completed_m4_config','Historical bare-flange constructor thresholds, not DROID 40/40 or new Hand approval')
    for key,old in [('lower_rad','joint_lower_rad'),('upper_rad','joint_upper_rad'),('margin_rad','position_margin_rad')]:
        put('joint_bounds.'+key,legacy['safety'][old],'legacy_reference','Archived model range/margin; effective values checked against completed M4 run')
    for lo,hi,margin,e_lo,e_hi in zip(legacy['safety']['joint_lower_rad'],legacy['safety']['joint_upper_rad'],
        legacy['safety']['position_margin_rad'],m4['limits']['joint_lower'],m4['limits']['joint_upper']):
        if abs(lo+margin-e_lo)>1e-8 or abs(hi-margin-e_hi)>1e-8:raise ValueError('Legacy margin differs from completed bounds')
    put('acceptance.max_target_lead_rad',mock['limits']['max_target_lead_rad'],'mock_link','D5 mock setpoint-lead baseline; physical acceptance proposal')
    proposal('acceptance.predicted_torque_fraction',0.9,'Reject chunks whose predicted raw impedance torque step exceeds 90% of the command-torque guard')
    put('acceptance.max_consecutive_row_speed_rad_s',droid['robot_client']['executable_cfg']['limits']['joint_vel'],'droid_fr3','DROID reference velocity ceilings adopted for row-to-row target admission after cross-host replay; not a measured-speed guard or pi0.5 action-contract constant')
    meta=droid['robot_client']['metadata_cfg'];exec_cfg=droid['robot_client']['executable_cfg'];imp=legacy['robot']['impedance_controller']
    for field,key in [('joint_stiffness','default_Kq'),('joint_damping','default_Kqd'),('cartesian_stiffness','default_Kx'),('cartesian_damping','default_Kxd')]:
        put('controller.'+field,meta[key],'droid_fr3','Pinned DROID FR3 default hybrid feedback; candidate, not lab qualification')
    for field,key in [('error_clip_rad','expected_error_clip_rad'),('gain_time_constant_s','gains_time_constant_s'),
                      ('compensate_coriolis','compensate_coriolis'),('max_delta_nm_per_callback','max_delta_tau_nm_per_ms')]:
        put('controller.'+field,imp[key],'legacy_reference','Existing Franky processing; slew is per callback, not elapsed-time scaling')
    put('controller.libfranka_limit_rate',exec_cfg['limit_rate'],'droid_fr3','Pinned DROID filter/rate configuration')
    put('controller.lowpass_cutoff_hz',exec_cfg['lpf_cutoff_frequency'],'droid_fr3','Pinned DROID 100 Hz filter; preserves Franky slew before libfranka filtering')
    for field,key in [('max_measured_joint_velocity_rad_s','joint_velocity'),('max_command_torque_nm','command_torque')]:
        put('guards.'+field,m4['limits'][key],'completed_m4_config','Historical M4 guard, not DROID clamp semantics and not Hand qualification')
    for field,key in [('max_callback_gap_ns','callback_gap_s'),('publisher_timeout_ns','reference_timeout_s')]:
        put('guards.'+field,round(m4['limits'][key]*1e9),'completed_m4_config','Historical native time guard; remains provisional for this runtime')
    for field,key in [('damping','damping'),('ramp_duration_s','ramp_duration'),('rest_velocity_rad_s','velocity_epsilon'),
                      ('max_duration_s','max_duration'),('compensate_coriolis','compensate_coriolis'),('max_delta_nm_per_callback','max_delta_tau')]:
        put('stop.'+field,m4['stop'][key],'completed_m4_config','Completed archived smooth stop; bare-flange history does not qualify the mounted Hand')
    for field in ('control_hz','hold_timeout_ms','heartbeat_timeout_ms','command_timeout_ms','max_lease_s'):
        put('timing.'+field,mock['timing'][field],'mock_link','Existing cross-host mock baseline; physical margins await timing measurements; control_hz is supervisor, servo remains 1000 Hz')
    proposal('timing.startup_timeout_ns',5_000_000_000,'5 s admission allowance, inspired by DROID policy-start wait; not a measured native startup bound')
    proposal('timing.consumption_timeout_ns',50_000_000,'50 ms first-consumption allowance; choose after timing tests')
    proposal('timing.stop_timeout_ns',int((m4['stop']['max_duration']+.5)*1e9),'native stop maximum plus 0.5 s for telemetry/worker join')
    put('timing.state_timeout_ns',round(ppo['timing']['state_timeout_ms']*1e6),'ppo_candidate','PPO migration candidate state timeout; pending requalification for DROID')
    put('timing.native_log_capacity',m4['log_capacity'],'completed_m4_config','Existing native buffer capacity')
    for field,value in mock['clock'].items():put('clock.'+field,value,'mock_link','Existing mock clock baseline; not measured physical margin')
    put('gripper.close_width_m',0.,'droid_robot','DROID closure=1 endpoint mapped to zero width; grasp outcome still needs Hand qualification')
    put('gripper.speed_m_s',wrapper['speed'],'droid_robot','Pinned DROID update_gripper speed=0.05 m/s')
    put('gripper.grasp_force_n',mock['gripper']['grasp_force_n'],'mock_link','20 N provisional direct-libfranka force; not blindly copying Polymetis goto(force=0.1)')
    if fresh_state is not None:
        st=load('fresh_state',Path(fresh_state))
        if st.get('robot_mode')!='Idle' or st.get('current_errors'):
            raise ValueError('Fresh tool state must be an Idle, error-free read-only capture')
        c=st['F_T_EE'];mount=[[c[col*4+row] for col in range(4)] for row in range(4)]  # libfranka column-major
        basis=('Read-only echo_robot_state with the Franka Hand configured in Desk; Desk nominal Hand '
               'parameters as reported by the controller, not a per-unit calibration')
        put('tool.F_T_EE',mount,'fresh_state',basis)
        load_keys=() if payload is not None else ('m_load','I_load','F_x_Cload')
        for key in ('m_ee','I_ee','F_x_Cee',*load_keys):put('tool.'+key,st[key],'fresh_state',basis)
    if payload is not None:
        load_value=load('payload',Path(payload))
        if (load_value.get('schema_version')!=1 or not load_value.get('basis') or not 0<load_value['m_load']<3
                or len(load_value['F_x_Cload'])!=3 or len(load_value['I_load'])!=9):
            raise ValueError('Payload needs schema 1, basis, 0 < m_load < 3 kg, F_x_Cload[3], I_load[9]')
        for key in ('m_load','F_x_Cload','I_load'):
            put('tool.'+key,load_value[key],'payload',f"Operator-measured payload {load_value['name']} set by the runtime at connect: {load_value['basis']}")
    collision_manifest=None
    if collision_bundle is not None:
        collision_bundle=Path(collision_bundle)
        collision_manifest=load('collision_manifest',collision_bundle/'manifest.json')
        if collision_manifest['workspace_urdf_sha256']!=hashlib.sha256(aliased).hexdigest():
            raise ValueError('Collision bundle is bound to a different workspace URDF')
        if fresh_state is not None and collision_manifest['hand_mount_F_T_EE']!=mount:
            raise ValueError('Collision bundle Hand mount differs from the fresh tool state')
        put('artifacts.collision_model','assets/collision/manifest.json','collision_manifest',
            'Bundled franka_description FR3 v2.1 + Franka Hand mesh collision model (manifest-pinned files)')
        put('artifacts.collision_model_sha256',sources['collision_manifest']['sha256'],'collision_manifest',
            'SHA-256 of the collision manifest, which pins every model file')
    if hand_timing is not None:
        timing=load('hand_timing',Path(hand_timing))
        period_ms=timing['hand']['max_ms']
        if timing.get('read_only') is not True or not 0<period_ms or 2*period_ms*1e6>=HAND_FRESHNESS_NS:
            raise ValueError('Hand timing evidence must be read-only and leave two measured periods of margin')
        put('gripper.max_state_age_ns',HAND_FRESHNESS_NS,'hand_timing',
            f"Measured read-only Hand state period {timing['hand']['median_ms']} ms median / {period_ms} ms max "
            f"({timing['hand']['samples']} samples, {timing['captured']}); 250 ms is about 2.5 periods")
    for field,value,basis in [('epsilon_inner_m',.005,'Existing libfranka/Franky grasp default'),('epsilon_outer_m',.005,'Existing libfranka/Franky grasp default'),
        ('poll_hz',50,'Contract feedback target, not achieved-rate evidence'),
        *([] if hand_timing is not None else [('max_state_age_ns',100_000_000,'Runner 100 ms freshness baseline')]),
        ('moving_epsilon_m_s',.001,'Width-change detection threshold'),('cancel_interval_ms',10,'Existing fake-device cancellation test cadence'),
        ('cancel_attempts',20,'Existing fake-device bounded cancellation attempts'),('history_capacity',1024,'Existing serialized Hand worker default')]:
        proposal('gripper.'+field,value,basis)
    proposal('tool.scalar_tolerance',1e-6,'Configured-value comparison tolerance; not physical calibration accuracy')
    proposal('tool.transform_tolerance',1e-6,'Configured matrix comparison tolerance; not physical calibration accuracy')
    # An archived arm-only URDF can be a reference, not the missing Hand collision model.
    proposal('workspace.model_urdf','assets/arm-contract-names.reference.urdf','Copied arm-only URDF with joint labels aliased in order; no Hand geometry added')
    proposal('workspace.model_sha256',hashlib.sha256(aliased).hexdigest(),'Hash of the exact derived reference asset')
    rig=None if payload is None else load_value.get('bounding_box')
    if rig and fresh_state is None:raise ValueError('Rig body points need the fresh state Hand mount')
    observed=json.loads(Path(workspace_observation).read_text()) if workspace_observation is not None else {}
    table=('table_z_m','clearance_m') if 'table_z_m' in observed else ()
    for field in ('base_link','flange_link','link_points',*(() if fresh_state else ('tool_points_m',)),
                  *(f for f in ('table_z_m','clearance_m') if f not in table),'interpolation_steps'):
        put('workspace.'+field,mock['workspace'][field],'mock_link','Nominal archived arm/Hand point-check setup; not a surveyed workspace or collision model')
    if fresh_state is not None:
        R,t=[row[:3] for row in mount[:3]],[mount[i][3] for i in range(3)]
        tip=lambda y:[round(sum(R[i][j]*v for j,v in enumerate((0.,y,0.)))+t[i],6) for i in range(3)]
        put('workspace.tool_points_m',[tip(0.),tip(.04),tip(-.04)],'fresh_state',
            'Flange-frame TCP and open-Hand fingertips at +/-0.04 m along the TCP y axis of the recorded Hand mount')
    if rig:
        put('workspace.body_points_m',rig['corners_flange_m'],'payload',
            f"8 corners of the operator-measured rig bounding box of payload {load_value['name']}; table plane only")
    else:
        proposal('workspace.body_points_m',[],'No rig geometry beyond the Hand declared')
    put('workspace.joint_names',logical,'mock_link','Logical contract aliases for the seven reference-model joints; explicit ordered rename recorded')
    if workspace_observation is not None:
        box=load('workspace_observation',Path(workspace_observation))
        lo,hi=box.get('box_min_m'),box.get('box_max_m')
        if (box.get('schema_version')!=1 or not box.get('basis') or not isinstance(lo,list) or not isinstance(hi,list)
                or len(lo)!=3 or len(hi)!=3 or any(a>=b for a,b in zip(lo,hi))):
            raise ValueError('Workspace observation needs schema 1, basis and box_min_m < box_max_m (3 each)')
        for field,value in (('box_min_m',lo),('box_max_m',hi)):put('workspace.'+field,value,'workspace_observation',box['basis'])
        if 'table_z_m' in box:
            if not (isinstance(box.get('clearance_m'),(int,float)) and 0<box['clearance_m']<=.1 and lo[2]<=box['table_z_m']+box['clearance_m']):
                raise ValueError('Workspace observation table needs 0 < clearance_m <= 0.1 m')
            for field in ('table_z_m','clearance_m'):put('workspace.'+field,box[field],'workspace_observation',box['basis'])
    else:
        for field,key in [('box_min_m','workspace_lower'),('box_max_m','workspace_upper')]:
            put('workspace.'+field,m4['limits'][key],'completed_m4_config','Historical circle-task box; may reject DROID task targets and must be reviewed for the new scene')
    for field,value in mock['network'].items():put('network.'+field,value,'mock_link','Recorded direct-link test configuration; no network changes performed')
    proposal('recording.directory',str(Path(recording_dir).resolve()),'New per-run recording directory; parent must exist and directory must not be reused')
    for field,value,basis in [('queue_capacity',4096,'Match historical native logging capacity; writer must keep up'),('max_record_bytes',65536,'64 KiB per-event bound'),
        ('max_event_bytes',1073741824,'1 GiB event budget; session faults on exhaustion'),('min_free_bytes',1073741824,'Additional 1 GiB free-space reserve'),
        ('fsync_interval_s',1.,'1 s target durability interval'),('close_timeout_s',5.,'5 s bounded writer close')]:proposal('recording.'+field,value,basis)
    for field,value in decided.get('fields',{}).items():
        put(field,value,'operator_decisions',f"Operator decision: {decided['basis']}")
    # Generate evidence in a staging file; preserve any pre-existing review directory.
    output_dir.parent.mkdir(parents=True,exist_ok=True)
    import tempfile
    with tempfile.TemporaryDirectory(dir=output_dir.parent) as temp:
        retained=Path(temp)/'source-evidence';retained.mkdir()
        source_map={}
        for label,info in sources.items():
            original=Path(info['path']);name=label+original.suffix
            shutil.copyfile(original,retained/name)
            if digest(retained/name)!=info['sha256']:raise ValueError('Source changed during snapshot: '+label)
            source_map[info['path']]='source-evidence/'+name
            info['snapshot']=source_map[info['path']]
        for entry in entries:entry['source']=source_map[entry['source']]
        evidence=Path(temp)/'evidence.json';evidence.write_text(json.dumps(dict(schema_version=1,entries=entries),indent=2)+'\n')
        report=prepare_review(contract_path=contract,native_pin_path=repo/'src/franka_runtime/droid/droid-server10.candidate.json',
            audit_fixture_path=repo/'tests/fixtures/droid/hybrid-feedback.json',output_dir=output_dir,evidence_path=evidence)
        (output_dir/'field-evidence.json').write_bytes(evidence.read_bytes())
        shutil.copytree(retained,output_dir/'source-evidence')
        receipt_path=output_dir/'provenance.json';meta=json.loads(receipt_path.read_text())
        for row in meta['field_provenance'].values():
            if isinstance(row['source'],dict) and row['source'].get('path','').startswith(str(retained)):
                row['source']['path']=str(output_dir/Path(row['source']['path']).relative_to(temp))
        receipt_path.write_text(json.dumps(meta,indent=2)+'\n')
    (output_dir/'assets').mkdir();(output_dir/'assets/arm-contract-names.reference.urdf').write_bytes(aliased)
    if collision_bundle is not None:
        shutil.copytree(collision_bundle,output_dir/'assets/collision')
        if digest(output_dir/'assets/collision/manifest.json')!=sources['collision_manifest']['sha256']:
            raise ValueError('Collision manifest changed while copying')
    alias=SerialChain(output_dir/'assets/arm-contract-names.reference.urdf','link0','link8')
    for q in ([0.]*7,state['q']):
        if raw_chain.frames(q)!=alias.frames(q):raise ValueError('Joint alias changed FK geometry')
    observations=dict(archived_state=sources['archived_state'],reported_tool={k:state[k] for k in
        ('F_T_EE','m_ee','I_ee','F_x_Cee','m_load','I_load','F_x_Cload')},
        note='Archived Desk state has zero EE mass/identity flange tool transform despite documented mounted Hand. Not promoted to usable tool parameters.',
        joint_aliases=renames,geometry_note='Arm reference only; transformed joint names do not add Hand geometry or self-collision qualification.',
        droid_alternatives=dict(joint_limits=exec_cfg['limits'],collision_behavior=exec_cfg['collision_behavior'],
            gripper_wrapper_force_argument=wrapper['force']),
        fresh_tool=None if fresh_state is None else dict(source=sources['fresh_state'],
            note='Promoted to tool.* from a read-only capture after the Hand was configured in Desk; Desk nominal values, not a per-unit calibration'),
        remaining=report['missing'],sources=sources)
    (output_dir/'observations-and-alternatives.json').write_text(json.dumps(observations,indent=2)+'\n')
    # Fix the evidence receipt to the retained copy rather than the removed staging path.
    p=output_dir/'provenance.json';receipt=json.loads(p.read_text())
    receipt['sources']['field_evidence']=dict(path=str((output_dir/'field-evidence.json').resolve()),sha256=digest(output_dir/'field-evidence.json'))
    receipt['population_sources']=sources;receipt['joint_aliases']=renames
    p.write_text(json.dumps(receipt,indent=2)+'\n')
    review=output_dir/'REVIEW.md'
    review.write_text(review.read_text().replace('Upstream gains are in `controller.reference.json` and were not applied to the draft.',
        'DROID gains have been populated as unapproved candidate values from the pinned implementation; see per-field provenance and controller.reference.json.'))
    return dict(prefilled=len(receipt['field_provenance']),remaining=len(report['missing']),missing=report['missing'],output_dir=str(output_dir))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('repo','m4-run','legacy-reference','droid-sources','mock-link','observed-state','output-dir','recording-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--hand-timing',type=Path,help='read-only Hand timing summary (sets gripper.max_state_age_ns from measurement)')
    p.add_argument('--fresh-state',type=Path,help='read-only robot state JSON with the Hand configured (tool fields)')
    p.add_argument('--collision-bundle',type=Path,help='directory with manifest.json from build_collision_manifest.py')
    p.add_argument('--unit-observation',type=Path,help='operator/observation evidence JSON with the robot address')
    p.add_argument('--payload',type=Path,help='per-rig payload file from build_payload.py (tool.m_load/F_x_Cload/I_load)')
    p.add_argument('--workspace-observation',type=Path,help='operator-measured task box in the base frame (workspace.box_*)')
    p.add_argument('--operator-decisions',type=Path,help='operator-decided field values (schema 1: fields, basis); override other sources')
    args=p.parse_args();print(json.dumps(populate(**vars(args)),indent=2))


if __name__=='__main__':main()
