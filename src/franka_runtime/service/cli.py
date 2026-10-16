"""Installed offline M4 commissioning tools. No FCI or hardware execution option."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import shutil
import sys
from contracts.motion import CartesianReference
from contracts.sessions import LeaseRequest, LeaseRelease, CommandEnvelope, payload_sha256


def _output(path):
    if path is None:return None
    path=Path(path).absolute()
    if not path.parent.is_dir():raise ValueError('Explicit output parent must already exist')
    if shutil.disk_usage(path.parent).free<16*1024*1024:
        raise ValueError('Insufficient artifact storage')
    path.mkdir(exist_ok=False)
    return path


def _write(path,value):
    with path.open('x') as stream:json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False);stream.write('\n')


class _Clock:
    def __init__(self):self.ns=1_000_000_000
    def __call__(self):return self.ns


def _prepare(args):
    from ..common.backends.franky_m4 import load_hardware_config
    from ..common.artifacts.bundle import verify_motion_bundle
    c=load_hardware_config(args.config)
    if args.bundle is not None:c['bundle']=str(args.bundle.resolve(strict=True))
    else:c['bundle']=str((args.config.resolve().parent/c['bundle']).resolve(strict=True))
    verify_motion_bundle(c['bundle'],c['bundle_manifest_sha256'])
    return c


def _mock(args,c,output):
    from ..common.backends.offline import MockMotionBackend, OfflineShadowBackend
    from .m4 import create_m4_session
    state=json.loads(args.state.read_text())
    backend=(OfflineShadowBackend if args.shadow else MockMotionBackend)(c,state)
    snapshot=backend.preflight();path=backend.plan(snapshot)
    if args.command=='preflight':
        result={'hardware_executed':False,'mode':'offline_preflight','start':asdict(snapshot),
                'planned_duration_s':path.duration_s if path else c['task']['duration_s'],
                'qualification':'NOT QUALIFIED; mock/archived model only'}
        if output:_write(output/'preflight.json',result)
        return result
    if output is None:raise ValueError('Mock run requires an explicit output location')
    clock=_Clock();clock_id='offline-mock:simulation'
    authority=create_m4_session(backend,robot_id='offline-fr3',clock_id=clock_id,clock=clock,expect_application=not args.shadow)
    duration=path.duration_s if path else c['task']['duration_s']
    lease=authority.acquire(LeaseRequest('local-run','offline-fr3','local-commissioning',clock_id,clock(),int((duration+2)*1e9)),episode_id='mock-episode')
    authority.start(lease)
    count=math.ceil(duration*c['reference_hz']);period_ns=round(1e9/c['reference_hz'])
    submitted=applied=0
    with (output/'references.jsonl').open('x') as refs,(output/'receipts.jsonl').open('x') as receipts:
        try:
            for i in range(count):
                now=clock();elapsed=i/c['reference_hz']
                if path:
                    ref=path.evaluate(elapsed).reference
                    p,q,twist=ref.position_m,ref.quaternion_xyzw,ref.linear_velocity_m_s+ref.angular_velocity_rad_s
                    posture=None
                else:
                    anchor=getattr(backend,'posture_plan',{})
                    p=anchor.get('fixed_position_m',snapshot.flange_position_m)
                    q=anchor.get('fixed_orientation_xyzw',snapshot.flange_orientation_xyzw)
                    twist=(0.,)*6;posture=backend.posture_target(elapsed,snapshot)
                payload=CartesianReference('arm','fr3_link0','fr3_flange',p,q,twist).to_dict()
                if posture is not None:payload['posture_rad']=list(posture)
                envelope=CommandEnvelope(lease,i+1,i,now,now,now+int(c['limits']['reference_timeout_s']*1e9),authority.capability_id,payload_sha256(payload))
                receipt=authority.submit(envelope,payload)
                if receipt.status!='accepted':raise RuntimeError(receipt.reason)
                submitted+=1;authority.dispatch();clock.ns=now+1_000_000
                evidence=backend.consume(clock())
                if evidence is not None:
                    ack=authority.acknowledge(lease_id=lease.lease_id,clock_id=clock_id,**evidence)
                    if ack is not None and ack.status=='applied':applied+=1
                refs.write(json.dumps({'sequence':i+1,'elapsed_s':elapsed,'reference':payload,'feedback_source':'static_mock_urdf_fk','hardware_executed':False},allow_nan=False)+'\n')
                for r in authority.drain_receipts():receipts.write(r.to_json()+'\n')
                if authority.fault:raise RuntimeError(authority.fault.code)
                clock.ns=now+period_ns
            authority.release(LeaseRelease(lease,clock(),'offline_complete'))
        except Exception:
            authority.poll(native_fault='offline_runner_failure')
            raise
        finally:
            ok=authority.finish_stop(backend.stop_evidence())
            for r in authority.drain_receipts():receipts.write(r.to_json()+'\n')
            result={'hardware_executed':False,'mode':'offline_shadow' if args.shadow else 'mock',
                    'terminal_state':authority.state,'fault':asdict(authority.fault) if authority.fault else None,
                    'submitted':submitted,'applied_receipts':applied,'stop_ok':ok,
                    'receipt_source':backend.receipt_source,'dropped_receipts':authority.dropped_receipts,
                    'qualification':'Mock lifecycle only; no physical dynamics or native timing validation'}
            _write(output/'report.json',result)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    for name in ('preflight','mock','replay'):
        p=sub.add_parser(name)
        p.add_argument('--config',type=Path,required=True)
        p.add_argument('--bundle',type=Path)
        p.add_argument('--output',type=Path,required=name!='preflight')
        if name=='replay':p.add_argument('--input',type=Path,required=True)
        else:
            p.add_argument('--state',type=Path,required=True)
            p.add_argument('--shadow',action='store_true',help='offline shadow, never live FCI')
    args=parser.parse_args(argv)
    output=None
    try:
        c=_prepare(args);output=_output(args.output)
        if output:
            _write(output/'effective-config.json',c)
            _write(output/'provenance.json',{'profile_sha256':payload_sha256(c),'hardware_executed':False,'bundle_manifest_sha256':c['bundle_manifest_sha256']})
        if args.command=='replay':
            from .replay import replay_servo
            result=replay_servo(args.input,Path(c['bundle'])/'model/robot_returned_model.urdf')
            _write(output/'replay.json',result)
            code=0 if result['numerical_checks_passed'] else 1
        else:
            result=_mock(args,c,output)
            code=0 if args.command=='preflight' or (result['terminal_state']=='complete' and result['stop_ok']) else 1
        print(json.dumps(result,indent=2,sort_keys=True,allow_nan=False))
        return code
    except (ValueError,OSError,RuntimeError,KeyError) as error:
        if output is not None:
            try:_write(output/'fault.json',{'hardware_executed':False,'terminal_state':'fault','reason':str(error)})
            except OSError:pass
        print(f'Offline M4 fault: {error}',file=sys.stderr);return 1


if __name__=='__main__':raise SystemExit(main())
