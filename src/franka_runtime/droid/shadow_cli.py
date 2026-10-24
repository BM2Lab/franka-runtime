"""Explicit D7 read-only FCI service. No SDK import or connection on import."""
import argparse
import os
from pathlib import Path
import signal
from contracts.droid import DroidIOSpec
from contracts.droid_session import capability_id
from contracts.sessions import payload_sha256
from .recording import SessionRecorder
from .shadow_config import ShadowRuntimeConfig
from .shadow_source import ReadOnlyStateSource
from .shadow import ShadowDroidBackend


def prepare_shadow(config,spec,*,clock,clock_id,token,source_factory=ReadOnlyStateSource):
    settings=config.require_runnable()
    if (not isinstance(token,str) or not token or payload_sha256(spec) not in settings['allowed_contracts']
            or spec.robot.robot_id!=settings['robot_id'] or spec.gripper.open_width_m!=settings['open_width_m']
            or list(spec.robot.joint_names)!=config.workspace.chain.joint_names
            or capability_id(spec).split(':')[0]!=settings['capability_prefix']):
        raise ValueError('Shadow contract/unit/joints/Hand/token mismatch')
    r=dict(config.reference['recording']);r.pop('directory');r.pop('close_timeout_s')
    recorder=SessionRecorder(config.path(config.data['recording_directory']),metadata=dict(mode='shadow_readonly',
        clock_id=clock_id,contract_sha256=payload_sha256(spec),candidate_sha256=config.reference_sha256,
        geometry_scope='point_workspace_only_no_self_collision',commands_enabled=False),**r)
    recorder.start();source=None
    try:
        s=config.data['source']
        source=source_factory(config.path(s['helper']),s['helper_sha256'],s['address'],hz=s['poll_hz'],server_version=s['server_version'],
                              stderr_path=recorder.directory/'reader.stderr.log',max_stream_lag_ns=settings['state_timeout_ns'])
        source.start();source.wait_initial(s['initial_timeout_s'])
        from .service import DroidRuntimeService
        service=DroidRuntimeService(config,spec,clock=clock,clock_id=clock_id,token=token,recorder=recorder,
            backend_factory=lambda executor,settings:ShadowDroidBackend(executor,source,clock=clock,settings=settings,recorder=recorder))
        return service
    except Exception as error:
        if source is not None:source.stop()
        recorder.close(dict(shadow_startup_failed=True,error=type(error).__name__,commands_sent=False),
                       timeout_s=config.reference['recording']['close_timeout_s'])
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--read-only-fci',action='store_true',help='Explicitly authorize read-only arm/Hand connection, never commands')
    a=p.parse_args()
    if not a.read_only_fci:p.error('--read-only-fci is required for the read-only device connection')
    import time
    from .service import host_clock_id
    config=ShadowRuntimeConfig.load(a.config);settings=config.require_runnable()
    service=prepare_shadow(config,DroidIOSpec.from_json(a.contract.read_text()),clock=time.monotonic_ns,
        clock_id=host_clock_id(),token=os.environ.get(settings['token_env']))
    try:
        port=service.bind()
        signal.signal(signal.SIGTERM,lambda *_:service.shutdown())
        print(f'shadow read-only DROID runtime on {settings["bind_host"]}:{port} clock {service.clock_id}; no device commands',flush=True)
        service.serve()
    except KeyboardInterrupt:service.shutdown()
    finally:service.finish_shutdown()


if __name__=='__main__':main()
