"""Explicit physical DROID service for an authorized commissioning stage.

Needs a stage run config from scripts/droid/authorize_stage.py (reviewed, approved,
executable and motion-authorized), a hash-bound recorded identity and both explicit
flags. Starting this connects to the arm and Hand (constructor collision-threshold
writes); arm motion starts only when a client starts a session.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time
from contracts.droid import DroidIOSpec
from .admission import PhysicalAdmission, service_settings
from .collision import engine_from_physical_config
from .physical_config import PhysicalDroidConfig
from .physical_factory import PhysicalDeviceFactory
from .providers import BoundCollisionChecker, RecordedIdentityReader


def prepare_physical(config,spec,*,identity,identity_sha256,clock,clock_id,token,
                     authorize_device_setup=False,authorize_motion=False,factory_class=PhysicalDeviceFactory,control_cpu=None):
    if authorize_device_setup is not True or authorize_motion is not True:
        raise ValueError('Explicit device setup and motion authorization required')
    stage=config.path('stage-authorization.json')
    if not stage.is_file() or json.loads(stage.read_text()).get('run_config_sha256')!=config.sha256:
        raise ValueError('Stage authorization for this exact run config required')
    checker=BoundCollisionChecker(config,engine_from_physical_config(config))
    factory=factory_class(config,clock=clock,clock_id=clock_id,identity_reader=RecordedIdentityReader(identity,sha256=identity_sha256),
                          collision_checker=checker,control_cpu=control_cpu)
    admission=PhysicalAdmission(config,spec,factory,clock=clock,clock_id=clock_id)
    return admission.prepare(authorize_device_setup=True,authorize_motion=True,token=token)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--identity',type=Path,required=True);p.add_argument('--identity-sha256',required=True)
    p.add_argument('--authorize-device-setup',action='store_true',help='connect arm/Hand; sets collision thresholds')
    p.add_argument('--authorize-motion',action='store_true',help='allow a client session to start native control')
    p.add_argument('--control-cpu',type=int,help='dedicated CPU for the 1 kHz control thread; all other threads avoid it')
    a=p.parse_args()
    if not (a.authorize_device_setup and a.authorize_motion):
        p.error('--authorize-device-setup and --authorize-motion are both required')
    if a.control_cpu is not None:
        cpus=os.sched_getaffinity(0)
        if a.control_cpu not in cpus or len(cpus)<2:p.error('--control-cpu must be one of several available CPUs')
        os.sched_setaffinity(0,cpus-{a.control_cpu})  # inherited by every thread created from here on
    from .service import host_clock_id
    config=PhysicalDroidConfig.load(a.config);settings=service_settings(config)
    prepared=prepare_physical(config,DroidIOSpec.from_json(a.contract.read_text()),identity=a.identity,
        identity_sha256=a.identity_sha256,clock=time.monotonic_ns,clock_id=host_clock_id(),
        token=os.environ.get(settings['token_env']),authorize_device_setup=True,authorize_motion=True,control_cpu=a.control_cpu)
    service=prepared.service
    try:
        port=service.bind()
        signal.signal(signal.SIGTERM,lambda *_:service.shutdown())
        print(f'physical DROID runtime {prepared.configuration_sha256[:12]} on {settings["bind_host"]}:{port} '
              f'clock {service.clock_id}; arm/Hand connected, control starts on session start',flush=True)
        service.serve()
    except KeyboardInterrupt:service.shutdown()
    finally:print(json.dumps(prepared.close(),default=str),flush=True)


if __name__=='__main__':main()
