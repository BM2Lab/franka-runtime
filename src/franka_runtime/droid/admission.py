"""Explicit physical DROID composition. Default CLI is offline inspection only.

No provider discovery, vendor import, device setup or threads on import. Device
setup/motion authorization are explicit API inputs, separate from reviewed config.
The mock config never authorizes a physical backend.
"""
from copy import deepcopy
from dataclasses import dataclass
import json
from pathlib import Path
import time
from contracts.droid_session import capability_id
from contracts.sessions import payload_sha256
from .physical_config import PhysicalDroidConfig
from .physical_factory import PhysicalDeviceFactory
from .recording import from_physical_config
from .workspace import from_settings


def service_settings(config):
    d=config.require_approved();b=d['joint_bounds'];t=d['timing'];g=d['gripper']
    if t['stop_timeout_ns']/1e9+2>=60:raise ValueError('Stop budget exceeds supervised shutdown deadline')
    return dict(robot_id=d['robot']['robot_id'],allowed_contracts=tuple(d['contract']['allowed_sha256']),
        capability_prefix=d['contract']['capability_prefix'],lower=tuple(b['lower_rad']),upper=tuple(b['upper_rad']),
        margin=tuple(b['margin_rad']),max_velocity=tuple(d['acceptance']['max_consecutive_row_speed_rad_s']),
        max_target_lead=tuple(d['acceptance']['max_target_lead_rad']),control_period_ns=1_000_000_000//t['control_hz'],
        hold_timeout_ns=t['hold_timeout_ms']*1_000_000,heartbeat_timeout_ns=t['heartbeat_timeout_ms']*1_000_000,
        command_timeout_ns=t['command_timeout_ms']*1_000_000,max_lease_ns=t['max_lease_s']*1_000_000_000,
        max_uncertainty_ns=d['clock']['max_uncertainty_us']*1000,min_probes=d['clock']['min_probes'],
        open_width_m=g['open_width_m'],grasp_force_n=g['grasp_force_n'],gripper_speed_m_s=g['speed_m_s'],
        workspace=d['workspace'],config_dir=config.config_dir,recorder_close_timeout_s=d['recording']['close_timeout_s'],
        **{k:(tuple(v) if k=='allowed_peers' else v) for k,v in d['network'].items()})


class _AdmittedPhysicalConfig:
    """Internal composition marker; ordinary physical config/CLI stays disabled."""
    def __init__(self,config):
        self.config=PhysicalDroidConfig(deepcopy(config.data),config.source,config.config_dir)
        self.data=self.config.data
    def require_runnable(self):return service_settings(self.config)


@dataclass
class PreparedPhysicalRuntime:
    service: object
    configuration_sha256: str

    def close(self,**kwargs):return self.service.finish_shutdown(**kwargs)


class PhysicalAdmission:
    def __init__(self,config,spec,device_factory,*,clock=time.monotonic_ns,clock_id):
        if not isinstance(config,PhysicalDroidConfig) or not isinstance(device_factory,PhysicalDeviceFactory):
            raise ValueError('Explicit physical schema and device factory required')
        self.config=PhysicalDroidConfig(config.data,config.source,config.config_dir)
        self.spec,self.factory,self.clock,self.clock_id=spec,device_factory,clock,clock_id

    def inspect(self):
        """File/contract/geometry admission; never calls a device/identity reader."""
        d=self.config.require_approved();settings=service_settings(self.config)
        if (payload_sha256(self.spec) not in settings['allowed_contracts']
                or self.spec.robot.robot_id!=settings['robot_id']
                or list(self.spec.robot.joint_names)!=d['workspace']['joint_names']
                or self.spec.gripper.open_width_m!=settings['open_width_m']
                or capability_id(self.spec).split(':')[0]!=settings['capability_prefix']):
            raise ValueError('Physical contract/unit/joints/hand/capability mismatch')
        factory=self.factory
        if (factory.config.sha256!=self.config.sha256 or factory.clock is not self.clock
                or factory.clock_id!=self.clock_id or not self.clock_id):
            raise ValueError('Factory config or host clock binding mismatch')
        if (not callable(factory.identity_reader) or factory.collision_checker is None
                or not callable(getattr(factory.collision_checker,'check_configuration',None))):
            raise ValueError('Identity and collision providers required')
        if factory.collision_checker.artifact_sha256!=d['artifacts']['collision_model_sha256']:
            raise ValueError('Collision provider/model binding mismatch')
        workspace=from_settings(d['workspace'],self.config.config_dir)
        if list(workspace.chain.joint_names)!=d['workspace']['joint_names']:
            raise ValueError('URDF joint ordering mismatch')
        return dict(configuration_sha256=self.config.sha256,contract_sha256=payload_sha256(self.spec),
                    clock_id=self.clock_id,hardware_accessed=False)

    def prepare(self,*,authorize_device_setup=False,authorize_motion=False,token=None):
        """Compose one session, without binding a socket or starting control.

        Called only after separate operator authorization. Tests inject a factory
        subclass with fake devices. This method is never called by inspection CLI.
        """
        if authorize_device_setup is not True or authorize_motion is not True:
            raise ValueError('Explicit device setup and motion authorization required')
        report=self.inspect()
        import os
        settings=service_settings(self.config)
        session_token=os.environ.get(settings['token_env']) if token is None else token
        if not isinstance(session_token,str) or not session_token:
            raise ValueError('Session token required before recording/device setup')
        recorder=from_physical_config(self.config,clock_id=self.clock_id,contract_sha256=report['contract_sha256'])
        recorder.start()  # Reserve recording before any device constructor.
        try:
            if not recorder.emit('admission',report,timestamp_ns=self.clock()):
                raise RuntimeError('Admission recording unavailable')
            from .service import DroidRuntimeService
            config=_AdmittedPhysicalConfig(self.config)
            def backend_factory(executor,settings):
                if (self.factory.config.sha256!=report['configuration_sha256']
                        or self.config.sha256!=report['configuration_sha256']):
                    raise ValueError('Factory configuration changed during admission')
                backend=self.factory.connect(executor,spec=self.spec,authorize_device_setup=True,recorder=recorder)
                if backend.configuration_sha256!=report['configuration_sha256']:
                    raise ValueError('Connected backend configuration mismatch')
                backend.recorder=recorder
                return backend
            service=DroidRuntimeService(config,self.spec,clock=self.clock,clock_id=self.clock_id,
                                        token=session_token,backend_factory=backend_factory,recorder=recorder)
            return PreparedPhysicalRuntime(service,self.config.sha256)
        except Exception as error:
            recorder.close(dict(admission_failed=True,control_started=False,error=type(error).__name__),
                           timeout_s=self.config.data['recording']['close_timeout_s'])
            raise


def main():
    import argparse
    from contracts.droid import DroidIOSpec
    p=argparse.ArgumentParser(description='Inspect commissioning gaps; no physical execution option')
    p.add_argument('--config',type=Path,required=True);p.add_argument('--contract',type=Path,required=True)
    a=p.parse_args();c=PhysicalDroidConfig.load(a.config);spec=DroidIOSpec.from_json(a.contract.read_text())
    missing=c.missing()
    result=dict(physical_cli_enabled=False,hardware_accessed=False,configuration_sha256=c.sha256,
                contract_sha256=payload_sha256(spec),missing=missing)
    if not missing:
        settings=service_settings(c)
        if (payload_sha256(spec) not in settings['allowed_contracts']
                or spec.robot.robot_id!=settings['robot_id']
                or list(spec.robot.joint_names)!=c.data['workspace']['joint_names']
                or spec.gripper.open_width_m!=settings['open_width_m']
                or capability_id(spec).split(':')[0]!=settings['capability_prefix']):
            raise ValueError('Contract differs from physical commissioning')
        result['providers_required']=['fresh unit identity','vetted self-collision model/engine']
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
