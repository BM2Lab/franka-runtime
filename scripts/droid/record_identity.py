"""Record unit identity evidence for RecordedIdentityReader (read-only FCI check).

Model, firmware and serials are operator-stated (Desk / unit labels) and come from a
reviewed unit observation file; this tool never infers them. It confirms the address
and FCI server version with a short read-only connection through the pinned
droid_readonly_state helper, then stamps the observation with this host's boot clock.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
from franka_runtime.droid.service import host_clock_id
from franka_runtime.droid.shadow_source import ReadOnlyStateSource


def record(*,unit_observation,model,firmware_version,helper,helper_sha256,output,read_only_fci=False):
    if read_only_fci is not True:raise ValueError('Explicit read-only FCI authorization required')
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    unit=json.loads(Path(unit_observation).read_text())
    source=ReadOnlyStateSource(helper,helper_sha256,unit['address'],hz=50,server_version=10,
                               stderr_path=output.with_suffix('.reader.log'))
    source.start()
    try:
        sample=source.wait_initial(5.)
    finally:
        source.stop()
    if sample['server_version']!=10:raise ValueError('Unexpected FCI server version')
    observed=time.monotonic_ns()
    value=dict(schema_version=1,address=unit['address'],model=model,firmware_version=firmware_version,
               fci_version=sample['server_version'],arm_serial=unit['arm_serial'],control_serial=unit['control_serial'],
               clock_id=host_clock_id(),observed_ns=observed,
               source=(f'Operator-stated model/firmware/serials (unit observation sha256 '
                       f'{hashlib.sha256(Path(unit_observation).read_bytes()).hexdigest()}); address and FCI server '
                       f'version confirmed by a read-only libfranka connection at observed_ns'))
    output.write_text(json.dumps(value,indent=2)+'\n')
    return dict(path=str(output),sha256=hashlib.sha256(output.read_bytes()).hexdigest())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--unit-observation',type=Path,required=True)
    p.add_argument('--model',required=True);p.add_argument('--firmware-version',required=True)
    p.add_argument('--helper',type=Path,required=True);p.add_argument('--helper-sha256',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--read-only-fci',action='store_true')
    print(json.dumps(record(**vars(p.parse_args())),indent=2))


if __name__=='__main__':main()
