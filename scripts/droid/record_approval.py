"""Record operator approval of a fully populated DROID review candidate.

Approval is operator input, never field evidence: it is written only by this explicit
step, bound to the hash of the exact candidate the operator reviewed. It marks the
candidate ``reviewed`` but leaves ``executable`` and ``motion_authorized`` false; enabling
motion is a separate, per-stage authorization.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from franka_runtime.droid.physical_config import FIELDS, PhysicalDroidConfig, digest, text

CANDIDATE='physical.review.pending.json'


def record(*,candidate_dir,approval,expected_sha256):
    candidate_dir,approval=Path(candidate_dir),Path(approval)
    path=candidate_dir/CANDIDATE;c=PhysicalDroidConfig.load(path);d=c.data
    if c.sha256!=expected_sha256:raise ValueError('Candidate differs from the reviewed hash')
    if d['status']!='pending_lab_review' or d['executable'] or d['motion_authorized'] or any(d['approval'].values()):
        raise ValueError('Candidate is not an unapproved review draft')
    c.validated()  # every non-approval field present, artifacts match their hashes
    a=json.loads(approval.read_text())
    if set(a)!={'schema_version','statement',*FIELDS['approval']} or a['schema_version']!=1:
        raise ValueError('Approval needs schema 1, statement and every approval field')
    for key in ('statement',*FIELDS['approval']):text(a[key],'approval.'+key)
    d['approval']={key:a[key] for key in FIELDS['approval']}
    d.update(status='reviewed',provenance=f'Operator-reviewed candidate {expected_sha256}; see approval.json. '
             'Not executable and not motion-authorized; motion needs separate per-stage authorization.')
    approved=PhysicalDroidConfig(d,str(path),candidate_dir);approved.validated()
    shutil.copyfile(approval,candidate_dir/'approval.json')
    path.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
    receipt=dict(schema_version=1,reviewed_candidate_sha256=expected_sha256,approved_candidate_sha256=approved.sha256,
                 approval_sha256=digest(candidate_dir/'approval.json'),executable=False,motion_authorized=False,
                 hardware_accessed=False)
    (candidate_dir/'approval-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate-dir',type=Path,required=True)
    p.add_argument('--approval',type=Path,required=True)
    p.add_argument('--expected-sha256',required=True,help='PhysicalDroidConfig.sha256 of the candidate the operator reviewed')
    print(json.dumps(record(**vars(p.parse_args())),indent=2))


if __name__=='__main__':main()
