"""Portable algebraic fixtures; no vendor import, controller thread or hardware."""
import json
from pathlib import Path
import pytest

np=pytest.importorskip('numpy')
FIXTURE=Path(__file__).parents[1]/'fixtures/droid/hybrid-feedback.json'
DATA=json.loads(FIXTURE.read_text())


@pytest.mark.parametrize('case',DATA['cases'],ids=lambda c:c['name'])
def test_pinned_feedback_expression_fixture(case):
    q,dq,qr,J=(np.asarray(case[k]) for k in ('q','dq','qref','J'))
    kq,dqgain,kx,dx=(np.diag(case[k]) for k in ('Kq','Dq','Kx','Dx'))
    K=kq+J.T@kx@J;D=dqgain+J.T@dx@J
    np.testing.assert_allclose(K@(qr-q)-D@dq,case['droid_feedback_nm'],rtol=1e-12,atol=1e-12)
    np.testing.assert_allclose(K@np.clip(qr-q,-.5,.5)-D@dq,case['franky_clipped_feedback_nm'],rtol=1e-12,atol=1e-12)


def test_default_clipping_is_an_explicit_nonparity_case():
    case=next(c for c in DATA['cases'] if c['name']=='outside-clip')
    assert np.max(np.abs(np.array(case['droid_feedback_nm'])-case['franky_clipped_feedback_nm']))>1


def test_j7_is_controlled_without_a_held_joint_mapping():
    case=next(c for c in DATA['cases'] if c['name']=='j7-only')
    assert case['qref'][6]!=case['q'][6]
    assert abs(case['droid_feedback_nm'][6])>0
