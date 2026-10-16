"""Offline fixture integrity and independent analytical invariants.

Future extracted controller/FK/reset implementations must consume these inputs
and compare their own outputs; these tests alone are not target-runtime parity.
"""
import json
import math
from pathlib import Path

FIXTURES=Path(__file__).parent/'fixtures'


def data():return json.loads((FIXTURES/'legacy-parity-v1.json').read_text())


def test_reference_projection_history_and_held_joint():
    d=data();r=d['reference']; a=r['action'];p=r['prior']; c=r['position_mapping']
    expected=p[:]
    for i,v in enumerate(a):expected[i]=min(1,max(-1,p[i]+c['action_mapping']['max_position_increment_rad'][i]*v))
    assert r['position_target']==expected
    assert r['position_target'][6]==p[6]
    assert r['velocity_target'][6]==0
    assert len(d['observation']['values'])==29
    assert d['observation']['values'][-6:]==a
    # Normalized previous reference; not the newly projected target.
    assert all(math.isclose(a,b,abs_tol=1e-12) for a,b in zip(d['observation']['values'][17:23],p[:6]))


def test_torque_order_and_independent_reset():
    d=data();alpha=.001/(.001+1/(2*math.pi*100)); previous=0
    for sample in d['torque_processing']:
        requested=sample['requested'][0]
        limited=previous+max(-1,min(1,requested-previous))
        filtered=previous+alpha*(limited-previous)
        out=sample['output']
        assert math.isclose(out['limited'][0],limited,abs_tol=1e-12)
        assert math.isclose(out['filtered'][0],filtered,abs_tol=1e-12)
        assert math.isclose(out['applied'][0],filtered+2,abs_tol=1e-12)
        previous=filtered
    r=d['selective_reset']
    assert math.isclose(r['reset_instance']['filtered'][0],alpha,abs_tol=1e-12)
    assert math.isclose(r['untouched_instance']['filtered'][0],2*alpha,abs_tol=1e-12)
    c=d['joint_controller'];assert all(math.isclose(v,3,abs_tol=1e-12) for v in c['output']['pd'])


def test_fk_frame_feedback_and_latched_fault():
    d=data();a=d['encoder_fk_adapter']
    assert a['model_call']['frame']=='Frame.Flange'
    assert a['reported_position_error_m']<1e-12
    assert a['model_call']['q']==d['urdf_fk']['q']
    assert d['fault_latch']['final_state']=='fault'
    assert d['fault_latch']['final_reason']=='callback_gap'
    assert len(d['urdf_fk']['jacobian'])==6
    assert len(d['urdf_fk']['joint_names'])==7


def test_fixture_geometric_invariants_and_projector_residuals():
    d=data()
    for f in d['curve_fixtures']:
        assert math.isclose(sum(v*v for v in f['tangent']),1,abs_tol=1e-12)
        assert abs(sum((p-q)*t for p,q,t in zip(f['position'],f['projected'],f['tangent'])))<1e-12
    p=d['projectors'];j=p['jacobian']
    for kind in ('kinematic','dynamic'):
        n=p[kind]
        for row in j:
            for col in range(7):
                # J N = 0 versus J M^-1 N_tau = 0.
                residual=sum(row[k]*n[k][col]/(p['mass'][k][k] if kind=='dynamic' else 1) for k in range(7))
                assert abs(residual)<1e-10
