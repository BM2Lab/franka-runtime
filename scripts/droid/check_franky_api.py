"""Read-only installed Franky audit: constructors/mailboxes only, never Robot/Gripper."""
import argparse
import hashlib
from importlib.metadata import version
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pin',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    import franky as f
    import numpy as np
    def forbidden(*args,**kwargs):
        raise AssertionError('Hardware construction forbidden during offline API audit')
    f.Robot=forbidden
    f.Gripper=forbidden
    pin=json.loads(a.pin.read_text());site=Path(f.__file__).resolve().parent.parent
    assert version('franky-control')==pin['franky_version']
    for path,expected in pin['files'].items():
        assert hashlib.sha256((site/path).read_bytes()).hexdigest()==expected,path
    k=[40,30,50,25,35,25,10];d=[4,6,5,5,3,2,1]
    kx=[400,400,400,15,15,15];dx=[37,37,37,2,2,2]
    m=f.JointImpedanceTrackingMotion(stiffness=k,damping=d,cartesian_stiffness=kx,cartesian_damping=dx)
    np.testing.assert_array_equal(m.params.stiffness,k)
    np.testing.assert_array_equal(m.params.damping,d)
    np.testing.assert_array_equal(m.get_cartesian_gains().stiffness,np.diag(kx))
    np.testing.assert_array_equal(m.get_cartesian_gains().damping,np.diag(dx))
    np.testing.assert_array_equal(m.params.error_clip,[.5]*7)
    assert m.get_reference() is None
    q=[0.,0.,0.,-1.,0.,1.,.123]
    ref=f.JointReference(q=q,dq=[0.]*7,tau_ff=[0.]*7)
    m.set_reference(ref)
    np.testing.assert_array_equal(m.get_reference().q,q)
    np.testing.assert_array_equal(m.get_reference().dq,[0.]*7)
    # params is a copy: changing it cannot configure the running controller.
    params=m.params;params.error_clip=[.2]*7
    np.testing.assert_array_equal(m.params.error_clip,[.5]*7)
    try:
        f.JointImpedanceTrackingMotion(cartesian_damping=dx)
    except ValueError:
        pass
    else:
        raise AssertionError('Damping without shaping stiffness must reject')
    result=dict(schema_version=1,franky_version=version('franky-control'),
                checked_native_files=len(pin['files']),wheel_sha256=pin['wheel_sha256'],
                shaping_constructor=True,explicit_damping=True,seven_joint_publish_readback=True,
                readback_is='last published reference, not native consumption',
                exposed_motion_methods=[n for n in dir(m) if not n.startswith('_')],
                error_clip_rad=m.params.error_clip.tolist(),params_mutation_changes_motion=False,
                hardware_accessed=False,native_rebuilt=False)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
