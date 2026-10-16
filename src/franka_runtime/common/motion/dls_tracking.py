"""Portable resolved-rate DLS reference law for native-controller parity tests."""
import numpy as np


def resolved_rate(jacobian, error, desired_twist, posture_error, velocity_limit, *,
                  pose_gain, task_scale, lambda_value=.01, rank_tolerance=1e-6, posture_gain=.5):
    j=np.asarray(jacobian,dtype=float);e=np.asarray(error,dtype=float);v=np.asarray(desired_twist,dtype=float)
    p=np.asarray(posture_error,dtype=float);bound=np.asarray(velocity_limit,dtype=float)
    g=np.asarray(pose_gain,dtype=float);scale=np.asarray(task_scale,dtype=float)
    if j.shape!=(6,7) or any(x.shape!=(6,) for x in (e,v,g,scale)) or any(x.shape!=(7,) for x in (p,bound)):
        raise ValueError('DLS dimensions')
    if not all(np.isfinite(x).all() for x in (j,e,v,p,bound,g,scale)) or np.any(bound<=0) or np.any(g<=0) or np.any(scale<=0):
        raise ValueError('DLS nonfinite input or nonpositive gains/limits')
    if not np.isfinite([lambda_value,rank_tolerance,posture_gain]).all() or lambda_value<=0 or not 0<rank_tolerance<1 or posture_gain<0:
        raise ValueError('DLS solver parameters')
    scaled=scale[:,None]*j;u,s,vh=np.linalg.svd(scaled,full_matrices=True)
    primary=vh[:6].T@((s/(s*s+lambda_value**2))*(u.T@(scale*(v+g*e))))
    active=s>rank_tolerance*s[0];basis=vh[:6][active].T
    secondary=(np.eye(7)-basis@basis.T)@(posture_gain*p)
    velocity=primary+secondary;factor=min(1.,1./max(1.,np.max(abs(velocity)/bound)))
    return velocity*factor,secondary*factor,factor,int(active.sum())
