"""Simulator-independent serial URDF FK, geometric Jacobian and full-pose DLS.

This checks sampled kinematic feasibility, not mesh clearance or dynamics.
All geometric quantities are base-frame and quaternions are xyzw.
"""
import xml.etree.ElementTree as ET
import numpy as np
from .reference_types import finite_vector, unit_quaternion_xyzw, positive


def nullspace_projector(jacobian, rcond=1e-6):
    """Orthogonal kinematic torque projector; not dynamically consistent."""
    j = np.asarray(jacobian,dtype=float)
    if j.shape != (6,7) or not np.isfinite(j).all() or not 0 < rcond < 1:
        raise ValueError("Expected finite 6x7 Jacobian and 0<rcond<1")
    return np.eye(7)-j.T@np.linalg.pinv(j.T,rcond=rcond)


def quat_multiply(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3], b[:3]), a[3]*b[3]-a[:3]@b[:3]]


def rotation_vector(q):
    q = np.array(unit_quaternion_xyzw(q))
    if q[3] < 0:
        q = -q
    norm = np.linalg.norm(q[:3])
    return q[:3]*(2*np.arctan2(norm, q[3])/norm if norm > 1e-12 else 2.)


def quat_exp(vector):
    v = np.asarray(vector)
    angle = np.linalg.norm(v)
    return np.r_[v*(np.sin(angle/2)/angle if angle > 1e-12 else .5), np.cos(angle/2)]


def quaternion_from_matrix(r):
    # Symmetric eigenproblem remains well conditioned at rotations of pi.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])/3
    q = np.linalg.eigh(k)[1][:, -1]
    return q if q[3] >= 0 else -q


def pose_error_base(position, quaternion, desired_position, desired_quaternion):
    q = np.asarray(quaternion)
    return np.r_[np.asarray(desired_position)-position,
                 rotation_vector(quat_multiply(desired_quaternion, np.r_[-q[:3], q[3]]))]


def axis_rotation(axis, angle):
    axis = np.asarray(axis)/np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)


class UrdfKinematicsProvider:
    def __init__(self, path, base_link="link0", tip_link="link8"):
        joints = {j.find("child").get("link"): j for j in ET.parse(path).getroot().findall("joint")}
        chain, child = [], tip_link
        while child != base_link:
            j = joints[child]
            chain.append(j)
            child = j.find("parent").get("link")
        self.chain = []
        self.joint_names = []
        for j in reversed(chain):
            origin = j.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            r = axis_rotation((0,0,1), rpy[2])@axis_rotation((0,1,0), rpy[1])@axis_rotation((1,0,0), rpy[0])
            kind = j.get("type")
            if kind not in ("fixed", "revolute"):
                raise ValueError("Only fixed/revolute serial chains are supported")
            axis = np.fromstring(j.find("axis").get("xyz"), sep=" ") if kind == "revolute" else None
            self.chain.append((xyz, r, axis))
            if axis is not None:
                self.joint_names.append(j.get("name"))

    def evaluate(self, q):
        q = finite_vector(q, len(self.joint_names), "joint_position")
        r, p, origins, axes, i = np.eye(3), np.zeros(3), [], [], 0
        for xyz, rotation, axis in self.chain:
            p = p+r@xyz
            r = r@rotation
            if axis is not None:
                origins.append(p.copy())
                axes.append(r@axis/np.linalg.norm(axis))
                r = r@axis_rotation(axis, q[i])
                i += 1
        jacobian = np.vstack((np.array([np.cross(a,p-o) for a,o in zip(axes,origins)]).T, np.array(axes).T))
        return p, quaternion_from_matrix(r), jacobian


class DampedLeastSquaresIK:
    def __init__(self, provider, lower, upper, *, damping=.01, max_step_rad=.05,
                 position_tolerance_m=1e-4, rotation_tolerance_rad=1e-3, margin_rad=.02, max_iterations=100):
        self.provider = provider
        margin_rad = positive(margin_rad,"margin_rad",allow_zero=True)
        self.lower = np.asarray(finite_vector(lower,len(provider.joint_names),"lower"))+margin_rad
        self.upper = np.asarray(finite_vector(upper,len(provider.joint_names),"upper"))-margin_rad
        if np.any(self.lower >= self.upper) or damping <= 0 or max_step_rad <= 0:
            raise ValueError("Invalid DLS bounds/damping/step")
        self.damping, self.step = positive(damping,"damping"), positive(max_step_rad,"max_step_rad")
        self.p_tol = positive(position_tolerance_m,"position_tolerance_m")
        self.r_tol = positive(rotation_tolerance_rad,"rotation_tolerance_rad")
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError("max_iterations must be a positive integer")
        self.iterations = max_iterations

    def solve(self, position, quaternion, seed):
        position = finite_vector(position,3,"position")
        quaternion = unit_quaternion_xyzw(quaternion)
        q = np.array(finite_vector(seed,len(self.provider.joint_names),"seed"))
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("IK seed violates configured interior joint bounds")
        for _ in range(self.iterations):
            p, rotation, j = self.provider.evaluate(q)
            error = pose_error_base(p, rotation, position, quaternion)
            if np.linalg.norm(error[:3]) <= self.p_tol and np.linalg.norm(error[3:]) <= self.r_tol:
                return q
            delta = j.T@np.linalg.solve(j@j.T+self.damping**2*np.eye(6), error)
            delta *= min(1., self.step/max(np.max(np.abs(delta)), 1e-12))
            q = np.clip(q+delta, self.lower, self.upper)
        raise ValueError(f"Full-pose IK did not converge: position={np.linalg.norm(error[:3]):.6f} m, rotation={np.linalg.norm(error[3:]):.6f} rad")


def validate_path_ik(trajectory, solver, seed, *, sample_hz=20.):
    sample_hz = positive(sample_hz,"sample_hz")
    times = np.linspace(0, trajectory.duration_s, int(np.ceil(trajectory.duration_s*sample_hz))+1)
    solutions, errors, singular_values = [], [], []
    q = np.array(seed)
    for t in times:
        ref = trajectory.evaluate(float(t)).reference
        q = solver.solve(ref.position_m, ref.quaternion_xyzw, q)
        p, rotation, j = solver.provider.evaluate(q)
        solutions.append(q.copy())
        errors.append(pose_error_base(p, rotation, ref.position_m, ref.quaternion_xyzw))
        singular_values.append(np.linalg.svd(j, compute_uv=False)[-1])
    solutions, errors = np.array(solutions), np.array(errors)
    return {"passed": True, "method": "sampled full-pose DLS; previous solution seed; no mesh clearance certificate",
            "sample_count": len(times), "sample_hz_requested": sample_hz,
            "max_position_residual_m": float(np.linalg.norm(errors[:,:3], axis=1).max()),
            "max_rotation_residual_rad": float(np.linalg.norm(errors[:,3:], axis=1).max()),
            "minimum_interior_joint_margin_rad": float(np.minimum(solutions-solver.lower, solver.upper-solutions).min()),
            "minimum_jacobian_singular_value_mixed_units": float(min(singular_values))}, {"ik_time_s": times, "ik_q": solutions}
