"""Offline torque oracle only; physical torques remain in native Franky."""
import math
import numpy as np

from .reference_types import finite_vector, positive
from .kinematics import pose_error_base, nullspace_projector


def dynamic_torque_projector(jacobian,mass,rcond=1e-6):
    """Inertia-aware secondary torque projection, for portable/hardware parity."""
    j,m = np.asarray(jacobian,dtype=float),np.asarray(mass,dtype=float)
    if j.shape!=(6,7) or m.shape!=(7,7) or not np.isfinite(j).all() or not np.isfinite(m).all():
        raise ValueError("Expected finite 6x7 Jacobian and 7x7 inertia")
    if not np.allclose(m,m.T) or not 0<rcond<1:
        raise ValueError("Expected symmetric inertia and valid pseudoinverse cutoff")
    np.linalg.cholesky(m)
    x=np.linalg.solve(m,j.T)
    return np.eye(7)-j.T@np.linalg.pinv(j@x,rcond=rcond)@x.T


def compute_posture_torque(jacobian,q,dq,target,stiffness,damping,rcond=1e-6):
    """Projected secondary PD torque before the common servo processing."""
    q,dq,target = (np.array(finite_vector(x,7,"posture")) for x in (q,dq,target))
    positive(stiffness,"stiffness",allow_zero=True)
    positive(damping,"damping",allow_zero=True)
    return nullspace_projector(jacobian,rcond)@(stiffness*(target-q)-damping*dq)


class CartesianImpedanceController:
    """Base-frame SO(3) log spring, twist damper and J-transpose torque.

    No operational-space inertia shaping or hidden posture controller. This is
    a simulation candidate until the installed Franky Cartesian audit arrives.
    """
    def __init__(self, stiffness, damping, **processing):
        self.k = np.array(finite_vector(stiffness, 6, "stiffness"))
        self.d = np.array(finite_vector(damping, 6, "damping"))
        if np.any(self.k < 0) or np.any(self.d < 0):
            raise ValueError("Gains must be nonnegative")
        self.processor = TorqueCommandProcessor(**processing)

    def reset(self):
        self.processor.reset()

    def compute(self, pose, twist, reference, jacobian, coriolis, gravity):
        pose = np.asarray(finite_vector(pose, 7, "pose"))
        twist = np.asarray(finite_vector(twist, 6, "twist"))
        reference = np.asarray(finite_vector(reference, 13, "reference"))
        j = np.asarray(jacobian)
        if j.shape != (6,7) or not np.isfinite(j).all():
            raise ValueError("Expected finite 6x7 base-frame geometric Jacobian")
        error = pose_error_base(pose[:3], pose[3:], reference[:3], reference[3:7])
        wrench = self.k*error+self.d*(reference[7:]-twist)
        pd = j.T@wrench
        return {"error": error, "wrench": wrench, "pd": pd,
                "coriolis": np.asarray(coriolis), **self.processor.compute(pd+coriolis, gravity)}


class TorqueCommandProcessor:
    """Slew non-gravity command, low-pass it, add gravity, then enforce effort bounds.

    Franky slew and libfranka low-pass both use the previous outgoing command.
    Only the filtered output persists; the limited candidate is telemetry.
    Gravity is supplied by PhysX in simulation (FCI compensates it on
    hardware). The final clamp is a simulator effort bound, not a claimed Franky
    implementation detail. Reset deliberately seeds non-gravity state at zero.
    """
    def __init__(self, *, dt=.001, slew_rate_nm_s=1000., cutoff_hz=100., effort_limits_nm=(87.,)*4+(12.,)*3):
        self.dt = positive(dt, "dt")
        self.step = positive(slew_rate_nm_s, "slew_rate_nm_s") * self.dt
        self.alpha = self.dt / (self.dt + 1 / (2 * math.pi * positive(cutoff_hz, "cutoff_hz")))
        self.effort = np.asarray(finite_vector(effort_limits_nm, 7, "effort_limits_nm"))
        if np.any(self.effort <= 0):
            raise ValueError("effort limits must be positive")
        self.reset()

    def reset(self, initial_command=None):
        """Zero deterministic reset, or seed recorded non-gravity tau_J_d."""
        self.filtered = (np.zeros(7) if initial_command is None else
                         np.asarray(finite_vector(initial_command, 7, "initial_command")).copy())
        self.limited = self.filtered.copy()

    def compute(self, requested, gravity):
        requested = np.asarray(finite_vector(requested, 7, "requested"))
        gravity = np.asarray(finite_vector(gravity, 7, "gravity"))
        self.limited = self.filtered + np.clip(requested - self.filtered, -self.step, self.step)
        self.filtered += self.alpha * (self.limited - self.filtered)
        unclipped = self.filtered + gravity
        applied = np.clip(unclipped, -self.effort, self.effort)
        return {"requested": requested.copy(), "limited": self.limited.copy(),
                "limited_candidate": self.limited.copy(), "filtered": self.filtered.copy(),
                "gravity": gravity.copy(), "unclipped": unclipped, "applied": applied}


class JointImpedanceController:
    def __init__(self, stiffness, damping, *, position_error_clip=.5, **processing):
        self.stiffness = np.asarray(finite_vector(stiffness, 7, "stiffness"))
        self.damping = np.asarray(finite_vector(damping, 7, "damping"))
        if np.any(self.stiffness < 0) or np.any(self.damping < 0):
            raise ValueError("gains must be nonnegative")
        self.error_clip = positive(position_error_clip, "position_error_clip")
        self.processor = TorqueCommandProcessor(**processing)

    def reset(self):
        self.processor.reset()

    def compute(self, q, dq, q_ref, dq_ref, coriolis, gravity):
        q, dq, q_ref, dq_ref, coriolis, gravity = (
            np.asarray(finite_vector(value, 7, name)) for name, value in
            zip(("q", "dq", "q_ref", "dq_ref", "coriolis", "gravity"), (q, dq, q_ref, dq_ref, coriolis, gravity)))
        pd = self.stiffness * np.clip(q_ref - q, -self.error_clip, self.error_clip) + self.damping * (dq_ref - dq)
        return {"pd": pd, "coriolis": coriolis, **self.processor.compute(pd + coriolis, gravity)}
