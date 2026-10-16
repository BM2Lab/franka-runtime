"""Extracted hardware motion helpers; provenance: deployment/migration/phase2-extraction.json."""
from __future__ import annotations
import numpy as np
from .reference_types import finite_vector, positive

class CurveVirtualFixture:
    """Analytic infinite Z line or closed YZ circle, expressed in robot base."""
    def __init__(self,kind,origin,radius=.05):
        if kind not in ("z_line","yz_circle"):
            raise ValueError("Expected z_line or yz_circle")
        self.kind = kind
        self.origin = np.array(finite_vector(origin,3,"curve origin"))
        self.radius = positive(radius,"radius")

    def project(self,position):
        p = np.array(finite_vector(position,3,"position"))
        if self.kind == "z_line":
            return np.r_[self.origin[:2],p[2]],np.array([0.,0.,1.])
        radial = p[1:]-self.origin[1:]
        distance = np.linalg.norm(radial)
        if distance <= 1e-6:
            raise ValueError("YZ circle projection undefined near its center")
        u = radial/distance
        return self.origin+np.r_[0.,self.radius*u],np.r_[0.,-u[1],u[0]]

    def force(self,position,velocity,stiffness=100.,normal_damping=20.,tangent_damping=20.):
        for value in (stiffness,normal_damping,tangent_damping):
            positive(value,"fixture gain")
        p = np.array(finite_vector(position,3,"position"))
        v = np.array(finite_vector(velocity,3,"velocity"))
        target,tangent = self.project(p)
        normal = np.eye(3)-np.outer(tangent,tangent)
        return -stiffness*(normal@(p-target))-normal_damping*(normal@v)-tangent_damping*tangent*(tangent@v)
