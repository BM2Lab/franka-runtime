"""Pinocchio/Coal self-collision engine for the bound FR3 + Franka Hand model.

Optional ``collision`` extra (``pin==4.0.0``, the Pinocchio/Coal versions bundled in the
control stack's Franky build); pinocchio is imported only when an engine is built.

The model is the franka_description FR3 v2.1 + Franka Hand mesh collision geometry with
Franka's SRDF exclusions, recorded in a hashed manifest. Loading refuses any changed
file, joint-name or tool mismatch, unpinned engine version, or kinematics that differ
from the workspace URDF used by the point checks. A configuration is accepted only if no
remaining link pair is closer than the manifest's ``min_clearance_m``.
"""
import hashlib
import json
import math
from pathlib import Path
import random

from contracts.sessions import payload_sha256
from .workspace import SerialChain

FK_SAMPLES = 64
FK_TOLERANCE_M = 1e-9


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class SelfCollisionEngine:
    def __init__(self, manifest_path, *, workspace_urdf, joint_names, tool):
        manifest_path = Path(manifest_path)
        raw = manifest_path.read_bytes()
        self.model_sha256 = hashlib.sha256(raw).hexdigest()
        m = json.loads(raw)
        if m.get("schema_version") != 1 or m.get("kind") != "franka_description_mesh_self_collision_v1":
            raise ValueError("Unsupported collision manifest")
        root = manifest_path.parent
        for name, digest in m["files"].items():
            path = (root/name).resolve()
            path.relative_to(root.resolve())
            if _sha256(path) != digest:
                raise ValueError(f"Collision model file changed: {name}")
        if _sha256(workspace_urdf) != m["workspace_urdf_sha256"]:
            raise ValueError("Collision model is bound to a different workspace URDF")
        if list(joint_names) != m["contract_joint_names"]:
            raise ValueError("Collision model joint names differ from the contract")
        expected, tol = m["hand_mount_F_T_EE"], tool["transform_tolerance"]
        if any(abs(a-b) > tol for row_a, row_b in zip(tool["F_T_EE"], expected) for a, b in zip(row_a, row_b)):
            raise ValueError("Configured tool F_T_EE differs from the collision model's Hand mount")
        self.urdf_sha256 = m["workspace_urdf_sha256"]
        self.joint_names = tuple(joint_names)
        self.tool_sha256 = payload_sha256(tool)
        self.min_clearance_m = float(m["min_clearance_m"])
        if not 0 <= self.min_clearance_m < 0.1:
            raise ValueError("min_clearance_m out of range")
        self.provenance = m["provenance"]

        import pinocchio as pin
        if pin.__version__ != m["engine"]["pinocchio"]:
            raise ValueError(f"Pinocchio {pin.__version__} is not the pinned {m['engine']['pinocchio']}")
        self._pin = pin
        urdf, srdf = str(root/m["urdf"]), str(root/m["srdf"])
        self.model = pin.buildModelFromUrdf(urdf)
        self.geometry = pin.buildGeomFromUrdf(self.model, urdf, pin.GeometryType.COLLISION, package_dirs=[str(root)])
        self.geometry.addAllCollisionPairs()
        pin.removeCollisionPairs(self.model, self.geometry, srdf)
        if len(self.geometry.collisionPairs) != m["collision_pairs"]:
            raise ValueError("Collision pair count differs from the manifest")
        self.data, self.geometry_data = self.model.createData(), pin.GeometryData(self.geometry)
        for request in self.geometry_data.collisionRequests:
            request.security_margin = self.min_clearance_m
            request.enable_contact = False
        self._arm = [self.model.joints[self.model.getJointId(n)].idx_q for n in m["arm_joints"]]
        self._fingers = [self.model.joints[self.model.getJointId(n)].idx_q for n in m["finger_joints"]]
        self._finger_position = float(m["finger_position_m"])
        self._flange = self.model.getFrameId(m["flange_frame"])
        self._verify_kinematics(workspace_urdf)

    def _configuration(self, q):
        x = self._pin.neutral(self.model)
        for index, value in zip(self._arm, q):
            x[index] = value
        for index in self._fingers:
            x[index] = self._finger_position  # fingers fully open: the largest Hand envelope
        return x

    def _verify_kinematics(self, workspace_urdf):
        chain = SerialChain(workspace_urdf, "link0", "link8")
        rng = random.Random(0)
        for _ in range(FK_SAMPLES):
            q = tuple(rng.uniform(-2.5, 2.5) for _ in range(7))
            self._pin.framesForwardKinematics(self.model, self.data, self._configuration(q))
            a = self.data.oMf[self._flange].translation
            b = chain.frames(q)["link8"]
            if max(abs(a[i]-b[i][3]) for i in range(3)) > FK_TOLERANCE_M:
                raise ValueError("Collision model kinematics differ from the workspace URDF")

    def check_configuration(self, q):
        """True only if every non-excluded pair keeps at least ``min_clearance_m``."""
        q = tuple(q)
        if len(q) != 7 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in q):
            raise ValueError("Seven finite joint positions required")
        return not self._pin.computeCollisions(self.model, self.data, self.geometry, self.geometry_data,
                                               self._configuration(q), True)

    def closest_pair(self, q):
        """Diagnostic: (distance_m, first, second) for the closest non-excluded pair."""
        self._pin.computeDistances(self.model, self.data, self.geometry, self.geometry_data, self._configuration(q))
        distances = [r.min_distance for r in self.geometry_data.distanceResults]
        i = min(range(len(distances)), key=distances.__getitem__)
        pair = self.geometry.collisionPairs[i]
        return (distances[i], self.geometry.geometryObjects[pair.first].name,
                self.geometry.geometryObjects[pair.second].name)


def engine_from_physical_config(config):
    """Engine bound to a validated physical config's collision artifact, workspace URDF and tool."""
    d = config.validated() if hasattr(config, "validated") else config.data
    w = d["workspace"]
    return SelfCollisionEngine(config.path(d["artifacts"]["collision_model"]),
                               workspace_urdf=config.path(w["model_urdf"]),
                               joint_names=w["joint_names"], tool=d["tool"])
