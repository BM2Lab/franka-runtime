"""Standard-library forward kinematics and point-based workspace checks for DROID rows.

An approximation, not collision checking: selected joint-frame origins and tool points
(in the flange frame, e.g. Franka Hand fingertips) must stay above a table plane by a
clearance, and tool points inside an axis-aligned workspace box, all in the robot base
frame. Configurations are checked at each row and at interpolated points between rows
(including measured q → row 0). Self-collision needs a vetted collision model and is
not covered here.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _origin(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0., 0., 0., 1.]]


def _axis_rotation(axis, angle):
    x, y, z = axis
    n = math.sqrt(x*x+y*y+z*z)
    x, y, z = x/n, y/n, z/n
    c, s, t = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y, 0.], [t*x*y+s*z, t*y*y+c, t*y*z-s*x, 0.],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c, 0.], [0., 0., 0., 1.]]


class SerialChain:
    """Revolute/fixed chain from ``base_link`` to ``tip_link`` parsed from a URDF."""

    def __init__(self, urdf_path, base_link="link0", tip_link="link8"):
        joints = {j.find("child").get("link"): j for j in ET.parse(urdf_path).getroot().findall("joint")}
        chain, child = [], tip_link
        while child != base_link:
            if child not in joints:
                raise ValueError(f"{tip_link} is not connected to {base_link}")
            chain.append(joints[child])
            child = joints[child].find("parent").get("link")
        self.steps, self.joint_names = [], []
        for joint in reversed(chain):
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            kind = joint.get("type")
            if kind not in ("fixed", "revolute"):
                raise ValueError("Only fixed/revolute serial chains are supported")
            axis = [float(v) for v in joint.find("axis").get("xyz").split()] if kind == "revolute" else None
            self.steps.append((joint.find("child").get("link"), _origin(xyz, rpy), axis))
            if axis is not None:
                self.joint_names.append(joint.get("name"))
        if len(self.joint_names) != 7:
            raise ValueError("Expected a seven-joint arm chain")

    def frames(self, q):
        """Base-frame transform of every child link, after its joint rotation."""
        transform, result, i = [[float(i == j) for j in range(4)] for i in range(4)], {}, 0
        for link, origin, axis in self.steps:
            transform = _matmul(transform, origin)
            if axis is not None:
                transform = _matmul(transform, _axis_rotation(axis, q[i]))
                i += 1
            result[link] = transform
        return result


def _apply(transform, point):
    return tuple(sum(transform[r][c]*v for c, v in enumerate((*point, 1.)))for r in range(3))


class WorkspaceCheck:
    def __init__(self, chain, *, link_points, tool_points_m, flange_link, table_z_m, clearance_m, box_min_m,
                 box_max_m, interpolation_steps):
        self.chain, self.links, self.tool = chain, tuple(link_points), tuple(tuple(p) for p in tool_points_m)
        self.flange, self.table_z, self.clearance = flange_link, table_z_m, clearance_m
        self.box_min, self.box_max, self.steps = tuple(box_min_m), tuple(box_max_m), interpolation_steps

    def points(self, q):
        frames = self.chain.frames(q)
        links = {name: tuple(frames[name][r][3] for r in range(3)) for name in self.links}
        tool = [_apply(frames[self.flange], p) for p in self.tool]
        return links, tool

    def check_configuration(self, q, label):
        links, tool = self.points(q)
        floor = self.table_z+self.clearance
        for name, point in [*links.items(), *((f"tool point {i}", p) for i, p in enumerate(tool))]:
            if point[2] < floor:
                raise ValueError(f"{label}: {name} at z={point[2]:.3f} m below table + clearance {floor:.3f} m")
        for i, point in enumerate(tool):
            if not all(lo <= v <= hi for v, lo, hi in zip(point, self.box_min, self.box_max)):
                raise ValueError(f"{label}: tool point {i} at {tuple(round(v, 3) for v in point)} outside workspace box")

    def check_rows(self, rows, measured_q):
        previous = tuple(measured_q)
        for index, row in enumerate(rows):
            q = tuple(row[:7])
            for k in range(1, self.steps+1):
                mid = tuple(a+(b-a)*k/self.steps for a, b in zip(previous, q))
                self.check_configuration(mid, f"row {index}" if k == self.steps else f"before row {index}")
            previous = q


def from_settings(workspace, config_dir):
    path = Path(workspace["model_urdf"])
    path = path if path.is_absolute() else Path(config_dir)/path
    chain = SerialChain(path, workspace["base_link"], workspace["flange_link"])
    return WorkspaceCheck(chain, link_points=workspace["link_points"], tool_points_m=workspace["tool_points_m"],
                          flange_link=workspace["flange_link"], table_z_m=workspace["table_z_m"],
                          clearance_m=workspace["clearance_m"], box_min_m=workspace["box_min_m"],
                          box_max_m=workspace["box_max_m"], interpolation_steps=workspace["interpolation_steps"])
