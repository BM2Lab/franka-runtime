"""Write the hashed manifest for a franka_description FR3 + Franka Hand collision bundle.

The bundle (URDF/SRDF from xacro plus the referenced collision meshes, laid out under
``franka_description/``) must already exist. The Hand mount F_T_EE is taken from a
read-only robot state capture (libfranka column-major ``F_T_EE``), so the model is
bound to the tool the robot reports. ``--min-clearance-m`` is an engineering proposal
for lab review, not a qualified value.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bundle", type=Path, required=True)
    p.add_argument("--workspace-urdf", type=Path, required=True)
    p.add_argument("--state", type=Path, required=True, help="read-only robot state JSON with F_T_EE")
    p.add_argument("--state-origin", help="canonical location of the state evidence to record (default: --state)")
    p.add_argument("--franka-description-version", required=True)
    p.add_argument("--franka-description-commit", required=True)
    p.add_argument("--min-clearance-m", type=float, required=True)
    p.add_argument("--contract-joint-names", nargs=7, default=[f"panda_joint{i}" for i in range(1, 8)])
    args = p.parse_args()
    urdf, srdf = "fr3v2_1_hand.urdf", "fr3v2_1_hand.srdf"
    files = {str(f.relative_to(args.bundle)): sha(f) for f in sorted(args.bundle.rglob("*"))
             if f.is_file() and f.name != "manifest.json"}
    referenced = {m.get("filename", "").replace("package://", "") for m in ET.parse(args.bundle/urdf).getroot().iter("mesh")
                  if "/collision/" in m.get("filename", "")}
    if not referenced <= set(files):
        raise SystemExit(f"Collision meshes missing from bundle: {sorted(referenced-set(files))}")
    c = json.loads(args.state.read_text())["F_T_EE"]  # column-major 4x4
    mount = [[c[col*4+row] for col in range(4)] for row in range(4)]
    import pinocchio as pin
    model = pin.buildModelFromUrdf(str(args.bundle/urdf))
    geom = pin.buildGeomFromUrdf(model, str(args.bundle/urdf), pin.GeometryType.COLLISION, package_dirs=[str(args.bundle)])
    geom.addAllCollisionPairs()
    pin.removeCollisionPairs(model, geom, str(args.bundle/srdf))
    manifest = dict(
        schema_version=1, kind="franka_description_mesh_self_collision_v1",
        engine=dict(pinocchio=pin.__version__, note="pin wheel; same Pinocchio/Coal versions as the Franky build"),
        source=dict(package="franka_description", version=args.franka_description_version,
                    commit=args.franka_description_commit,
                    urdf_xacro="robots/fr3v2.1/fr3v2.1.urdf.xacro", srdf_xacro="robots/fr3v2.1/fr3v2.1.srdf.xacro",
                    xacro_args=dict(arm_id="fr3v2.1", hand="true", with_sc="false"),
                    note="Mesh collision geometry shared with the robot-returned model and the PPO simulation asset; "
                         "URDF/SRDF are unmodified xacro output; visual meshes are referenced but not bundled"),
        urdf=urdf, srdf=srdf, files=files, collision_pairs=len(geom.collisionPairs),
        arm_joints=[f"fr3v2.1_joint{i}" for i in range(1, 8)], contract_joint_names=args.contract_joint_names,
        finger_joints=["fr3v2.1_finger_joint1", "fr3v2.1_finger_joint2"], finger_position_m=0.04,
        flange_frame="fr3v2.1_link8", workspace_urdf_sha256=sha(args.workspace_urdf),
        hand_mount_F_T_EE=mount, hand_mount_source=dict(path=args.state_origin or str(args.state.resolve()), sha256=sha(args.state)),
        min_clearance_m=args.min_clearance_m,
        min_clearance_basis="ENGINEERING PROPOSAL for lab review: margin on mesh geometry; franka_description's own "
                            "with_sc capsules use safety_distance 0.03 m",
        provenance=f"franka_description {args.franka_description_version} ({args.franka_description_commit[:7]}) FR3 v2.1 "
                   f"+ Franka Hand mesh collision model, Franka SRDF exclusions, Pinocchio {pin.__version__}/Coal")
    out = args.bundle/"manifest.json"
    out.write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps(dict(manifest=str(out), sha256=sha(out), files=len(files), collision_pairs=manifest["collision_pairs"]), indent=2))


if __name__ == "__main__":
    main()
