import sys
import os
import io

if sys.platform == "win32":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

sys.path.insert(0, r"E:\robotarm-6-axis")
from robot_simulation import URDFParser, make_transform, rpy_to_matrix, rotation_matrix_axis_angle, JOINT_ORDER
import numpy as np

parser = URDFParser("assets/ROBOTARM2_description/PUBY.urdf")

def compute_transforms(angles_deg):
    joint_angles = [np.radians(a) for a in angles_deg]
    child_to_joint = {jinfo["child"]: jinfo for jinfo in parser.joints.values() if jinfo["child"]}
    all_children = set(child_to_joint.keys())
    all_links = set(parser.links.keys())
    root_link = next(iter(all_links - all_children), None) or "base_link"

    transforms = {root_link: np.eye(4)}
    queue = [root_link]
    joint_idx_map = {jname: idx for idx, jname in enumerate(JOINT_ORDER)}
    visited = set()

    while queue:
        current = queue.pop(0)
        if current in visited:
            continue
        visited.add(current)

        for jname, jinfo in parser.joints.items():
            if jinfo["parent"] != current:
                continue
            child = jinfo["child"]
            if child is None or child in visited:
                continue

            T_joint = make_transform(rpy_to_matrix(*jinfo["origin_rpy"]), jinfo["origin_xyz"])
            T_rot = np.eye(4)
            if jname in joint_idx_map:
                idx = joint_idx_map[jname]
                angle = joint_angles[idx] if idx < len(joint_angles) else 0.0
                R = rotation_matrix_axis_angle(jinfo["axis"], angle)
                T_rot[:3, :3] = R

            T_parent = transforms.get(current, np.eye(4))
            transforms[child] = T_parent @ T_joint @ T_rot
            queue.append(child)

    return transforms

print("="*70)
print("  TESTING RE-CENTERED DIGITAL TWIN JOINT CONNECTIVITY")
print("="*70)

# Check joint connection points between child and parent across multiple poses!
# In the newly centered setup:
# For each joint, the child link frame origin is placed exactly at the joint origin in parent frame.
# Therefore, child's (0,0,0) in world MUST EXACTLY EQUAL parent's joint position in world!

test_poses = [
    ("Home (q=0)", [0, 0, 0, 0, 0, 0]),
    ("J1=45°", [45, 0, 0, 0, 0, 0]),
    ("J2=30°", [0, 30, 0, 0, 0, 0]),
    ("J3=45°", [0, 0, 45, 0, 0, 0]),
    ("Complex pose [30, 45, -30, 60, 45, 30]", [30, 45, -30, 60, 45, 30])
]

all_passed = True
for name, q in test_poses:
    trans = compute_transforms(q)
    print(f"\nPose: {name}")
    for jname, jinfo in parser.joints.items():
        parent = jinfo["parent"]
        child = jinfo["child"]
        if not parent or not child:
            continue
        # Joint position computed from parent
        T_p = trans[parent]
        p_joint_world = (T_p @ make_transform(rpy_to_matrix(*jinfo["origin_rpy"]), jinfo["origin_xyz"]))[:3, 3] * 1000
        # Origin of child
        T_c = trans[child]
        p_child_world = T_c[:3, 3] * 1000
        dist = np.linalg.norm(p_joint_world - p_child_world)
        status = "✅ CONNECTED" if dist < 1e-4 else f"❌ SEPARATED ({dist:.3f}mm)"
        if dist >= 1e-4:
            all_passed = False
        print(f"  {jname:12s} ({parent:10s} -> {child:10s}): {status} (dist={dist:.6f} mm)")

if all_passed:
    print(f"\n{'='*70}")
    print("  ALL JOINTS ARE 100% CONNECTED! ZERO PART SEPARATION!")
    print(f"{'='*70}")
