import sys
import os
import io
import shutil

if sys.platform == "win32":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

import trimesh
import numpy as np

print("="*70)
print("  RE-CENTERING ROBOT MESHES TO LOCAL JOINT FRAMES")
print("="*70)

src_mesh_dir = r"E:\robotarm-6-axis\assets\ROBOTARM2_description\meshes"
bak_mesh_dir = r"E:\robotarm-6-axis\Changes\meshes_original"
out_mesh_dir = r"E:\robotarm-6-axis\Changes\meshes"

os.makedirs(bak_mesh_dir, exist_ok=True)
os.makedirs(out_mesh_dir, exist_ok=True)

# Visual offsets from PUBY.urdf (in meters -> convert to mm for STL which is in mm)
offsets_m = {
    "base_link.stl": np.array([0.0, 0.0, 0.0]),
    "link_1_1.stl":  np.array([ 0.225639, -0.14811,   0.082132]),
    "link_2_1.stl":  np.array([ 0.341842, -0.157622,  0.116891]),
    "link_3_1.stl":  np.array([ 0.484994, -0.140326,  0.087227]),
    "link_4_1.stl":  np.array([ 0.198789, -0.02446,  -0.252988]),
    "link_5_1.stl":  np.array([ 0.524173,  0.106516,  0.143093]),
    "link_6_1.stl":  np.array([ 0.539657,  0.098785,  0.20421 ]),
}

for mesh_name, offset_m in offsets_m.items():
    src_file = os.path.join(src_mesh_dir, mesh_name)
    bak_file = os.path.join(bak_mesh_dir, mesh_name)
    out_file = os.path.join(out_mesh_dir, mesh_name)

    # Backup original
    if not os.path.exists(bak_file) and os.path.exists(src_file):
        shutil.copy2(src_file, bak_file)

    print(f"\nProcessing {mesh_name}:")
    mesh = trimesh.load(src_file)
    offset_mm = offset_m * 1000.0
    print(f"  Old bounds min: {mesh.bounds[0]}")
    print(f"  Old bounds max: {mesh.bounds[1]}")
    print(f"  Applying shift offset: {offset_mm} mm")

    mesh.apply_translation(offset_mm)
    print(f"  New bounds min: {mesh.bounds[0]}")
    print(f"  New bounds max: {mesh.bounds[1]}")

    # Export to Changes folder
    mesh.export(out_file)
    # Overwrite assets mesh
    mesh.export(src_file)
    print(f"  -> Saved centered mesh to {out_file} and {src_file}")

print("\nAll 7 meshes successfully re-centered to their joint rotation origins!")
