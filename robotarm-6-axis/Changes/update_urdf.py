import sys
import os
import re

urdf_in = r"E:\robotarm-6-axis\Changes\PUBY.urdf.bak"
urdf_out_changes = r"E:\robotarm-6-axis\Changes\PUBY.urdf"
urdf_out_assets = r"E:\robotarm-6-axis\assets\ROBOTARM2_description\PUBY.urdf"

with open(urdf_in, "r", encoding="utf-8") as f:
    content = f.read()

# Replace visual and collision origins for links 1 to 6 with 0 0 0
# Pattern: <origin xyz="..." rpy="..."/> inside <visual> and <collision>
def zero_origin(match):
    return '<origin xyz="0 0 0" rpy="0 0 0"/>'

# Specifically inside visual and collision tags
content_updated = re.sub(
    r'(<(?:visual|collision)>\s*)<origin\s+xyz="[^"]*"\s+rpy="[^"]*"\s*/>',
    r'\1<origin xyz="0 0 0" rpy="0 0 0"/>',
    content
)

with open(urdf_out_changes, "w", encoding="utf-8") as f:
    f.write(content_updated)

with open(urdf_out_assets, "w", encoding="utf-8") as f:
    f.write(content_updated)

print("Updated PUBY.urdf successfully with zero visual/collision offsets!")
