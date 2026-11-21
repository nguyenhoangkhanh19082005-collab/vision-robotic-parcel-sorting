import sys
import os
import io

if sys.platform == "win32":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# Run Qt offscreen
os.environ["QT_QPA_PLATFORM"] = "offscreen"

sys.path.insert(0, r"E:\robotarm-6-axis")
from PyQt6.QtWidgets import QApplication
from main import WorldClassRoboDKApp
import numpy as np

print("="*70)
print("  TESTING MAIN GUI APPLICATION INITIALIZATION & SIMULATION")
print("="*70)

app = QApplication.instance()
if app is None:
    app = QApplication(sys.argv)

try:
    window = WorldClassRoboDKApp()
    print("✅ WorldClassRoboDKApp instantiated successfully.")

    # Test slider update and FK sync
    window.sliders[0].setValue(30)
    window.sliders[1].setValue(20)
    window.sliders[2].setValue(-15)
    print("✅ Sliders updated and sync_system_pose executed.")

    # Test Task 1 simulation step
    print("Testing Task 1 calculation...")
    q_curr = [s.value() for s in window.sliders]
    T_start = window.kin.fk(np.radians(q_curr))
    q_sol_rad, ok = window.kin.ik_pieper(T_start, q_curr)
    assert ok, "IK failed for start pose!"
    print("✅ Task 1 IK solved successfully.")

    # Test tree right click context menu mapToGlobal
    pos = window.ui.tree_station.rect().center()
    pt_global = window.ui.tree_station.mapToGlobal(pos)
    print(f"✅ Context menu mapToGlobal executed without AttributeError: {pt_global}")

    # Close window cleanly
    window.close()
    print("\n🎉 ALL GUI AND SIMULATION INITIALIZATION TESTS PASSED!")

except Exception as e:
    print(f"❌ Test failed with error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
