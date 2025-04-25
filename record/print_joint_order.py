import os
import sys
import pinocchio as pin

if len(sys.argv) < 2:
    print("Usage: python3 print_joint_order.py path/to/model.urdf")
    sys.exit(1)

urdf_path = sys.argv[1]
model = pin.buildModelFromUrdf(urdf_path)

print("Joint names in order (index: joint):")
for i in range(len(model.joints)):
    print(f"{i}: {model.names[i]}")
