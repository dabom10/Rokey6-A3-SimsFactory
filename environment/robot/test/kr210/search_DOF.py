import omni
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

robot_root = "/Root/run_robot/body"

print("\n==== DOF LIST ====\n")

count = 0

for prim in stage.Traverse():
    path = prim.GetPath().pathString

    if path.startswith(robot_root):
        type_name = prim.GetTypeName()

        if type_name in [
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint"
        ]:
            print(f"{count} : {prim.GetName()}")
            count += 1

print("\nTotal Joints:", count)