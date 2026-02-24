# ============================================================
# Isaac Sim 5.x - UR10 Pick & Place (Production Stable)
# - Parent Xform 기준 고정
# - 기존 XformOp 재사용 (precision mismatch 해결)
# - ROS2 제거
# - world frame 기반 RMPFlow
# ============================================================

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ---------------- Core Imports ----------------
import numpy as np
import carb
import omni.usd
import omni.kit.commands

from pxr import UsdPhysics, UsdGeom, Gf
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.rotations import euler_angles_to_quat
import isaacsim.robot_motion.motion_generation as mg


# ============================================================
# RMPFlow Controller
# ============================================================

class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name, robot_articulation, physics_dt=1/60):
        config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        rmp = mg.lula.motion_policies.RmpFlow(**config)
        articulation_policy = mg.ArticulationMotionPolicy(robot_articulation, rmp, physics_dt)
        super().__init__(name=name, articulation_motion_policy=articulation_policy)


def execute_motion(robot, controller, target_pos, target_ori):
    action = controller.forward(
        target_end_effector_position=target_pos,
        target_end_effector_orientation=target_ori
    )
    robot.apply_action(action)

    current = robot.get_joint_positions()
    return np.all(np.abs(current[:6] - action.joint_positions[:6]) < 0.01)


# ============================================================
# MAIN
# ============================================================

def main():

    env_usd_path = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"

    # ---------------- Stage Open ----------------
    open_stage(env_usd_path)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()

    # ---------------- Articulation Root 탐색 ----------------
    robot_prim_path = None
    ee_path = None

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "run_robot" in path and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            robot_prim_path = path
        if path.endswith("ee_link"):
            ee_path = path

    if robot_prim_path is None:
        raise RuntimeError("run_robot articulation root not found")

    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    parent_prim = robot_prim.GetParent()

    # ============================================================
    # 🔥 기존 XformOp 재사용 (precision mismatch 해결)
    # ============================================================

    xform = UsdGeom.Xformable(parent_prim)
    ops = xform.GetOrderedXformOps()

    translate_op = None
    rotate_op = None

    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            rotate_op = op

    if translate_op is None:
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)

    if rotate_op is None:
        rotate_op = xform.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)

    translate_op.Set(Gf.Vec3d(
        -9.610604115900275,
        6.006036154997052,
        -0.1408392725055303
    ))

    rotate_op.Set(Gf.Vec3d(0, 0, 90))

    carb.log_info("✅ Robot Parent Xform fixed safely")

    # ---------------- World 생성 ----------------
    world = World(physics_dt=1/60, rendering_dt=1/60)

    # ---------------- Robot 등록 ----------------
    ur10 = world.scene.add(
        SingleManipulator(
            prim_path=robot_prim_path,
            name="ur10",
            end_effector_prim_path=ee_path
        )
    )

    # ---------------- Test Object ----------------
    cube_position = np.array([-11.8, 5.2, 1.35])

    red_book = world.scene.add(
        DynamicCuboid(
            prim_path="/red_book",
            name="red_book",
            position=cube_position,
            scale=np.array([0.11, 0.15, 0.04]),
            color=np.array([1, 0, 0]),
            mass=0.5
        )
    )

    # ---------------- Reset ----------------
    world.reset()

    # ---------------- Controller ----------------
    controller = RMPFlowController("ur10_controller", ur10)

    # ---------------- State Machine ----------------
    STATES = {
        "IDLE": 0,
        "PREGRASP": 1,
        "LOWER": 2,
        "ATTACH": 3,
        "LIFT": 4,
        "MOVE_PLACE": 5,
        "DETACH": 6,
        "RETREAT": 7,
        "DONE": 8
    }

    state = STATES["IDLE"]

    place_position = np.array([-21.35, 40.53, 0.39])
    grasp_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))

    carb.log_info("🚀 Simulation Started")

    while simulation_app.is_running():

        world.step(render=True)

        if not world.is_playing():
            continue

        cube_pos, _ = red_book.get_world_pose()

        if state == STATES["IDLE"]:
            state = STATES["PREGRASP"]
            controller.reset()

        elif state == STATES["PREGRASP"]:
            target = cube_pos + np.array([0, 0, 0.2])
            if execute_motion(ur10, controller, target, grasp_orientation):
                state = STATES["LOWER"]
                controller.reset()

        elif state == STATES["LOWER"]:
            target = cube_pos + np.array([0, 0, 0.02])
            if execute_motion(ur10, controller, target, grasp_orientation):
                state = STATES["ATTACH"]

        elif state == STATES["ATTACH"]:
            omni.kit.commands.execute(
                'CreateJointCommand',
                stage=world.stage,
                joint_type='Fixed',
                path0=ur10.end_effector.prim_path,
                path1='/red_book',
                joint_path='/red_book/fake_grasp_joint'
            )
            state = STATES["LIFT"]
            controller.reset()

        elif state == STATES["LIFT"]:
            target = cube_pos + np.array([0, 0, 0.3])
            if execute_motion(ur10, controller, target, grasp_orientation):
                state = STATES["MOVE_PLACE"]
                controller.reset()

        elif state == STATES["MOVE_PLACE"]:
            target = place_position + np.array([0, 0, 0.2])
            if execute_motion(ur10, controller, target, grasp_orientation):
                state = STATES["DETACH"]
                controller.reset()

        elif state == STATES["DETACH"]:
            omni.kit.commands.execute(
                'DeletePrims',
                paths=['/red_book/fake_grasp_joint']
            )
            state = STATES["RETREAT"]
            controller.reset()

        elif state == STATES["RETREAT"]:
            target = place_position + np.array([0, 0, 0.4])
            if execute_motion(ur10, controller, target, grasp_orientation):
                carb.log_info("🎯 Task Complete")
                state = STATES["DONE"]

    simulation_app.close()


if __name__ == "__main__":
    main()
