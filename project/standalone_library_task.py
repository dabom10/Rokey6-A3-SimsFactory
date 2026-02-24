#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import traceback
import numpy as np

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.timeline
import omni.usd
import omni.kit.commands

from pxr import UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.prims import SingleXFormPrim
import isaacsim.robot_motion.motion_generation as mg


ENV_USD_PATH = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"


# ============================================================
# Controller (Position Only)
# ============================================================

class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name, robot_articulation, physics_dt=1/60):
        config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        rmp = mg.lula.motion_policies.RmpFlow(**config)
        articulation_policy = mg.ArticulationMotionPolicy(robot_articulation, rmp, physics_dt)
        super().__init__(name=name, articulation_motion_policy=articulation_policy)


def execute_motion(robot, controller, target_pos):

    # 🔥 orientation 완전 제거
    action = controller.forward(
        target_end_effector_position=target_pos
    )

    robot.apply_action(action)

    current_joints = robot.get_joint_positions()
    joint_error = np.abs(current_joints[:6] - action.joint_positions[:6])

    mean_error = np.mean(joint_error)

    carb.log_info(
        f"[IK] target={np.round(target_pos,3)} "
        f"joint_err_mean={mean_error:.4f}"
    )

    if np.all(joint_error < 0.01):
        return True
    return False


# ============================================================
# MAIN
# ============================================================

def main():

    if not os.path.isfile(ENV_USD_PATH):
        raise FileNotFoundError("USD 파일 없음")

    open_stage(ENV_USD_PATH)
    simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    world = World(physics_dt=1/60, rendering_dt=1/60)

    # -------------------------
    # 로봇 탐색
    # -------------------------
    robot_prim_path = None
    ee_path = None

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "run_robot" in path and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            robot_prim_path = path
        if path.endswith("ee_link"):
            ee_path = path

    if not robot_prim_path or not ee_path:
        raise RuntimeError("로봇 탐색 실패")

    ur10 = world.scene.add(
        SingleManipulator(
            prim_path=robot_prim_path,
            name="ur10",
            end_effector_prim_path=ee_path
        )
    )

    # -------------------------
    # 타겟 및 큐브
    # -------------------------
    red_box_target = SingleXFormPrim(
        prim_path=f"{robot_prim_path}/small_KLT",
        name="red_box_target"
    )

    cube_pos = np.array([-11.89, 5.21, 1.35])

    red_book = world.scene.add(
        DynamicCuboid(
            prim_path="/World/red_book",
            name="red_book",
            position=cube_pos,
            scale=np.array([0.114, 0.157, 0.04]),
            color=np.array([1.0, 0.0, 0.0]),
            mass=0.5
        )
    )

    world.reset()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    world.step(render=True)

    # -------------------------
    # 초기 ㄱ자
    # -------------------------
    home_joints = np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])
    joints = ur10.get_joint_positions()
    joints[:6] = home_joints
    ur10.set_joint_positions(joints)

    world.step(render=True)

    controller = RMPFlowController("ur10_controller", ur10)
    controller.reset()

    # -------------------------
    # 상태
    # -------------------------
    STATES = {
        "PREGRASP": 0,
        "LOWER": 1,
        "DONE": 2
    }

    state = STATES["PREGRASP"]

    frame_count = 0

    carb.log_warn("===== POSITION ONLY DEBUG START =====")

    # ============================================================
    # LOOP
    # ============================================================

    while simulation_app.is_running():

        world.step(render=True)
        if not world.is_playing():
            continue

        frame_count += 1

        cube_world, _ = red_book.get_world_pose()
        ee_world, _ = ur10.end_effector.get_world_pose()
        robot_base, _ = ur10.get_world_pose()

        local_cube = cube_world - robot_base

        distance = np.linalg.norm(cube_world - ee_world)

        if frame_count % 30 == 0:
            carb.log_warn("------------------------------------------------")
            carb.log_warn(f"[FRAME] {frame_count}")
            carb.log_warn(f"[STATE] {state}")
            carb.log_warn(f"[CUBE world] {np.round(cube_world,3)}")
            carb.log_warn(f"[EE world]   {np.round(ee_world,3)}")
            carb.log_warn(f"[DIST] {distance:.3f}")
            carb.log_warn("------------------------------------------------")

        # ---------------- PREGRASP ----------------
        if state == STATES["PREGRASP"]:

            target = local_cube + np.array([0, 0, 0.40])

            if execute_motion(ur10, controller, target):
                carb.log_warn(">>> PREGRASP 도달")
                state = STATES["LOWER"]
                controller.reset()

        # ---------------- LOWER ----------------
        elif state == STATES["LOWER"]:

            target = local_cube + np.array([0, 0, 0.02])

            if execute_motion(ur10, controller, target):
                carb.log_warn(">>> LOWER 도달")
                state = STATES["DONE"]

        elif state == STATES["DONE"]:
            pass


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        carb.log_error(f"[FATAL] {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()