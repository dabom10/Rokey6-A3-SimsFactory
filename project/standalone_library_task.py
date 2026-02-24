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
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
import isaacsim.robot_motion.motion_generation as mg

ENV_USD_PATH = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"

# ============================================================
# 1. 제어기 및 헬퍼 함수
# ============================================================
class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: SingleArticulation, physics_dt: float = 1.0 / 60.0):
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
    current_joints = robot.get_joint_positions()
    if np.all(np.abs(current_joints[:6] - action.joint_positions[:6]) < 0.01):
        return True
    return False

# 🌟 [핵심 추가] 큐브의 절대 좌표를 로봇 기준 상대 좌표로 바꿔주는 함수
def get_local_pos(world_pos, robot_pos, robot_yaw):
    # 1. 로봇 위치만큼 빼기 (원점 이동)
    rx = world_pos[0] - robot_pos[0]
    ry = world_pos[1] - robot_pos[1]
    rz = world_pos[2] - robot_pos[2]
    
    # 2. 로봇이 회전한 각도의 반대만큼 돌리기 (회전 복구)
    ca = np.cos(-robot_yaw)
    sa = np.sin(-robot_yaw)
    
    local_x = rx * ca - ry * sa
    local_y = rx * sa + ry * ca
    local_z = rz
    return np.array([local_x, local_y, local_z])

def _assert_usd(path: str) -> None:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"USD 파일이 없습니다: {path}")

# ============================================================
# 2. 메인 실행 함수
# ============================================================
def main():
    _assert_usd(ENV_USD_PATH)

    carb.log_info(f"[STAGE] open_stage: {ENV_USD_PATH}")
    open_stage(ENV_USD_PATH)
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()

    world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)

    # 로봇 탐색
    robot_prim_path = None
    ee_path = None
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "run_robot" in path and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            robot_prim_path = path
        if path.endswith("ee_link"):
            ee_path = path

    if not robot_prim_path or not ee_path:
        raise RuntimeError("[ERROR] 로봇(run_robot) 또는 ee_link를 찾을 수 없습니다.")

    # 로봇 래핑
    ur10 = world.scene.add(
        SingleManipulator(prim_path=robot_prim_path, name="ur10", end_effector_prim_path=ee_path)
    )

    # 타겟 큐브 생성
    cube_pos = np.array([-11.894412279327886, 5.214124975327733, 1.3503861474629597])
    red_book = world.scene.add(
        DynamicCuboid(
            prim_path="/World/red_book", name="red_book", position=cube_pos,
            scale=np.array([0.114, 0.157, 0.04]), color=np.array([1.0, 0.0, 0.0]), mass=0.5
        )
    )

    world.reset()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    controller = RMPFlowController("ur10_controller", ur10)
    
    STATES = {
        "IDLE": 0, "PREGRASP": 1, "LOWER": 2, "ATTACH": 3, 
        "LIFT": 4, "MOVE_PLACE": 5, "DETACH": 6, "RETREAT": 7, "DONE": 8
    }
    state = STATES["IDLE"]
    
    # 박스 놓는 위치 (이미 상대 좌표이므로 변환 필요 없음)
    place_position = np.array([-21.351209, 40.533305, 0.396747])
    place_ori = np.array([0.670931, 0.0, 0.0, 0.741519]) 
    grasp_ori = euler_angles_to_quat(np.array([0, np.pi, 0])) 

    carb.log_info("[INFO] 🚀 Pick & Place 시뮬레이션 루프 진입")

    # ============================================================
    # 3. 시뮬레이션 루프
    # ============================================================
    while simulation_app.is_running():
        world.step(render=True)
        if not world.is_playing(): continue

        # 🌟 매 프레임 로봇의 절대 좌표와 각도를 가져옵니다.
        robot_pos, robot_rot = ur10.get_world_pose()
        robot_yaw = quat_to_euler_angles(robot_rot)[2]
        
        # 큐브의 절대 좌표를 로봇 시점(Local)으로 변환합니다!
        current_cube_pos, _ = red_book.get_world_pose()
        cube_local_pos = get_local_pos(current_cube_pos, robot_pos, robot_yaw)

        if state == STATES["IDLE"]:
            # 도착 판단은 World 기준(X좌표)으로 수행
            if current_cube_pos[0] > -12.0: 
                carb.log_info("[Task] Target detected. Moving to PREGRASP.")
                state = STATES["PREGRASP"]
                controller.reset()

        elif state == STATES["PREGRASP"]:
            # 🌟 타겟 명령은 반드시 Local 좌표(cube_local_pos)로 내립니다!
            target = cube_local_pos + np.array([0, 0, 0.20])
            if execute_motion(ur10, controller, target, grasp_ori):
                state = STATES["LOWER"]
                controller.reset()

        elif state == STATES["LOWER"]:
            target = cube_local_pos + np.array([0, 0, 0.02])
            if execute_motion(ur10, controller, target, grasp_ori):
                state = STATES["ATTACH"]

        elif state == STATES["ATTACH"]:
            omni.kit.commands.execute('CreateJointCommand',
                stage=world.stage, joint_type='Fixed',
                path0=ur10.end_effector.prim_path, path1='/World/red_book',
                joint_path='/World/red_book/fake_grasp_joint'
            )
            carb.log_info("[Task] Cube Attached (Fake Grasp).")
            state = STATES["LIFT"]
            controller.reset()

        elif state == STATES["LIFT"]:
            target = cube_local_pos + np.array([0, 0, 0.30])
            if execute_motion(ur10, controller, target, grasp_ori):
                state = STATES["MOVE_PLACE"]
                controller.reset()

        elif state == STATES["MOVE_PLACE"]:
            target = place_position + np.array([0, 0, 0.20])
            if execute_motion(ur10, controller, target, place_ori):
                state = STATES["DETACH"]
                controller.reset()

        elif state == STATES["DETACH"]:
            omni.kit.commands.execute('DeletePrims', paths=['/World/red_book/fake_grasp_joint'])
            carb.log_info("[Task] Cube Detached.")
            state = STATES["RETREAT"]
            controller.reset()

        elif state == STATES["RETREAT"]:
            target = place_position + np.array([0, 0, 0.40])
            if execute_motion(ur10, controller, target, place_ori):
                carb.log_info("[Task] 🎉 Pick & Place Sequence Completed!")
                state = STATES["DONE"]

        elif state == STATES["DONE"]:
            pass

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        carb.log_error(f"[FATAL] {type(e).__name__}: {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()