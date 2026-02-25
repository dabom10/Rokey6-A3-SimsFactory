#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import traceback
import numpy as np

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.usd
import omni.timeline

from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics

from isaacsim.core.api import World
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.types import ArticulationAction


# ================================
# 설정 상수
# ================================
ENV_USD_PATH = "/home/kyb/Rokey6-A3-SimsFactory/project/environment.usd"

ROBOT_ARTICULATION_ROOT = "/Root/robot/run_robot"
UR10_PRIM_PATH = "/Root/robot/run_robot/ur10"
EE_LINK_PATH = "/Root/robot/run_robot/ur10/ee_link"

GRAPH_UR10 = "/Root/robot/run_robot/ur10/Graphs/Graphs/Position_Controller"

# Book
BOOK_PRIM_PATH = "/Root/book"
BOOK_CREATE_POS = (-12.7, 5.2, 1.5)
BOOK_SCALE = (0.15, 0.25, 0.04)

# 동작 타이밍
START_DELAY_S = 3.0
HOLD_APPROACH_S = 2.0
HOLD_GRASP_S = 1.5
HOLD_LIFT_S = 2.0
HOLD_MOVE_S = 2.0
HOLD_PLACE_S = 1.5

# UR10 조인트 이름
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# 관절 포즈 프리셋 (rad)
POSE_APPROACH = np.array([-1.5, -1.20, 1.40, -1.80, -1.57, 0.20], dtype=np.float64)
POSE_GRASP    = np.array([-1.5, -1.05, 1.55, -2.05, -1.57, 0.25], dtype=np.float64)
POSE_LIFT     = POSE_APPROACH.copy()
POSE_MOVE     = np.array([-3.4, -1.20, 1.35, -1.75, -1.57, -0.30], dtype=np.float64)
POSE_PLACE    = np.array([-3.4, -1.05, 1.50, -2.00, -1.57, -0.28], dtype=np.float64)

# ================================
# suction_cup 경로 (스크린샷 기준)
# ================================
SUCTION_CUP_PATH = "/Root/robot/run_robot/ur10/ee_link/short_gripper/suction_cup"

# ================================
# "suction_cup 아래 좌표계"로 붙이는 오프셋
# - attached=True일 때:
#   book_world = T_cup_world * [ATTACH_OFFSET_IN_CUP_FRAME, 1]
# - suction_cup 끝 방향이 어느 축인지에 따라 조정 필요
#   예) 끝이 cup의 -Z 방향이면 (0,0,-0.01)
#       끝이 cup의 +Z 방향이면 (0,0,+0.01)
# ================================
ATTACH_OFFSET_IN_CUP_FRAME = np.array([0.0, 0.0, 0.05], dtype=np.float64)

# 책 자세를 cup과 동일하게 맞출지 여부
ATTACH_MATCH_CUP_ORIENTATION = True


# ================================
# 유틸리티 함수
# ================================
def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()

def prim_exists(stage: Usd.Stage, prim_path: str) -> bool:
    prim = stage.GetPrimAtPath(prim_path)
    return bool(prim and prim.IsValid())

def wxyz_to_quatf(q_wxyz):
    w, x, y, z = q_wxyz
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

def get_world_pose(stage: Usd.Stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = xf.ExtractTranslation()
    rot = xf.ExtractRotationQuat()
    pos = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64)
    quat_wxyz = (
        float(rot.GetReal()),
        float(rot.GetImaginary()[0]),
        float(rot.GetImaginary()[1]),
        float(rot.GetImaginary()[2]),
    )
    return pos, quat_wxyz

def quat_wxyz_to_rotmat(q_wxyz: tuple) -> np.ndarray:
    w, x, y, z = [float(v) for v in q_wxyz]
    n = (w*w + x*x + y*y + z*z) ** 0.5
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array(
        [
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ],
        dtype=np.float64,
    )

def teleport_prim_to_pose(stage: Usd.Stage, prim_path: str, pos_xyz, quat_wxyz) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()

    op_t = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
    op_r = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient), None)

    if (op_t is None) or (op_r is None):
        xform.ClearXformOpOrder()
        op_t = xform.AddTranslateOp()
        op_r = xform.AddOrientOp()
        xform.AddScaleOp()

    op_t.Set(Gf.Vec3d(float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])))
    op_r.Set(wxyz_to_quatf(quat_wxyz))


# ================================
# Book 생성 (물리 및 색상 복구)
# ================================
def ensure_book(stage: Usd.Stage) -> None:
    prim = stage.GetPrimAtPath(BOOK_PRIM_PATH)
    if not prim or not prim.IsValid():
        cube = UsdGeom.Cube.Define(stage, BOOK_PRIM_PATH)
        cube.CreateSizeAttr(1.0)

    prim = stage.GetPrimAtPath(BOOK_PRIM_PATH)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()

    op_t = xform.AddTranslateOp()
    op_r = xform.AddOrientOp()
    op_s = xform.AddScaleOp()

    op_t.Set(Gf.Vec3d(*BOOK_CREATE_POS))
    op_r.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    op_s.Set(Gf.Vec3f(*BOOK_SCALE))

    prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([(1.0, 0.0, 0.0)])

    try:
        UsdPhysics.CollisionAPI.Apply(prim)
    except Exception:
        pass
    try:
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
    except Exception:
        pass
    try:
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.2)
    except Exception:
        pass


# ================================
# 조인트 중력 처짐 방지 (Stiffness 펌핑)
# ================================
def fix_ur10_stiffness(stage):
    for prim in stage.Traverse():
        if prim.GetName() in JOINT_NAMES:
            for drive_name in ["angular", "rotX", "rotY", "rotZ"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
                if drive:
                    drive.GetStiffnessAttr().Set(1e7)
                    drive.GetDampingAttr().Set(1e6)


# ================================
# 메인 루프
# ================================
def main():
    if not os.path.isfile(ENV_USD_PATH):
        raise FileNotFoundError(f"USD not found: {ENV_USD_PATH}")

    from isaacsim.core.utils.stage import open_stage
    carb.log_warn(f"[STAGE] open_stage: {ENV_USD_PATH}")
    open_stage(ENV_USD_PATH)
    simulation_app.update()

    stage = get_stage()
    ensure_book(stage)

    og_prim = stage.GetPrimAtPath(GRAPH_UR10)
    if og_prim and og_prim.IsValid():
        og_prim.SetActive(False)
        carb.log_warn("[INIT] OmniGraph 컨트롤러를 강제 비활성화했습니다.")

    fix_ur10_stiffness(stage)

    world = World(physics_dt=1/60, rendering_dt=1/60)
    ur10 = world.scene.add(
        SingleManipulator(prim_path=ROBOT_ARTICULATION_ROOT, name="ur10", end_effector_prim_path=EE_LINK_PATH)
    )
    world.reset()

    if not prim_exists(stage, SUCTION_CUP_PATH):
        raise RuntimeError(f"suction_cup prim not found: {SUCTION_CUP_PATH}")

    ur10_indices = [ur10.get_dof_index(n) for n in JOINT_NAMES]
    current_action = None

    def move_robot(q_rad):
        nonlocal current_action
        current_action = ArticulationAction(joint_positions=q_rad, joint_indices=ur10_indices)

    def hold_seconds(seconds: float, attached: bool):
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < seconds:
            if current_action is not None:
                ur10.apply_action(current_action)

            world.step(render=True)

            # attached=True면:
            # suction_cup 월드포즈를 기준으로 "cup 로컬 오프셋"을 월드로 변환해서 책을 텔레포트
            if attached:
                cup_pos, cup_quat = get_world_pose(stage, SUCTION_CUP_PATH)
                R = quat_wxyz_to_rotmat(cup_quat)
                offset_world = R @ ATTACH_OFFSET_IN_CUP_FRAME
                book_pos = cup_pos + offset_world

                if ATTACH_MATCH_CUP_ORIENTATION:
                    book_quat = cup_quat
                else:
                    # 자세는 그대로 두고 위치만 따라가게 하고 싶으면 False로 두면 됨
                    _, book_quat = get_world_pose(stage, BOOK_PRIM_PATH)

                teleport_prim_to_pose(stage, BOOK_PRIM_PATH, book_pos, book_quat)

    omni.timeline.get_timeline_interface().play()

    carb.log_warn(f"[WAIT] 로봇 초기 자세 그대로 {START_DELAY_S:.1f}초 대기...")
    for _ in range(int(START_DELAY_S * 60)):
        world.step(render=True)

    carb.log_warn("[RUN] Pick & Place 시작!")
    attached = False

    carb.log_warn(">> 1. APPROACH")
    move_robot(POSE_APPROACH)
    hold_seconds(HOLD_APPROACH_S, attached)

    carb.log_warn(">> 2. GRASP")
    move_robot(POSE_GRASP)
    hold_seconds(HOLD_GRASP_S, attached)

    # 여기서부터 suction_cup 기준으로 텔레포트 추종
    attached = True

    carb.log_warn(">> 3. LIFT")
    move_robot(POSE_LIFT)
    hold_seconds(HOLD_LIFT_S, attached)

    carb.log_warn(">> 4. MOVE")
    move_robot(POSE_MOVE)
    hold_seconds(HOLD_MOVE_S, attached)

    carb.log_warn(">> 5. PLACE")
    move_robot(POSE_PLACE)
    hold_seconds(HOLD_PLACE_S, attached)

    # 분리
    attached = False

    carb.log_warn(">> 6. RETREAT")
    move_robot(POSE_LIFT)
    hold_seconds(HOLD_LIFT_S, attached)

    carb.log_warn("[DONE] 작업 완료. 시뮬레이션 유지...")
    while simulation_app.is_running():
        if current_action is not None:
            ur10.apply_action(current_action)
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        carb.log_error(f"[FATAL] Exception occurred: {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()