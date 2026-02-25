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

# ★ 완벽하게 검증된 Isaac Sim 공식 API 사용 (Import Error 해결)
from isaacsim.core.api import World
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.types import ArticulationAction


# ================================
# 설정 상수
# ================================
ENV_USD_PATH = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"

ROBOT_ARTICULATION_ROOT = "/Root/robot/run_robot"
UR10_PRIM_PATH = "/Root/robot/run_robot/ur10"
EE_LINK_PATH = "/Root/robot/run_robot/ur10/ee_link"

# 고장난 옴니그래프 경로 (강제로 끕니다)
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

# 관절 포즈 프리셋 (rad) - HOME 제외
POSE_APPROACH = np.array([0.20, -1.20, 1.40, -1.80, -1.57, 0.20], dtype=np.float64)
POSE_GRASP    = np.array([0.25, -1.05, 1.55, -2.05, -1.57, 0.25], dtype=np.float64)
POSE_LIFT     = POSE_APPROACH.copy()
POSE_MOVE     = np.array([-0.30, -1.20, 1.35, -1.75, -1.57, -0.30], dtype=np.float64)
POSE_PLACE    = np.array([-0.28, -1.05, 1.50, -2.00, -1.57, -0.28], dtype=np.float64)


# ================================
# 유틸리티 함수
# ================================
def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()

def wxyz_to_quatf(q_wxyz):
    w, x, y, z = q_wxyz
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

def get_world_pose(stage: Usd.Stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = xf.ExtractTranslation()
    rot = xf.ExtractRotationQuat()
    pos = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float32)
    quat_wxyz = (float(rot.GetReal()), float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]), float(rot.GetImaginary()[2]))
    return pos, quat_wxyz

def teleport_prim_to_pose(stage: Usd.Stage, prim_path: str, pos_xyz, quat_wxyz) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    
    op_t = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
    op_r = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient), None)
    
    if not op_t or not op_r:
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
    
    # 🛠️ 큐브 빨간색 복구
    prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([(1.0, 0.0, 0.0)])

    try: UsdPhysics.CollisionAPI.Apply(prim)
    except: pass
    try: UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
    except: pass
    try: UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.2)
    except: pass


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

    # 1. 낡고 에러 나는 OmniGraph 강제 종료
    og_prim = stage.GetPrimAtPath(GRAPH_UR10)
    if og_prim.IsValid():
        og_prim.SetActive(False)
        carb.log_warn("[INIT] OmniGraph 컨트롤러를 강제 비활성화했습니다.")

    # 2. 강성(Stiffness)을 키워서 로봇 팔이 무너지지 않게 고정
    fix_ur10_stiffness(stage)

    # 3. World 및 Manipulator 초기화 (가장 안정적인 최신 제어 방식)
    world = World(physics_dt=1/60, rendering_dt=1/60)
    ur10 = world.scene.add(
        SingleManipulator(prim_path=ROBOT_ARTICULATION_ROOT, name="ur10", end_effector_prim_path=EE_LINK_PATH)
    )
    world.reset()

    # 인덱스 매핑
    ur10_indices = [ur10.get_dof_index(n) for n in JOINT_NAMES]
    
    # 제어용 전역 변수
    current_action = None

    def move_robot(q_rad):
        nonlocal current_action
        current_action = ArticulationAction(joint_positions=q_rad, joint_indices=ur10_indices)

    def hold_seconds(seconds: float, attached: bool):
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < seconds:
            # 매 프레임 타겟을 계속 주입하여 중력에 처지거나 다른 포즈로 풀리는 것을 원천 차단
            if current_action is not None:
                ur10.apply_action(current_action)
                
            world.step(render=True)
            
            # Fake Grasp: 책을 EE 링크에 텔레포트
            if attached:
                ee_pos, ee_quat = get_world_pose(stage, EE_LINK_PATH)
                teleport_prim_to_pose(stage, BOOK_PRIM_PATH, ee_pos, ee_quat)

    # 시뮬레이션 플레이 시작
    omni.timeline.get_timeline_interface().play()
    
    # 초반부 대기 (로봇 초기 자세 유지)
    carb.log_warn(f"[WAIT] 로봇 초기 자세 그대로 {START_DELAY_S:.1f}초 대기...")
    for _ in range(int(START_DELAY_S * 60)):
        world.step(render=True)

    # ============================
    # Pick & Place 시퀀스
    # ============================
    carb.log_warn("[RUN] Pick & Place 시작!")
    attached = False

    carb.log_warn(">> 1. APPROACH")
    move_robot(POSE_APPROACH)
    hold_seconds(HOLD_APPROACH_S, attached)

    carb.log_warn(">> 2. GRASP")
    move_robot(POSE_GRASP)
    hold_seconds(HOLD_GRASP_S, attached)

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

    attached = False

    carb.log_warn(">> 6. RETREAT")
    move_robot(POSE_LIFT)
    hold_seconds(HOLD_LIFT_S, attached)

    carb.log_warn("[DONE] 작업 완료. 시뮬레이션 유지...")
    while simulation_app.is_running():
        # 작업이 끝나도 마지막 자세를 단단히 유지합니다.
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