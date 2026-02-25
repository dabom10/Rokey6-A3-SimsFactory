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
ENV_USD_PATH = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"

ROBOT_ARTICULATION_ROOT = "/Root/robot/run_robot"
UR10_PRIM_PATH = "/Root/robot/run_robot/ur10"
EE_LINK_PATH = "/Root/robot/run_robot/ur10/ee_link"

GRAPH_UR10 = "/Root/robot/run_robot/ur10/Graphs/Graphs/Position_Controller"

# Book 초기 생성 위치
BOOK_SPAWN_POS = (-12.7, 5.2, 1.5)
BOOK_SCALE = (0.15, 0.25, 0.04)

# Book 정의 (색상, 경로)
BOOKS = {
    "red": {
        "path": "/Root/red_book",
        "color": (1.0, 0.0, 0.0),
    },
    "blue": {
        "path": "/Root/blue_book",
        "color": (0.0, 0.0, 1.0),
    },
    "yellow": {
        "path": "/Root/yellow_book",
        "color": (1.0, 1.0, 0.0),
    },
}

# Book spawn 간격 (초)
BOOK_SPAWN_INTERVAL_S = 0.2

# 동작 타이밍
START_DELAY_S = 4.0
HOLD_APPROACH_S = 2.0
HOLD_GRASP_S = 2.0
HOLD_LIFT_S = 2.0
HOLD_MOVE_S = 2.0
HOLD_PLACE_S = 3.0

# UR10 조인트 이름
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# ================================
# 관절 포즈 프리셋 (deg)
# ================================
POSE_READY_DEG    = [0, -90.0, -90.0, -90, 90.0, 0.0]
POSE_APPROACH_DEG = [115, -90.0, -90.0, -90, 90.0, 0.0]
POSE_GRASP_DEG  = [115, -123.0, -87.0, -60.0, 90.0, 0.0]
POSE_LIFT_DEG     = [115, -90.0, -90.0, -90, 90.0, 0.0]
POSE_MOVE_DEG     = POSE_READY_DEG.copy()
POSE_PLACE_RED_DEG    = [5, -120.0, -90.0, -60, 90.0, 0.0]
POSE_PLACE_YELLOW_DEG    = [7, -103.0, -122.0, -45, 90.0, 0.0]
POSE_PLACE_BLUE_DEG    = [10, -70, -140.0, -55, 90.0, 0.0]

# ================================
# suction_cup 자동 탐색 키워드
# ================================
SUCTION_CUP_NAME = "suction_cup"

# suction_cup "로컬 좌표계"에서의 오프셋
ATTACH_OFFSET_IN_CUP_FRAME = np.array([0.0, 0.0, 0.01], dtype=np.float64)

# 책 회전을 cup과 "동기화하지 않음"
ATTACH_MATCH_CUP_ORIENTATION = False


# ================================
# 유틸리티 함수
# ================================
def deg2rad(deg_array):
    return np.deg2rad(np.array(deg_array, dtype=np.float64))

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


def resolve_suction_cup_path(stage: Usd.Stage) -> str:
    direct = EE_LINK_PATH + "/" + SUCTION_CUP_NAME
    if prim_exists(stage, direct):
        carb.log_warn(f"[CUP] found by direct path: {direct}")
        return direct

    ee = stage.GetPrimAtPath(EE_LINK_PATH)
    if ee and ee.IsValid():
        found = []
        for p in Usd.PrimRange(ee):
            if p.GetName() == SUCTION_CUP_NAME:
                found.append(str(p.GetPath()))
        if len(found) == 1:
            carb.log_warn(f"[CUP] found under ee_link: {found[0]}")
            return found[0]
        if len(found) > 1:
            found.sort(key=len)
            carb.log_warn(f"[CUP] multiple found under ee_link, choose: {found[0]}")
            return found[0]

    global_found = []
    for p in stage.Traverse():
        if p.GetName() == SUCTION_CUP_NAME:
            global_found.append(str(p.GetPath()))

    if not global_found:
        raise RuntimeError("suction_cup prim not found anywhere.")

    prefer = [p for p in global_found if EE_LINK_PATH in p]
    pick = (prefer[0] if prefer else global_found[0])

    carb.log_warn(f"[CUP] found by global scan, choose: {pick}")
    return pick


# ================================
# Book 생성 함수 (spawn 시점에 호출)
# ================================
def create_book(stage: Usd.Stage, book_name: str) -> None:
    book_path = BOOKS[book_name]["path"]
    color = BOOKS[book_name]["color"]

    # 이미 존재하면 삭제
    if prim_exists(stage, book_path):
        prim = stage.GetPrimAtPath(book_path)
        stage.RemovePrim(prim)

    # 새 Cube 생성
    cube = UsdGeom.Cube.Define(stage, book_path)
    cube.CreateSizeAttr(1.0)

    prim = stage.GetPrimAtPath(book_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()

    op_t = xform.AddTranslateOp()
    op_r = xform.AddOrientOp()
    op_s = xform.AddScaleOp()

    op_t.Set(Gf.Vec3d(*BOOK_SPAWN_POS))
    op_r.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    op_s.Set(Gf.Vec3f(*BOOK_SCALE))

    # 색상 설정
    prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([color])

    # 물리 설정
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

    carb.log_warn(f"[BOOK] {book_name} spawned at {book_path}")


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
# Pick & Place 작업 함수
# ================================
def pick_and_place_book(
    stage: Usd.Stage,
    world: World,
    ur10,
    ur10_indices,
    book_name: str,
    book_path: str,
    place_pose_deg,
    suction_cup_path: str,
):
    """
    특정 책에 대해 pick & place 작업을 수행합니다.
    """
    carb.log_warn(f"[TASK] {book_name} pick & place 시작")

    current_action = None
    attached_quat_wxyz = None

    def move_robot_deg(q_deg):
        nonlocal current_action
        q_rad = deg2rad(q_deg)
        current_action = ArticulationAction(joint_positions=q_rad, joint_indices=ur10_indices)

    def hold_seconds(seconds: float, attached: bool):
        nonlocal attached_quat_wxyz

        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < seconds:
            if current_action is not None:
                ur10.apply_action(current_action)

            world.step(render=True)

            if attached:
                cup_pos, cup_quat = get_world_pose(stage, suction_cup_path)

                R = quat_wxyz_to_rotmat(cup_quat)
                offset_world = R @ ATTACH_OFFSET_IN_CUP_FRAME
                book_pos = cup_pos + offset_world

                if attached_quat_wxyz is None:
                    _, attached_quat_wxyz = get_world_pose(stage, book_path)

                book_quat = attached_quat_wxyz if not ATTACH_MATCH_CUP_ORIENTATION else cup_quat

                teleport_prim_to_pose(stage, book_path, book_pos, book_quat)

    # Pick & Place 시퀀스
    attached = False

    carb.log_warn(f">> {book_name} 0. READY (deg)")
    move_robot_deg(POSE_READY_DEG)
    hold_seconds(HOLD_APPROACH_S, attached)

    carb.log_warn(f">> {book_name} 1-2. APPROACH (deg)")
    move_robot_deg(POSE_APPROACH_DEG)
    hold_seconds(HOLD_APPROACH_S, attached)

    carb.log_warn(f">> {book_name} 2. GRASP (deg)")
    move_robot_deg(POSE_GRASP_DEG)
    hold_seconds(HOLD_GRASP_S, attached)

    attached = True
    attached_quat_wxyz = None

    carb.log_warn(f">> {book_name} 3. LIFT (deg)")
    move_robot_deg(POSE_LIFT_DEG)
    hold_seconds(HOLD_LIFT_S, attached)

    carb.log_warn(f">> {book_name} 4. MOVE (deg)")
    move_robot_deg(POSE_MOVE_DEG)
    hold_seconds(HOLD_MOVE_S, attached)

    carb.log_warn(f">> {book_name} 5. PLACE (deg)")
    move_robot_deg(place_pose_deg)
    hold_seconds(HOLD_PLACE_S, attached)

    attached = False

    carb.log_warn(f">> {book_name} 6. PLACE 후 잠시 대기")
    move_robot_deg(place_pose_deg)
    hold_seconds(1, attached)

    carb.log_warn(f">> {book_name} 0. READY (deg)")
    move_robot_deg(POSE_READY_DEG)
    hold_seconds(0.2, attached)

    carb.log_warn(f"[TASK] {book_name} pick & place 완료")


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

    suction_cup_path = resolve_suction_cup_path(stage)
    carb.log_warn(f"[CUP] final suction_cup path: {suction_cup_path}")

    ur10_indices = [ur10.get_dof_index(n) for n in JOINT_NAMES]

    omni.timeline.get_timeline_interface().play()

    carb.log_warn(f"[WAIT] 로봇 초기 자세 그대로 {START_DELAY_S:.1f}초 대기...")
    for _ in range(int(START_DELAY_S * 60)):
        world.step(render=True)

    carb.log_warn("[RUN] Pick & Place 시작! (다중 책)")

    # 각 책을 순회하며 spawn & pick&place 수행
    books_order = ["red", "blue", "yellow"]
    place_poses = {
        "red": POSE_PLACE_RED_DEG,
        "blue": POSE_PLACE_BLUE_DEG,
        "yellow": POSE_PLACE_YELLOW_DEG,
    }

    for idx, book_name in enumerate(books_order):
        book_path = BOOKS[book_name]["path"]
        place_pose = place_poses[book_name]

        # 첫 번째 책은 즉시 spawn, 이후는 10초 간격
        if idx > 0:
            carb.log_warn(f"[WAIT] 다음 책 spawn 대기 ({BOOK_SPAWN_INTERVAL_S:.1f}초)...")
            for _ in range(int(BOOK_SPAWN_INTERVAL_S * 60)):
                world.step(render=True)

        # 책 생성
        create_book(stage, book_name)
        simulation_app.update()

        # Pick & Place 수행
        pick_and_place_book(
            stage=stage,
            world=world,
            ur10=ur10,
            ur10_indices=ur10_indices,
            book_name=book_name,
            book_path=book_path,
            place_pose_deg=place_pose,
            suction_cup_path=suction_cup_path,
        )

    carb.log_warn("[DONE] 모든 책 작업 완료. 시뮬레이션 유지...")
    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        carb.log_error(f"[FATAL] Exception occurred: {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()