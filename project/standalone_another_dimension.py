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
import omni.kit.app
import omni.graph.core as og

from pxr import Usd, UsdGeom, Gf
from pxr import UsdPhysics


# ================================
# 설정 상수
# ================================
ENV_USD_PATH = "/home/kyb/Rokey6-A3-SimsFactory/project/environment.usd"

# Stage 구조 기준
ROBOT_ARTICULATION_ROOT = "/Root/robot/run_robot"
UR10_PRIM_PATH = "/Root/robot/run_robot/ur10"
EE_LINK_PATH = "/Root/robot/run_robot/ur10/ee_link"

# UR10 팔 컨트롤 그래프
GRAPH_UR10 = "/Root/robot/run_robot/ur10/Graphs/Graphs/Position_Controller"
NODE_UR10_ARTIC = f"{GRAPH_UR10}/ArticulationController"

# Book
BOOK_PRIM_PATH = "/Root/book"
BOOK_CREATE_POS = (-12.7, 5.2, 1.5)
BOOK_SCALE = (0.15, 0.25, 0.04)

# 동작 타이밍
START_DELAY_S = 3.0
HOLD_HOME_S = 1.0
HOLD_APPROACH_S = 2.0
HOLD_GRASP_S = 1.5
HOLD_LIFT_S = 2.0
HOLD_MOVE_S = 2.0
HOLD_PLACE_S = 1.5

# UR10 조인트 이름(일반적으로)
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# 관절 포즈 프리셋 (rad) - 예시
POSE_HOME = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], dtype=np.float64)
POSE_APPROACH = np.array([0.20, -1.20, 1.40, -1.80, -1.57, 0.20], dtype=np.float64)
POSE_GRASP = np.array([0.25, -1.05, 1.55, -2.05, -1.57, 0.25], dtype=np.float64)
POSE_LIFT = POSE_APPROACH.copy()
POSE_MOVE = np.array([-0.30, -1.20, 1.35, -1.75, -1.57, -0.30], dtype=np.float64)
POSE_PLACE = np.array([-0.28, -1.05, 1.50, -2.00, -1.57, -0.28], dtype=np.float64)


# ================================
# 유틸
# ================================
def enable_ext(ext_id: str) -> None:
    app = omni.kit.app.get_app()
    em = app.get_extension_manager()
    if not em.is_extension_enabled(ext_id):
        carb.log_warn(f"[EXT] enabling: {ext_id}")
        em.set_extension_enabled_immediate(ext_id, True)


def get_stage() -> Usd.Stage:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Failed to get stage from omni.usd context")
    return stage


def wxyz_to_quatf(q_wxyz):
    w, x, y, z = q_wxyz
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


def get_world_pose(stage: Usd.Stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")

    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = xf.ExtractTranslation()
    rot = xf.ExtractRotationQuat()
    w = rot.GetReal()
    v = rot.GetImaginary()

    pos = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float32)
    quat_wxyz = (float(w), float(v[0]), float(v[1]), float(v[2]))
    return pos, quat_wxyz


def teleport_prim_to_pose(stage: Usd.Stage, prim_path: str, pos_xyz, quat_wxyz) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")

    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()

    op_t = None
    op_r = None
    op_s = None

    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op_t = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            op_r = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
            op_s = op

    if op_t is None or op_r is None:
        xform.ClearXformOpOrder()
        op_t = xform.AddTranslateOp()
        op_r = xform.AddOrientOp()
        op_s = xform.AddScaleOp()

    if op_s is None:
        op_s = xform.AddScaleOp()

    op_t.Set(Gf.Vec3d(float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])))
    op_r.Set(wxyz_to_quatf(quat_wxyz))


def hold_seconds(seconds: float, attached: bool, stage: Usd.Stage) -> None:
    t0 = time.time()
    while simulation_app.is_running() and (time.time() - t0) < seconds:
        simulation_app.update()
        if attached:
            ee_pos, ee_quat = get_world_pose(stage, EE_LINK_PATH)
            teleport_prim_to_pose(stage, BOOK_PRIM_PATH, ee_pos, ee_quat)


# ================================
# Book 생성 + Physics Preset(스키마) 적용
# ================================
def ensure_book(stage: Usd.Stage) -> None:
    prim = stage.GetPrimAtPath(BOOK_PRIM_PATH)
    if not prim or not prim.IsValid():
        carb.log_warn(f"[BOOK] create cube: {BOOK_PRIM_PATH}")
        cube = UsdGeom.Cube.Define(stage, BOOK_PRIM_PATH)
        cube.CreateSizeAttr(1.0)

    prim = stage.GetPrimAtPath(BOOK_PRIM_PATH)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()

    op_t = xform.AddTranslateOp()
    op_r = xform.AddOrientOp()
    op_s = xform.AddScaleOp()

    op_t.Set(Gf.Vec3d(float(BOOK_CREATE_POS[0]), float(BOOK_CREATE_POS[1]), float(BOOK_CREATE_POS[2])))
    op_r.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    op_s.Set(Gf.Vec3f(float(BOOK_SCALE[0]), float(BOOK_SCALE[1]), float(BOOK_SCALE[2])))

    # Physics preset equivalent: RigidBody + Collider + Mass
    _apply_rigidbody_and_collider(stage, BOOK_PRIM_PATH)


def _apply_rigidbody_and_collider(stage: Usd.Stage, prim_path: str) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")

    # Collider
    try:
        UsdPhysics.CollisionAPI.Apply(prim)
    except Exception:
        pass

    # RigidBody
    try:
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateRigidBodyEnabledAttr(True)
    except Exception:
        pass

    # Mass (너무 가벼우면 튈 수 있어서 적당히)
    try:
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        # 환경 단위가 m 기준이면 0.2kg 정도가 무난
        mass_api.CreateMassAttr(0.2)
    except Exception:
        pass

    carb.log_warn("[BOOK] physics applied: RigidBody + Collision + Mass")


# ================================
# OmniGraph 패치: '/Root/run_robot' -> '/Root/robot/run_robot'
# ================================
def patch_graph_string_inputs(graph_path: str, old: str, new: str) -> int:
    graph = og.get_graph_by_path(graph_path)
    if graph is None:
        carb.log_warn(f"[OG] graph not found: {graph_path}")
        return 0

    ctrl = og.Controller()
    patched = 0

    for node in graph.get_nodes():
        for attr in node.get_attributes():
            try:
                tname = attr.get_type_name().lower()
                if "string" not in tname:
                    continue
                ap = attr.get_path()
                val = ctrl.get(ap)
                if isinstance(val, str) and old in val:
                    ctrl.set(ap, val.replace(old, new))
                    patched += 1
            except Exception:
                pass

    return patched


# ================================
# 핵심: ArticulationController 입력 직접 세팅
# ================================
def set_ur10_joint_positions(q_rad: np.ndarray) -> None:
    ctrl = og.Controller()

    # robotPath는 articulation root로 (run_robot)
    ctrl.set(f"{NODE_UR10_ARTIC}.inputs:robotPath", ROBOT_ARTICULATION_ROOT)

    # targetPrim은 UR10 프림 (target 타입이라 string으로 안 먹는 환경이 있어, 실패해도 계속)
    try:
        ctrl.set(f"{NODE_UR10_ARTIC}.inputs:targetPrim", UR10_PRIM_PATH)
    except Exception:
        pass

    # jointNames / positionCommand
    ctrl.set(f"{NODE_UR10_ARTIC}.inputs:jointNames", list(JOINT_NAMES))
    ctrl.set(f"{NODE_UR10_ARTIC}.inputs:positionCommand", [float(x) for x in q_rad.tolist()])


# ================================
# 메인
# ================================
def main():
    if not os.path.isfile(ENV_USD_PATH):
        raise FileNotFoundError(f"USD not found: {ENV_USD_PATH}")

    # 그래프 노드용
    enable_ext("isaacsim.core.nodes")

    from isaacsim.core.utils.stage import open_stage
    carb.log_warn(f"[STAGE] open_stage: {ENV_USD_PATH}")
    open_stage(ENV_USD_PATH)

    stage = get_stage()

    # prim 체크
    for p in [ROBOT_ARTICULATION_ROOT, UR10_PRIM_PATH, EE_LINK_PATH]:
        if not stage.GetPrimAtPath(p).IsValid():
            raise RuntimeError(f"Required prim not found: {p}")

    # book 생성 + 물리 적용
    ensure_book(stage)

    # timeline play + warmup
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(240):
        simulation_app.update()

    # OG 그래프 내부에 남아있는 잘못된 경로를 패치(중요)
    patched = patch_graph_string_inputs(GRAPH_UR10, "/Root/run_robot", ROBOT_ARTICULATION_ROOT)
    carb.log_warn(f"[OG] patched string attrs in UR10 graph: {patched}")

    # 패치 적용 안정화
    for _ in range(60):
        simulation_app.update()

    # 시작 딜레이
    carb.log_warn(f"[WAIT] {START_DELAY_S:.1f}s")
    hold_seconds(START_DELAY_S, attached=False, stage=stage)

    # ============================
    # Pick & Place (관절 프리셋 + teleport)
    # ============================
    carb.log_warn("[RUN] start")

    attached = False

    set_ur10_joint_positions(POSE_HOME)
    hold_seconds(HOLD_HOME_S, attached, stage)

    set_ur10_joint_positions(POSE_APPROACH)
    hold_seconds(HOLD_APPROACH_S, attached, stage)

    set_ur10_joint_positions(POSE_GRASP)
    hold_seconds(HOLD_GRASP_S, attached, stage)

    attached = True

    set_ur10_joint_positions(POSE_LIFT)
    hold_seconds(HOLD_LIFT_S, attached, stage)

    set_ur10_joint_positions(POSE_MOVE)
    hold_seconds(HOLD_MOVE_S, attached, stage)

    set_ur10_joint_positions(POSE_PLACE)
    hold_seconds(HOLD_PLACE_S, attached, stage)

    attached = False

    carb.log_warn("[DONE] finished. keep running...")
    while simulation_app.is_running():
        simulation_app.update()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        error_text = f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}"
        carb.log_error("[FATAL] Exception occurred.")
        carb.log_error(error_text)
        try:
            with open("/tmp/isaac_fatal.log", "w", encoding="utf-8") as f:
                f.write(error_text)
        except Exception:
            pass
        while simulation_app.is_running():
            try:
                simulation_app.update()
            except Exception:
                break
    finally:
        simulation_app.close()