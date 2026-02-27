#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_move.py  (통합 최종본 - Phase1→2 책 낙하 방지)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[Phase 1]  책 3권 KLT 박스 담기
[전환]     책 3권을 /Root/robot 자식으로 reparent
           → teleport 시 로봇과 함께 이동
           → teleport 완료 후 /Root 로 복귀
[Phase 2]  Home → A(빨간책) → B(노란책) → C(파란책) → Home
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

# ══════════════════════════════════════════════════════════════
#  [Phase 1] 설정
# ══════════════════════════════════════════════════════════════
P1_USD_PATH             = "/home/kyb/Rokey6-A3-SimsFactory/project/environment.usd"
P1_ROBOT_ART_ROOT       = "/Root/robot/robot"
P1_EE_LINK_PATH         = "/Root/robot/robot/nova_carter/ur10/ee_link"
P1_CAMERA_PRIM_PATH     = "/Root/robot/robot/nova_carter/ur10/ee_link/short_gripper/Camera"

P1_BOOK_SPAWN_POS       = (-2.34664195001105, -1.0139820630486194, 1.3490003153683388)
P1_BOOK_SCALE           = (0.15, 0.25, 0.04)
P1_BOOK_SPAWN_INTERVAL_S = 1.0

import numpy as _np
P1_BOOKS = {
    "red":    {"path": "/Root/red_book",    "color": (1.0, 0.0, 0.0),
               "hsv_ranges": [(_np.array([0,30,30]),   _np.array([10,255,255])),
                               (_np.array([170,30,30]), _np.array([179,255,255]))]},
    "blue":   {"path": "/Root/blue_book",   "color": (0.0, 0.0, 1.0),
               "hsv_ranges": [(_np.array([100,30,30]), _np.array([130,255,255]))]},
    "yellow": {"path": "/Root/yellow_book", "color": (1.0, 1.0, 0.0),
               "hsv_ranges": [(_np.array([15,30,30]),  _np.array([35,255,255]))]},
}

P1_START_DELAY_S        = 4.0
P1_HOLD_APPROACH_S      = 2.0
P1_HOLD_GRASP_S         = 2.0
P1_HOLD_LIFT_S          = 2.0
P1_HOLD_MOVE_S          = 2.0
P1_HOLD_PLACE_S         = 3.0
P1_COLOR_DETECT_WAIT_S  = 3.0

P1_JOINT_NAMES = [
    "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
    "wrist_1_joint","wrist_2_joint","wrist_3_joint",
]
P1_POSES = {
    "ready":        [0,    -90.0,  -90.0,  -90,   90.0,  0.0],
    "approach":     [115,  -90.0,  -90.0,  -90,   90.0,  0.0],
    "grasp":        [115, -123.0,  -87.0,  -60.0, 90.0,  0.0],
    "lift":         [115,  -90.0,  -90.0,  -90,   90.0,  0.0],
    "move":         [0,    -90.0,  -90.0,  -90,   90.0,  0.0],
    "place_red":    [5,   -120.0,  -90.0,  -60,   90.0,  0.0],
    "place_yellow": [7,   -103.0, -122.0,  -45,   90.0,  0.0],
    "place_blue":   [10,   -70.0, -140.0,  -55,   90.0,  0.0],
}
P1_SUCTION_CUP_NAME        = "suction_cup"
P1_ATTACH_OFFSET           = _np.array([0.0, 0.0, 0.01], dtype=_np.float64)
P1_ATTACH_MATCH_CUP_ORIENT = False

# ══════════════════════════════════════════════════════════════
#  [Phase 2] 설정
# ══════════════════════════════════════════════════════════════
P2_ROBOT_PRIM_PATH = "/Root/robot"
P2_EE_LINK_PATH    = "/Root/robot/robot/nova_carter/ur10/ee_link"

P2_WAYPOINTS = {
    "Home": ( 0.4,     0.0,     0.09004,  0.0),
    "A":    ( 5.704565065, 8.56107, 0.09004, 90.0),
    "B":    (-1.43879 ,11.48552, 0.09004, 90.0),
    "C":    (-5.032553, 8.53896, 0.09004, 90.0),
}
P2_SEQUENCE    = ["Home", "A", "B", "C", "Home"]
P2_SHELF_STOPS = {"A", "B", "C"}
P2_DWELL_TIME  = 3.0
P2_PLACE_COLOR = {"A": "red", "B": "yellow", "C": "blue"}

P2_JOINT_NAMES          = P1_JOINT_NAMES
P2_POSE_READY_DEG       = [0,   -90.0,  -90.0, -90,  90.0,   0.0]
P2_POSE_PICK_BY_COLOR   = {
    "red":    [5,  -127.0,  -92.0, -53,  90.0,  0.0],
    "yellow": [7,  -105.0, -121.0, -45,  90.0,  0.0],
    "blue":   [10,  -85.0, -145.0, -40,  90.0,  0.0],
}
P2_POSE_MID_BY_COLOR    = {
    "red":    [5,  -110.0,  -78.0, -79,  90.0,  0.0],
    "yellow": [7,   -90.0, -110.0, -70,  90.0,  0.0],
    "blue":   [10,  -66.0, -130.0, -72,  90.0,  0.0],
}
P2_POSE_2SHELF_DEG      = [90, -120.0, -70.0, -90, 180.0, -15.0]
P2_POSE_PLACE2SHELF_DEG = [90, -140.0, -70.0, -70, 180.0, -15.0]
P2_SUCTION_CUP_NAME     = "suction_cup"
P2_ATTACH_OFFSET        = [0.0, 0.0, 0.01]

P2_CARTER_ACTIONGRAPH_PATHS = [
    "/Root/robot/robot/nova_carter/ActionGraph_differential",
    "/Root/robot/robot/nova_carter/ActionGraph_tf_odom",
    "/Root/robot/robot/nova_carter/ActionGraph_lidar",
]
P2_START_DELAY_S              = 2.0
P2_READY_BEFORE_ACTION_WAIT_S = 1.0
P2_HOLD_MOVE_S                = 3.0
P2_HOLD_ATTACH_S              = 0.6
P2_HOLD_DETACH_S              = 0.6

# ══════════════════════════════════════════════════════════════
#  공통 import
# ══════════════════════════════════════════════════════════════
import os, math, time, traceback
import numpy as np
from collections import defaultdict
import cv2

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.kit.app
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.ros2.bridge", True
)

import carb
import omni.usd
import omni.kit.commands
import omni.timeline

from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import open_stage


# ══════════════════════════════════════════════════════════════
#  공통 헬퍼
# ══════════════════════════════════════════════════════════════

def rpy_deg_to_quatd(roll=0.0, pitch=0.0, yaw=0.0):
    r, p, y_ = math.radians(roll), math.radians(pitch), math.radians(yaw)
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y_/2), math.sin(y_/2)
    return Gf.Quatd(
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    )


def prim_exists(stage, path):
    p = stage.GetPrimAtPath(path)
    return bool(p and p.IsValid())


def get_world_matrix(stage, path) -> Gf.Matrix4d:
    prim = stage.GetPrimAtPath(path)
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def move_prim(src: str, dst: str):
    omni.kit.commands.execute("MovePrim", path_from=src, path_to=dst)


def reparent_with_world_pose(stage, src_path: str, dst_parent_path: str) -> str:
    """
    src_path prim을 dst_parent_path 하위로 이동하면서
    World Pose를 유지하도록 local transform 재계산.
    반환: 새 경로
    """
    prim      = stage.GetPrimAtPath(src_path)
    prim_name = prim.GetName()
    new_path  = str(Sdf.Path(dst_parent_path).AppendChild(prim_name))

    # world pose 저장
    world_m = get_world_matrix(stage, src_path)

    # prim 이동
    move_prim(src_path, new_path)

    # 새 parent world pose 계산 → local = world * parent_inv
    parent_world_m = get_world_matrix(stage, dst_parent_path)
    local_m        = world_m * parent_world_m.GetInverse()

    # xform 재설정
    moved = stage.GetPrimAtPath(new_path)
    xf    = UsdGeom.Xformable(moved)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(local_m))

    return new_path


def attach_books_to_robot(stage, robot_root_path: str) -> dict:
    """
    Phase1→2 전환 시 책 3권을 robot 자식으로 reparent.
    → teleport 시 로봇과 함께 이동.
    반환: {color: new_book_path}
    """
    result = {}
    for color, info in P1_BOOKS.items():
        orig_path = find_book_by_color(stage, color, known_path=info["path"])
        new_path  = reparent_with_world_pose(stage, orig_path, robot_root_path)
        result[color] = new_path
        carb.log_warn(f"[ATTACH_ROBOT] {color}: {orig_path} → {new_path}")
    return result


def detach_books_from_robot(stage, book_paths: dict, restore_parent: str = "/Root") -> dict:
    """
    teleport 완료 후 책을 restore_parent 로 복귀.
    반환: {color: restored_path}
    """
    result = {}
    for color, path in book_paths.items():
        if not prim_exists(stage, path):
            carb.log_warn(f"[DETACH_ROBOT] {color} prim 없음, 건너뜀: {path}")
            continue
        new_path = reparent_with_world_pose(stage, path, restore_parent)
        result[color] = new_path
        carb.log_warn(f"[DETACH_ROBOT] {color}: {path} → {new_path}")
    return result


def detach_single_book_from_robot(stage, color: str,
                                   book_paths: dict,
                                   restore_parent: str = "/Root") -> str:
    """
    A/B/C 책꽂기 직전에 해당 색상 책 1권만 robot 자식에서 해제.
    반환: 복귀된 경로
    """
    path = book_paths.get(color)
    if not path or not prim_exists(stage, path):
        carb.log_warn(f"[DETACH_SINGLE] '{color}' 재탐색")
        path = find_book_by_color(stage, color)

    new_path = reparent_with_world_pose(stage, path, restore_parent)
    carb.log_warn(f"[DETACH_SINGLE] {color}: {path} → {new_path}")
    return new_path


def teleport_robot(world, robot_prim, x, y, z, orient_z_deg):
    """로봇 루트 prim을 순간이동. 책이 자식이면 함께 이동됨."""
    xformable    = UsdGeom.Xformable(robot_prim)
    existing_ops = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}
    quat         = rpy_deg_to_quatd(yaw=orient_z_deg)

    if "xformOp:translate" in existing_ops:
        existing_ops["xformOp:translate"].Set(Gf.Vec3d(x, y, z))
    else:
        xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

    if "xformOp:orient" in existing_ops:
        op = existing_ops["xformOp:orient"]
        tn = op.GetAttr().GetTypeName().type.typeName.lower()
        if "quatd" in tn:
            op.Set(Gf.Quatd(quat.GetReal(), quat.GetImaginary()))
        else:
            op.Set(Gf.Quatf(float(quat.GetReal()),
                            float(quat.GetImaginary()[0]),
                            float(quat.GetImaginary()[1]),
                            float(quat.GetImaginary()[2])))
    else:
        xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)

    for attr_name in ("physics:velocity", "physics:angularVelocity"):
        attr = robot_prim.GetAttribute(attr_name)
        if attr:
            attr.Set(Gf.Vec3f(0, 0, 0))

    world.step(render=True)


def dwell(world, seconds):
    t0 = time.time()
    while simulation_app.is_running() and (time.time() - t0) < seconds:
        world.step(render=True)


def find_book_by_color(stage, color: str, known_path: str = None) -> str:
    if known_path:
        p = stage.GetPrimAtPath(known_path)
        if p and p.IsValid():
            return known_path

    color_kw = color.lower()
    EXCLUDE  = ["look", "material", "shader", "mesh", "decal", "texture", "floor"]

    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if "book" in name and color_kw in name:
            path = str(prim.GetPath())
            carb.log_warn(f"[BOOK] '{color}' (book+color): {path}")
            return path

    candidates = []
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        path = str(prim.GetPath())
        if color_kw in name and not any(ex in path.lower() for ex in EXCLUDE):
            candidates.append(path)

    if candidates:
        candidates.sort(key=len)
        carb.log_warn(f"[BOOK] '{color}' 선택: {candidates[0]}")
        return candidates[0]

    carb.log_warn(f"[BOOK] '{color}' 탐색 실패. depth≤4 prim:")
    for prim in stage.Traverse():
        if prim.GetPath().pathString.count("/") <= 4:
            carb.log_warn(f"  {prim.GetPath()}")
    raise RuntimeError(f"[BOOK] '{color}' 책 prim을 찾을 수 없습니다.")


# ══════════════════════════════════════════════════════════════
#  Phase 1: place2amr
# ══════════════════════════════════════════════════════════════

class place2amr:
    def __init__(self, world, stage, art):
        self.world        = world
        self.stage        = stage
        self.robot_art    = art
        self.ur10_indices = [art.get_dof_index(n) for n in P1_JOINT_NAMES]
        self.suction_cup_path = self._resolve_cup(stage)
        carb.log_warn(f"[P1] suction_cup: {self.suction_cup_path}")

    @staticmethod
    def _prim_exists(stage, path):
        p = stage.GetPrimAtPath(path)
        return bool(p and p.IsValid())

    @staticmethod
    def deg2rad(arr):
        return np.deg2rad(np.array(arr, dtype=np.float64))

    @staticmethod
    def get_world_pose(stage, path):
        prim = stage.GetPrimAtPath(path)
        xf   = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t    = xf.ExtractTranslation()
        rot  = xf.ExtractRotationQuat()
        pos  = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64)
        q    = (float(rot.GetReal()),
                float(rot.GetImaginary()[0]),
                float(rot.GetImaginary()[1]),
                float(rot.GetImaginary()[2]))
        return pos, q

    @staticmethod
    def quat_to_rotmat(q):
        w,x,y,z = [float(v) for v in q]
        n = (w*w+x*x+y*y+z*z)**0.5
        if n < 1e-12: return np.eye(3)
        w,x,y,z = w/n,x/n,y/n,z/n
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
        ])

    @staticmethod
    def wxyz_to_quatf(q):
        w,x,y,z = q
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

    def _resolve_cup(self, stage):
        direct = P1_EE_LINK_PATH + "/" + P1_SUCTION_CUP_NAME
        if self._prim_exists(stage, direct):
            return direct
        for p in stage.Traverse():
            if p.GetName() == P1_SUCTION_CUP_NAME:
                return str(p.GetPath())
        raise RuntimeError("suction_cup not found")

    def _teleport_prim(self, path, pos, quat_wxyz):
        prim  = self.stage.GetPrimAtPath(path)
        xform = UsdGeom.Xformable(prim)
        ops   = xform.GetOrderedXformOps()
        op_t  = next((o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeTranslate), None)
        op_r  = next((o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeOrient),    None)
        if op_t is None or op_r is None:
            xform.ClearXformOpOrder()
            op_t = xform.AddTranslateOp()
            op_r = xform.AddOrientOp()
            xform.AddScaleOp()
        op_t.Set(Gf.Vec3d(*[float(v) for v in pos]))
        op_r.Set(self.wxyz_to_quatf(quat_wxyz))

    def _get_camera_image(self):
        try:
            from omni.isaac.sensor import Camera
            cam = Camera(prim_path=P1_CAMERA_PRIM_PATH)
            cam.initialize()
            rgb = cam.get_rgb()
            if rgb is None or rgb.size == 0:
                return None
            arr = (rgb[:,:,:3]*255).astype(np.uint8) if rgb.max() <= 1.0 \
                  else rgb[:,:,:3].astype(np.uint8)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        except Exception as e:
            carb.log_warn(f"[P1/CAM] {e}")
            return None

    def _detect_color(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        cnt = defaultdict(int)
        k   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        for name, info in P1_BOOKS.items():
            mask = None
            for lo, hi in info["hsv_ranges"]:
                m    = cv2.inRange(hsv, lo, hi)
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask      = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
            mask      = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            cnt[name] = cv2.countNonZero(mask)
        if cnt:
            best = max(cnt, key=cnt.get)
            if cnt[best] > 200:
                return best
        return "unknown"

    def _fix_stiffness(self):
        for prim in self.stage.Traverse():
            if prim.GetName() in P1_JOINT_NAMES:
                for dn in ["angular","rotX","rotY","rotZ"]:
                    drv = UsdPhysics.DriveAPI.Get(prim, dn)
                    if drv:
                        drv.GetStiffnessAttr().Set(1e7)
                        drv.GetDampingAttr().Set(1e6)

    def _create_book(self, book_name):
        path  = P1_BOOKS[book_name]["path"]
        color = P1_BOOKS[book_name]["color"]
        if self._prim_exists(self.stage, path):
            self.stage.RemovePrim(self.stage.GetPrimAtPath(path))
        cube  = UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        prim  = self.stage.GetPrimAtPath(path)
        xf    = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(P1_BOOK_SPAWN_POS)
        xf.AddOrientOp().Set(Gf.Quatf(1,0,0,0))
        xf.AddScaleOp().Set(P1_BOOK_SCALE)
        prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([color])
        try: UsdPhysics.CollisionAPI.Apply(prim)
        except: pass
        try: UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
        except: pass
        try: UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.2)
        except: pass
        carb.log_warn(f"[P1] {book_name} spawned: {path}")

    def _pick_and_place(self, book_name):
        stage            = self.stage
        book_path        = P1_BOOKS[book_name]["path"]
        suction_cup_path = self.suction_cup_path

        current_action     = None
        attached_quat_wxyz = None

        def move(q_deg):
            nonlocal current_action
            current_action = ArticulationAction(
                joint_positions=self.deg2rad(q_deg),
                joint_indices=np.array(self.ur10_indices, dtype=np.int32),
            )

        def hold(sec, is_att):
            nonlocal attached_quat_wxyz
            t0 = time.time()
            while simulation_app.is_running() and (time.time()-t0) < sec:
                if current_action is not None:
                    self.robot_art.apply_action(current_action)
                self.world.step(render=True)
                if is_att:
                    cp, cq = self.get_world_pose(stage, suction_cup_path)
                    off    = self.quat_to_rotmat(cq) @ P1_ATTACH_OFFSET
                    bp     = cp + off
                    if attached_quat_wxyz is None:
                        _, attached_quat_wxyz = self.get_world_pose(stage, book_path)
                    bq = attached_quat_wxyz if not P1_ATTACH_MATCH_CUP_ORIENT else cq
                    self._teleport_prim(book_path, bp, bq)

        carb.log_warn(f"[P1] {book_name} → READY")
        move(P1_POSES["ready"]); hold(P1_HOLD_APPROACH_S, False)

        carb.log_warn(f"[P1] {book_name} → APPROACH")
        move(P1_POSES["approach"]); hold(P1_HOLD_APPROACH_S, False)

        detected = "unknown"
        t0 = time.time(); attempt = 0
        while simulation_app.is_running() and (time.time()-t0) < P1_COLOR_DETECT_WAIT_S:
            if current_action is not None:
                self.robot_art.apply_action(current_action)
            self.world.step(render=True)
            el = int(time.time()-t0)
            if el != attempt:
                attempt = el
                img = self._get_camera_image()
                if img is not None:
                    res = self._detect_color(img)
                    if res != "unknown":
                        detected = res
                        carb.log_warn(f"[P1] {book_name} 색상 탐지: {detected}")
                        break

        if detected == "unknown":
            img = self._get_camera_image()
            if img is not None:
                detected = self._detect_color(img)
        carb.log_warn(f"[P1] {book_name} 최종 색상: {detected}")

        carb.log_warn(f"[P1] {book_name} → GRASP")
        move(P1_POSES["grasp"]); hold(P1_HOLD_GRASP_S, False)

        attached_quat_wxyz = None
        carb.log_warn(f"[P1] {book_name} → LIFT")
        move(P1_POSES["lift"]); hold(P1_HOLD_LIFT_S, True)

        carb.log_warn(f"[P1] {book_name} → MOVE")
        move(P1_POSES["move"]); hold(P1_HOLD_MOVE_S, True)

        place_pose = P1_POSES.get(f"place_{detected}", P1_POSES["place_blue"])
        carb.log_warn(f"[P1] {book_name} → PLACE ({detected})")
        move(place_pose); hold(P1_HOLD_PLACE_S, True)

        hold(1.0, False)

        carb.log_warn(f"[P1] {book_name} → READY 복귀")
        move(P1_POSES["ready"]); hold(P1_HOLD_PLACE_S, False)
        carb.log_warn(f"[P1] {book_name} 완료")

    def run_phase1(self):
        self._fix_stiffness()
        carb.log_warn("[P1] Pick & Place 시작!")
        books_order = ["red", "blue", "yellow"]
        for idx, book_name in enumerate(books_order):
            if idx > 0:
                dwell(self.world, P1_BOOK_SPAWN_INTERVAL_S)
            self._create_book(book_name)
            simulation_app.update()
            self._pick_and_place(book_name)
        carb.log_warn("[P1] 모든 책 KLT 박스 담기 완료!")


# ══════════════════════════════════════════════════════════════
#  Phase 2: amr2shelf
# ══════════════════════════════════════════════════════════════

class amr2shelf:
    def __init__(self, world, stage, art, place, book_paths: dict):
        """
        book_paths: {color: current_stage_path}
        Phase1→2 전환 후 detach_books_from_robot()이 반환한 경로를 넘겨받음.
        """
        self.world        = world
        self.stage        = stage
        self.art          = art
        self.place        = str(place).strip().upper()
        self.target_color = P2_PLACE_COLOR[self.place]

        # 전달받은 경로 우선, 없으면 탐색
        known = book_paths.get(self.target_color,
                               P1_BOOKS[self.target_color]["path"])
        self.book_path = find_book_by_color(stage, self.target_color, known_path=known)

        self.pose_pick_deg    = P2_POSE_PICK_BY_COLOR[self.target_color]
        self.pose_mid_deg     = P2_POSE_MID_BY_COLOR[self.target_color]
        self.arm_indices      = [art.get_dof_index(n) for n in P2_JOINT_NAMES]
        self.suction_cup_path = self._resolve_cup()
        self.attach_offset    = np.array(P2_ATTACH_OFFSET, dtype=np.float64)

        self._attached             = False
        self._original_parent_path = None
        self._attached_path        = None
        self._book_name            = None
        self._grasp_frame_path     = None

    @staticmethod
    def _prim_exists(stage, path):
        p = stage.GetPrimAtPath(path)
        return bool(p and p.IsValid())

    @staticmethod
    def _world_xf(stage, path):
        prim = stage.GetPrimAtPath(path)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    @staticmethod
    def _move_prim(src, dst):
        omni.kit.commands.execute("MovePrim", path_from=src, path_to=dst)

    def _resolve_cup(self):
        direct = P2_EE_LINK_PATH + "/" + P2_SUCTION_CUP_NAME
        if self._prim_exists(self.stage, direct):
            return direct
        for p in self.stage.Traverse():
            if p.GetName() == P2_SUCTION_CUP_NAME:
                return str(p.GetPath())
        raise RuntimeError("suction_cup not found")

    def _ensure_grasp_frame(self):
        path = str(Sdf.Path(self.suction_cup_path).AppendChild("grasp_frame"))
        if self._prim_exists(self.stage, path):
            return path
        prim = self.stage.DefinePrim(path, "Xform")
        xf   = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0,0,0))
        xf.AddOrientOp().Set(Gf.Quatf(1,0,0,0))
        xf.AddScaleOp().Set(Gf.Vec3f(1,1,1))
        return path

    def _apply_arm(self, q_deg):
        self.art.apply_action(ArticulationAction(
            joint_positions=np.deg2rad(np.array(q_deg, dtype=np.float64)),
            joint_indices=self.arm_indices,
        ))

    def _hold(self, sec, q_deg=None):
        t0 = time.time()
        while simulation_app.is_running() and (time.time()-t0) < sec:
            if q_deg is not None:
                self._apply_arm(q_deg)
            self.world.step(render=True)

    def attach(self):
        if self._attached:
            return
        book_prim = self.stage.GetPrimAtPath(self.book_path)
        if not book_prim or not book_prim.IsValid():
            carb.log_warn(f"[P2/ATTACH] 경로 재탐색: {self.book_path}")
            self.book_path = find_book_by_color(self.stage, self.target_color)
            book_prim      = self.stage.GetPrimAtPath(self.book_path)

        self._grasp_frame_path     = self._ensure_grasp_frame()
        self._book_name            = book_prim.GetName()
        self._original_parent_path = str(book_prim.GetParent().GetPath())

        book_world = self._world_xf(self.stage, self.book_path)
        new_path   = str(Sdf.Path(self._grasp_frame_path).AppendChild(self._book_name))
        self._move_prim(self.book_path, new_path)

        parent_world = self._world_xf(self.stage, self._grasp_frame_path)
        local_m      = Gf.Matrix4d(book_world * parent_world.GetInverse())
        local_m.SetTranslateOnly(
            local_m.ExtractTranslation() + Gf.Vec3d(*self.attach_offset.tolist())
        )
        xf = UsdGeom.Xformable(self.stage.GetPrimAtPath(new_path))
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local_m)

        self._attached      = True
        self._attached_path = new_path
        carb.log_warn(f"[P2/ATTACH] {self.place}({self.target_color}): {self.book_path} -> {new_path}")

    def detach(self):
        if not self._attached:
            return
        book_world = self._world_xf(self.stage, self._attached_path)
        dst_path   = str(Sdf.Path(self._original_parent_path).AppendChild(self._book_name))
        self._move_prim(self._attached_path, dst_path)
        parent_world = self._world_xf(self.stage, self._original_parent_path)
        local_m      = book_world * parent_world.GetInverse()
        xf = UsdGeom.Xformable(self.stage.GetPrimAtPath(dst_path))
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local_m)
        self.book_path      = dst_path
        self._attached      = False
        self._attached_path = None
        carb.log_warn(f"[P2/DETACH] {self.place}({self.target_color}) -> {dst_path}")

    def run_sequence(self):
        carb.log_warn(f"\n{'='*52}")
        carb.log_warn(f"[P2] PLACE={self.place} COLOR={self.target_color} BOOK={self.book_path}")
        carb.log_warn(f"{'='*52}")

        carb.log_warn("0) READY")
        self._hold(P2_HOLD_MOVE_S, P2_POSE_READY_DEG)
        self._hold(P2_READY_BEFORE_ACTION_WAIT_S, P2_POSE_READY_DEG)
        carb.log_warn("1) PICK pose")
        self._hold(P2_HOLD_MOVE_S, self.pose_pick_deg)
        carb.log_warn("2) ATTACH")
        self.attach()
        self._hold(P2_HOLD_ATTACH_S, self.pose_pick_deg)
        carb.log_warn("3) MID")
        self._hold(P2_HOLD_MOVE_S, self.pose_mid_deg)
        carb.log_warn("4) TO SHELF")
        self._hold(P2_HOLD_MOVE_S, P2_POSE_2SHELF_DEG)
        carb.log_warn("5) PLACE")
        self._hold(P2_HOLD_MOVE_S, P2_POSE_PLACE2SHELF_DEG)
        carb.log_warn("6) DETACH")
        self.detach()
        self._hold(P2_HOLD_DETACH_S, P2_POSE_PLACE2SHELF_DEG)
        carb.log_warn("7) READY (완료)")
        self._hold(P2_HOLD_MOVE_S, P2_POSE_READY_DEG)


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════

def main():
    carb.log_warn(f"\n[INIT] USD 로드: {P1_USD_PATH}")
    open_stage(usd_path=P1_USD_PATH)
    for _ in range(10):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()

    for ag_path in P2_CARTER_ACTIONGRAPH_PATHS:
        ag = stage.GetPrimAtPath(ag_path)
        if ag and ag.IsValid():
            ag.SetActive(False)
            carb.log_warn(f"[INIT] ActionGraph disabled: {ag_path}")

    world = World(physics_dt=1/60, rendering_dt=1/60)
    art   = world.scene.add(
        SingleArticulation(prim_path=P1_ROBOT_ART_ROOT, name="carter_ur10")
    )
    world.reset()
    omni.timeline.get_timeline_interface().play()

    carb.log_warn(f"[INIT] 초기 안정화 {P1_START_DELAY_S}초 대기...")
    for _ in range(int(P1_START_DELAY_S * 60)):
        world.step(render=True)

    # ── Phase 1 ──────────────────────────────────
    carb.log_warn("\n" + "="*60)
    carb.log_warn("  [PHASE 1]  책 → KLT 박스 담기 시작")
    carb.log_warn("="*60)

    p1 = place2amr(world=world, stage=stage, art=art)
    p1.run_phase1()

    # ── Phase 1 → 2 전환: 책을 로봇 자식으로 ────
    carb.log_warn("\n" + "="*60)
    carb.log_warn("  [전환]  책 3권 → /Root/robot 자식으로 reparent (낙하 방지)")
    carb.log_warn("="*60)

    # 책 경로를 현재 stage에서 확인 후 robot 자식으로 이동
    robot_book_paths = attach_books_to_robot(stage, robot_root_path=P2_ROBOT_PRIM_PATH)

    # 안정화
    dwell(world, P2_START_DELAY_S)

    # ── Phase 2 ──────────────────────────────────
    robot_prim = stage.GetPrimAtPath(P2_ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(f"[ERROR] robot prim 없음: {P2_ROBOT_PRIM_PATH}")

    carb.log_warn("\n" + "="*60)
    carb.log_warn("  [PHASE 2]  AMR 순회 + 책장 꽂기 시작")
    carb.log_warn("  Home → A(red) → B(yellow) → C(blue) → Home")
    carb.log_warn("="*60)

    # ── Home 이동: 책이 로봇 자식이므로 같이 이동 ────────────
    x, y, z, oz = P2_WAYPOINTS["Home"]
    carb.log_warn(f"\n[P2] [1/{len(P2_SEQUENCE)}] ▶  Home  ({x}, {y}, {z})  orient_z={oz}°")
    teleport_robot(world, robot_prim, x, y, z, oz)
    dwell(world, P2_DWELL_TIME)
    # Home 도착 후에도 책은 robot 자식 상태 유지 (아직 /Root로 복귀 X)

    # 나머지 경로 추적용 딕셔너리 (robot 자식 경로 유지)
    current_book_paths = dict(robot_book_paths)

    # A → B → C → Home 순회
    for idx, wp_name in enumerate(P2_SEQUENCE[1:], start=2):
        x, y, z, oz = P2_WAYPOINTS[wp_name]
        carb.log_warn(f"\n[P2] [{idx}/{len(P2_SEQUENCE)}] ▶  {wp_name}  "
                      f"({x:.4f}, {y:.4f}, {z:.5f})  orient_z={oz}°")

        teleport_robot(world, robot_prim, x, y, z, oz)
        dwell(world, P2_DWELL_TIME)

        if wp_name in P2_SHELF_STOPS:
            target_color = P2_PLACE_COLOR[wp_name]

            # ★ 해당 책 1권만 robot 자식에서 해제 → /Root 로 복귀
            carb.log_warn(f"[P2] [{wp_name}] {target_color} 책 robot 자식 해제")
            restored_path = detach_single_book_from_robot(
                stage, target_color, current_book_paths, restore_parent="/Root"
            )
            current_book_paths[target_color] = restored_path
            dwell(world, 0.5)   # 위치 안정화

            carb.log_warn(f"[P2] ★ [{wp_name}] 도착 → 책꽂기 시퀀스 시작")
            shelf = amr2shelf(
                world=world, stage=stage, art=art,
                place=wp_name,
                book_paths=current_book_paths,
            )
            shelf.run_sequence()
            current_book_paths[shelf.target_color] = shelf.book_path
            carb.log_warn(f"[P2] ★ [{wp_name}] 책꽂기 완료")

    carb.log_warn("\n" + "="*60)
    carb.log_warn("  [PHASE 2]  전체 완료! 창을 닫으면 종료됩니다.")
    carb.log_warn("="*60)

    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        carb.log_error(f"[FATAL] {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()