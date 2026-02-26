#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import traceback
import numpy as np
import cv2
from collections import defaultdict

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ROS2 bridge extension 수동 활성화
import omni.kit.app
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

import carb
import omni.usd
import omni.timeline

from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


# ================================
# 설정 상수
# ================================
ENV_USD_PATH = "/home/rokey/Rokey6-A3-SimsFactory/project/environment_carter.usd"

ROBOT_ARTICULATION_ROOT = "/Root/robot/robot"
EE_LINK_PATH            = "/Root/robot/robot/nova_carter/ur10/ee_link"
CAMERA_PRIM_PATH        = "/Root/robot/robot/nova_carter/ur10/ee_link/short_gripper/Camera"

GRAPH_UR10 = None

BOOK_SPAWN_POS = (-12.7, 5.2, 1.5)
BOOK_SCALE     = (0.15, 0.25, 0.04)

BOOKS = {
    "red": {
        "path": "/Root/red_book",
        "color": (1.0, 0.0, 0.0),
        "hsv_ranges": [
            (np.array([0,   30,  30]), np.array([10,  255, 255])),
            (np.array([170, 30,  30]), np.array([179, 255, 255])),
        ],
    },
    "blue": {
        "path": "/Root/blue_book",
        "color": (0.0, 0.0, 1.0),
        "hsv_ranges": [
            (np.array([100, 30, 30]), np.array([130, 255, 255])),
        ],
    },
    "yellow": {
        "path": "/Root/yellow_book",
        "color": (1.0, 1.0, 0.0),
        "hsv_ranges": [
            (np.array([15, 30, 30]), np.array([35, 255, 255])),
        ],
    },
}

BOOK_SPAWN_INTERVAL_S = 1.0

START_DELAY_S   = 4.0
HOLD_APPROACH_S = 2.0
HOLD_GRASP_S    = 2.0
HOLD_LIFT_S     = 2.0
HOLD_MOVE_S     = 2.0
HOLD_PLACE_S    = 3.0

# 색상 탐지 대기 시간 (초) - APPROACH 후 카메라 안정화 대기
COLOR_DETECT_WAIT_S = 3.0

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

POSE_READY_DEG        = [0,    -90.0,  -90.0,  -90,   90.0, 0.0]
POSE_APPROACH_DEG     = [115,  -90.0,  -90.0,  -90,   90.0, 0.0]
POSE_GRASP_DEG        = [115, -123.0,  -87.0,  -60.0, 90.0, 0.0]
POSE_LIFT_DEG         = [115,  -90.0,  -90.0,  -90,   90.0, 0.0]
POSE_MOVE_DEG         = POSE_READY_DEG.copy()
POSE_PLACE_RED_DEG    = [5,   -120.0,  -90.0,  -60,   90.0, 0.0]
POSE_PLACE_YELLOW_DEG = [7,   -103.0, -122.0,  -45,   90.0, 0.0]
POSE_PLACE_BLUE_DEG   = [10,   -70,   -140.0,  -55,   90.0, 0.0]

SUCTION_CUP_NAME             = "suction_cup"
ATTACH_OFFSET_IN_CUP_FRAME   = np.array([0.0, 0.0, 0.01], dtype=np.float64)
ATTACH_MATCH_CUP_ORIENTATION = False


# ================================
# place2amr 클래스
# ================================
class place2amr:
    def __init__(
        self,
        env_usd_path,
        robot_articulation_root,
        ee_link_path,
        camera_prim_path,
        graph_ur10,
        books,
        book_spawn_pos,
        book_scale,
        book_spawn_interval_s,
        joint_names,
        poses,
        suction_cup_name,
        attach_offset_in_cup_frame,
        attach_match_cup_orientation,
        start_delay_s,
        hold_approach_s,
        hold_grasp_s,
        hold_lift_s,
        hold_move_s,
        hold_place_s,
        color_detect_wait_s=3.0,
        debug_dir="/tmp/camera_debug",
    ):
        self.env_usd_path            = env_usd_path
        self.robot_articulation_root = robot_articulation_root
        self.ee_link_path            = ee_link_path
        self.camera_prim_path        = camera_prim_path
        self.graph_ur10              = graph_ur10

        self.books                 = books
        self.book_spawn_pos        = book_spawn_pos
        self.book_scale            = book_scale
        self.book_spawn_interval_s = float(book_spawn_interval_s)

        self.joint_names = joint_names
        self.poses       = poses

        self.suction_cup_name             = suction_cup_name
        self.attach_offset_in_cup_frame   = attach_offset_in_cup_frame.astype(np.float64).copy()
        self.attach_match_cup_orientation = bool(attach_match_cup_orientation)

        self.start_delay_s      = float(start_delay_s)
        self.hold_approach_s    = float(hold_approach_s)
        self.hold_grasp_s       = float(hold_grasp_s)
        self.hold_lift_s        = float(hold_lift_s)
        self.hold_move_s        = float(hold_move_s)
        self.hold_place_s       = float(hold_place_s)
        self.color_detect_wait_s = float(color_detect_wait_s)

        self.debug_dir = debug_dir

        self.stage            = None
        self.world            = None
        self.robot_art        = None
        self.ur10_indices     = None
        self.suction_cup_path = None

    # ----------------------------
    # 기본 유틸
    # ----------------------------
    @staticmethod
    def deg2rad(deg_array):
        return np.deg2rad(np.array(deg_array, dtype=np.float64))

    @staticmethod
    def get_stage() -> Usd.Stage:
        return omni.usd.get_context().get_stage()

    @staticmethod
    def prim_exists(stage: Usd.Stage, prim_path: str) -> bool:
        prim = stage.GetPrimAtPath(prim_path)
        return bool(prim and prim.IsValid())

    @staticmethod
    def wxyz_to_quatf(q_wxyz):
        w, x, y, z = q_wxyz
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

    @staticmethod
    def quat_wxyz_to_rotmat(q_wxyz):
        w, x, y, z = [float(v) for v in q_wxyz]
        n = (w*w + x*x + y*y + z*z) ** 0.5
        if n < 1e-12:
            return np.eye(3, dtype=np.float64)
        w, x, y, z = w/n, x/n, y/n, z/n
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
            [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
            [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
        ], dtype=np.float64)

    @staticmethod
    def get_world_pose(stage: Usd.Stage, prim_path: str):
        prim = stage.GetPrimAtPath(prim_path)
        xf   = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t    = xf.ExtractTranslation()
        rot  = xf.ExtractRotationQuat()
        pos  = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64)
        quat_wxyz = (
            float(rot.GetReal()),
            float(rot.GetImaginary()[0]),
            float(rot.GetImaginary()[1]),
            float(rot.GetImaginary()[2]),
        )
        return pos, quat_wxyz

    @staticmethod
    def teleport_prim_to_pose(stage: Usd.Stage, prim_path: str, pos_xyz, quat_wxyz):
        prim  = stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(prim)
        ops   = xform.GetOrderedXformOps()

        op_t = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
        op_r = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient), None)

        if op_t is None or op_r is None:
            xform.ClearXformOpOrder()
            op_t = xform.AddTranslateOp()
            op_r = xform.AddOrientOp()
            xform.AddScaleOp()

        op_t.Set(Gf.Vec3d(float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])))
        op_r.Set(place2amr.wxyz_to_quatf(quat_wxyz))

    # ----------------------------
    # ActionGraph 상태 디버그
    # ----------------------------
    def _check_ag_status(self, label: str):
        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "OmniGraph":
                status = "ACTIVE" if prim.IsActive() else "INACTIVE"
                carb.log_warn(f"[AG {label}] {status}  {prim.GetPath()}")

    # ----------------------------
    # suction_cup prim 탐색
    # ----------------------------
    def resolve_suction_cup_path(self, stage: Usd.Stage) -> str:
        direct = self.ee_link_path + "/" + self.suction_cup_name
        if self.prim_exists(stage, direct):
            carb.log_warn(f"[CUP] found by direct path: {direct}")
            return direct

        ee = stage.GetPrimAtPath(self.ee_link_path)
        if ee and ee.IsValid():
            found = [str(p.GetPath()) for p in Usd.PrimRange(ee)
                     if p.GetName() == self.suction_cup_name]
            if found:
                found.sort(key=len)
                carb.log_warn(f"[CUP] found under ee_link: {found[0]}")
                return found[0]

        global_found = [str(p.GetPath()) for p in stage.Traverse()
                        if p.GetName() == self.suction_cup_name]
        if not global_found:
            raise RuntimeError("suction_cup prim not found anywhere.")

        prefer = [p for p in global_found if self.ee_link_path in p]
        pick   = prefer[0] if prefer else global_found[0]
        carb.log_warn(f"[CUP] found by global scan: {pick}")
        return pick

    # ----------------------------
    # 카메라 / 색상 처리
    # ----------------------------
    def get_camera_image(self, stage, camera_prim_path):
        try:
            from omni.isaac.sensor import Camera
            camera_prim = stage.GetPrimAtPath(camera_prim_path)
            if not camera_prim or not camera_prim.IsValid():
                carb.log_error(f"[CAMERA] Camera prim not found: {camera_prim_path}")
                return None
            camera = Camera(prim_path=camera_prim_path)
            camera.initialize()
            rgb_data = camera.get_rgb()
            if rgb_data is None or rgb_data.size == 0:
                carb.log_warn("[CAMERA] Failed to get RGB data")
                return None
            carb.log_warn(f"[CAMERA] shape={rgb_data.shape}, range=[{rgb_data.min():.2f},{rgb_data.max():.2f}]")
            if rgb_data.max() <= 1.0:
                rgb_array = (rgb_data[:, :, :3] * 255).astype(np.uint8)
            else:
                rgb_array = rgb_data[:, :, :3].astype(np.uint8)
            return cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
        except Exception as e:
            carb.log_warn(f"[CAMERA] Error: {e}\n{traceback.format_exc()}")
            return None

    def save_camera_image_with_masks(self, image, book_name):
        try:
            os.makedirs(self.debug_dir, exist_ok=True)
            cv2.imwrite(f"{self.debug_dir}/{book_name}_00_original.png", image)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            for idx, (book_color, book_info) in enumerate(self.books.items()):
                combined_mask = None
                for lo, hi in book_info["hsv_ranges"]:
                    m = cv2.inRange(hsv, lo, hi)
                    combined_mask = m if combined_mask is None else cv2.bitwise_or(combined_mask, m)
                k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, k)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, k)
                cv2.imwrite(f"{self.debug_dir}/{book_name}_{idx+1:02d}_{book_color}_mask.png", combined_mask)
                carb.log_warn(f"[DEBUG] {book_color} mask: {cv2.countNonZero(combined_mask)} px")
        except Exception as e:
            carb.log_warn(f"[DEBUG] 이미지 저장 실패: {e}")

    def detect_book_color(self, image):
        if image is None or image.size == 0:
            return "unknown"
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_count = defaultdict(int)
        for name, info in self.books.items():
            combined_mask = None
            for lo, hi in info["hsv_ranges"]:
                m = cv2.inRange(hsv, lo, hi)
                combined_mask = m if combined_mask is None else cv2.bitwise_or(combined_mask, m)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, k)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, k)
            color_count[name] = cv2.countNonZero(combined_mask)
            carb.log_warn(f"[COLOR] {name:7s}: {color_count[name]:6d} px")
        if color_count:
            best = max(color_count, key=color_count.get)
            if color_count[best] > 200:
                carb.log_warn(f"[COLOR] ✓ detected: {best} ({color_count[best]} px)")
                return best
        carb.log_warn("[COLOR] ✗ 탐지 실패")
        return "unknown"

    def capture_and_detect_color(self, stage, camera_prim_path, book_name):
        image = self.get_camera_image(stage, camera_prim_path)
        if image is None:
            return "unknown"
        self.save_camera_image_with_masks(image, book_name)
        return self.detect_book_color(image)

    # ----------------------------
    # Book 생성
    # ----------------------------
    def create_book(self, stage, book_name):
        book_path = self.books[book_name]["path"]
        color     = self.books[book_name]["color"]

        if self.prim_exists(stage, book_path):
            stage.RemovePrim(stage.GetPrimAtPath(book_path))

        cube = UsdGeom.Cube.Define(stage, book_path)
        cube.CreateSizeAttr(1.0)

        prim  = stage.GetPrimAtPath(book_path)
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*self.book_spawn_pos))
        xform.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        xform.AddScaleOp().Set(Gf.Vec3f(*self.book_scale))

        prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([color])
        try: UsdPhysics.CollisionAPI.Apply(prim)
        except Exception: pass
        try: UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
        except Exception: pass
        try: UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.2)
        except Exception: pass

        carb.log_warn(f"[BOOK] {book_name} spawned at {book_path}")

    # ----------------------------
    # 조인트 stiffness 보정
    # ----------------------------
    def fix_ur10_stiffness(self, stage):
        for prim in stage.Traverse():
            if prim.GetName() in self.joint_names:
                for drive_name in ["angular", "rotX", "rotY", "rotZ"]:
                    drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
                    if drive:
                        drive.GetStiffnessAttr().Set(1e7)
                        drive.GetDampingAttr().Set(1e6)

    # ----------------------------
    # Pick & Place
    # ----------------------------
    def pick_and_place_book(self, book_name, book_path):
        stage            = self.stage
        world            = self.world
        robot_art        = self.robot_art
        ur10_indices     = self.ur10_indices
        suction_cup_path = self.suction_cup_path

        carb.log_warn(f"[TASK] {book_name} pick & place 시작")

        place_poses = {
            "red":    self.poses["place_red"],
            "blue":   self.poses["place_blue"],
            "yellow": self.poses["place_yellow"],
        }

        current_action     = None
        attached_quat_wxyz = None
        attached           = False

        def move_robot_deg(q_deg):
            nonlocal current_action
            q_rad = self.deg2rad(q_deg)
            current_action = ArticulationAction(
                joint_positions=q_rad,
                joint_indices=np.array(ur10_indices, dtype=np.int32),
            )

        def hold_seconds(seconds, is_attached):
            nonlocal attached_quat_wxyz
            t0 = time.time()
            while simulation_app.is_running() and (time.time() - t0) < seconds:
                if current_action is not None:
                    robot_art.apply_action(current_action)
                world.step(render=True)
                if is_attached:
                    cup_pos, cup_quat = self.get_world_pose(stage, suction_cup_path)
                    offset_world = self.quat_wxyz_to_rotmat(cup_quat) @ self.attach_offset_in_cup_frame
                    book_pos = cup_pos + offset_world
                    if attached_quat_wxyz is None:
                        _, attached_quat_wxyz = self.get_world_pose(stage, book_path)
                    book_quat = attached_quat_wxyz if not self.attach_match_cup_orientation else cup_quat
                    self.teleport_prim_to_pose(stage, book_path, book_pos, book_quat)

        carb.log_warn(f">> {book_name} 0. READY")
        move_robot_deg(self.poses["ready"])
        hold_seconds(self.hold_approach_s, attached)

        carb.log_warn(f">> {book_name} 1. APPROACH")
        move_robot_deg(self.poses["approach"])
        hold_seconds(self.hold_approach_s, attached)

        # 색상 탐지: color_detect_wait_s 동안 1초마다 시도, 성공하면 즉시 종료
        carb.log_warn(f">> {book_name} 색상 탐지 시작 (최대 {self.color_detect_wait_s:.0f}초)")
        detected_color    = "unknown"
        debug_start       = time.time()
        detection_attempt = 0
        while simulation_app.is_running() and (time.time() - debug_start) < self.color_detect_wait_s:
            if current_action is not None:
                robot_art.apply_action(current_action)
            world.step(render=True)
            elapsed = int(time.time() - debug_start)
            if elapsed != detection_attempt:
                detection_attempt = elapsed
                carb.log_warn(f">> {book_name} [COLOR] [{elapsed}초] 탐지 시도...")
                img = self.get_camera_image(stage, self.camera_prim_path)
                if img is not None:
                    result = self.detect_book_color(img)
                    if result != "unknown":
                        detected_color = result
                        carb.log_warn(f">> {book_name} [COLOR] 조기 탐지 성공: {detected_color} → 대기 종료")
                        break

        # 탐지 실패 시 최종 1회 재시도
        if detected_color == "unknown":
            carb.log_warn(f">> {book_name} [COLOR] 최종 재시도...")
            detected_color = self.capture_and_detect_color(stage, self.camera_prim_path, book_name)
        carb.log_warn(f">> {book_name} 최종 탐지 색상: {detected_color}")

        carb.log_warn(f">> {book_name} 2. GRASP")
        move_robot_deg(self.poses["grasp"])
        hold_seconds(self.hold_grasp_s, attached)

        attached = True
        attached_quat_wxyz = None

        carb.log_warn(f">> {book_name} 3. LIFT")
        move_robot_deg(self.poses["lift"])
        hold_seconds(self.hold_lift_s, attached)

        carb.log_warn(f">> {book_name} 4. MOVE")
        move_robot_deg(self.poses["move"])
        hold_seconds(self.hold_move_s, attached)

        place_pose = place_poses.get(detected_color, self.poses["place_blue"])
        carb.log_warn(f">> {book_name} 5. PLACE → {detected_color}")
        move_robot_deg(place_pose)
        hold_seconds(self.hold_place_s, attached)

        attached = False
        hold_seconds(1.0, attached)

        carb.log_warn(f">> {book_name} 6. READY 복귀")
        move_robot_deg(self.poses["ready"])
        hold_seconds(self.hold_place_s, attached)

        carb.log_warn(f"[TASK] {book_name} 완료 (detected: {detected_color})")

    # ----------------------------
    # 초기화 / 실행
    # ----------------------------
    def _open_stage_and_init(self):
        if not os.path.isfile(self.env_usd_path):
            raise FileNotFoundError(f"USD not found: {self.env_usd_path}")

        from isaacsim.core.utils.stage import open_stage
        carb.log_warn(f"[STAGE] open_stage: {self.env_usd_path}")
        open_stage(self.env_usd_path)
        simulation_app.update()

        self.stage = self.get_stage()
        self._check_ag_status("1_after_open_stage")

        self.fix_ur10_stiffness(self.stage)
        self._check_ag_status("2_after_fix_stiffness")

        self.world = World(physics_dt=1/60, rendering_dt=1/60)
        self._check_ag_status("3_after_world_create")

        self.robot_art = self.world.scene.add(
            SingleArticulation(
                prim_path=self.robot_articulation_root,
                name="carter_ur10",
            )
        )
        self._check_ag_status("4_after_scene_add")

        self.world.reset()
        self._check_ag_status("5_after_world_reset")

        self.suction_cup_path = self.resolve_suction_cup_path(self.stage)
        carb.log_warn(f"[CUP] suction_cup path: {self.suction_cup_path}")

        self.ur10_indices = [
            self.robot_art.get_dof_index(n) for n in self.joint_names
        ]
        carb.log_warn(f"[INIT] ur10 DOF indices: {list(zip(self.joint_names, self.ur10_indices))}")

        omni.timeline.get_timeline_interface().play()
        self._check_ag_status("6_after_timeline_play")

        carb.log_warn(f"[WAIT] 초기 안정화 {self.start_delay_s:.1f}초 대기...")
        for _ in range(int(self.start_delay_s * 60)):
            self.world.step(render=True)
        self._check_ag_status("7_after_wait")

    def run(self):
        self._open_stage_and_init()
        carb.log_warn("[RUN] Pick & Place 시작!")

        books_order = ["red", "blue", "yellow"]
        for idx, book_name in enumerate(books_order):
            book_path = self.books[book_name]["path"]

            if idx > 0:
                carb.log_warn(f"[WAIT] 다음 책 spawn 대기 ({self.book_spawn_interval_s:.1f}초)...")
                for _ in range(int(self.book_spawn_interval_s * 60)):
                    self.world.step(render=True)

            self.create_book(self.stage, book_name)
            simulation_app.update()
            self.pick_and_place_book(book_name=book_name, book_path=book_path)

        carb.log_warn("[DONE] 모든 책 작업 완료. 시뮬레이션 유지...")
        while simulation_app.is_running():
            self.world.step(render=True)


# ================================
# 엔트리포인트
# ================================
if __name__ == "__main__":
    poses = {
        "ready":        POSE_READY_DEG,
        "approach":     POSE_APPROACH_DEG,
        "grasp":        POSE_GRASP_DEG,
        "lift":         POSE_LIFT_DEG,
        "move":         POSE_MOVE_DEG,
        "place_red":    POSE_PLACE_RED_DEG,
        "place_yellow": POSE_PLACE_YELLOW_DEG,
        "place_blue":   POSE_PLACE_BLUE_DEG,
    }

    app = place2amr(
        env_usd_path=ENV_USD_PATH,
        robot_articulation_root=ROBOT_ARTICULATION_ROOT,
        ee_link_path=EE_LINK_PATH,
        camera_prim_path=CAMERA_PRIM_PATH,
        graph_ur10=GRAPH_UR10,
        books=BOOKS,
        book_spawn_pos=BOOK_SPAWN_POS,
        book_scale=BOOK_SCALE,
        book_spawn_interval_s=BOOK_SPAWN_INTERVAL_S,
        joint_names=JOINT_NAMES,
        poses=poses,
        suction_cup_name=SUCTION_CUP_NAME,
        attach_offset_in_cup_frame=ATTACH_OFFSET_IN_CUP_FRAME,
        attach_match_cup_orientation=ATTACH_MATCH_CUP_ORIENTATION,
        start_delay_s=START_DELAY_S,
        hold_approach_s=HOLD_APPROACH_S,
        hold_grasp_s=HOLD_GRASP_S,
        hold_lift_s=HOLD_LIFT_S,
        hold_move_s=HOLD_MOVE_S,
        hold_place_s=HOLD_PLACE_S,
        color_detect_wait_s=COLOR_DETECT_WAIT_S,
        debug_dir="/tmp/camera_debug",
    )

    try:
        app.run()
    except Exception as e:
        carb.log_error(f"[FATAL] {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()