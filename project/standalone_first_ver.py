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

import carb
import omni.usd
import omni.timeline

from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics

from isaacsim.core.api import World
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.types import ArticulationAction


# ================================
# 설정 상수 (클래스 밖)
# ================================
ENV_USD_PATH = "/home/kyb/Rokey6-A3-SimsFactory/project/environment_carter.usd"

ROBOT_ARTICULATION_ROOT = "/Root/robot/nova_carter"
UR10_PRIM_PATH = "/Root/robot/nova_carter/ur10"
EE_LINK_PATH = "/Root/robot/nova_carter/ur10/ee_link"
CAMERA_PRIM_PATH = "/Root/robot/nova_carter/ur10/ee_link/short_gripper/Camera"

GRAPH_UR10 = "/Root/robot/nova_carter/ur10/Graphs/Graphs/Position_Controller"

BOOK_SPAWN_POS = (-12.7, 5.2, 1.5)
BOOK_SCALE = (0.15, 0.25, 0.04)

BOOKS = {
    "red": {
        "path": "/Root/red_book",
        "color": (1.0, 0.0, 0.0),
        "hsv_ranges": [
            (np.array([0, 30, 30]), np.array([10, 255, 255])),
            (np.array([170, 30, 30]), np.array([179, 255, 255])),
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

START_DELAY_S = 4.0
HOLD_APPROACH_S = 2.0
HOLD_GRASP_S = 2.0
HOLD_LIFT_S = 2.0
HOLD_MOVE_S = 2.0
HOLD_PLACE_S = 3.0

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

POSE_READY_DEG = [0, -90.0, -90.0, -90, 90.0, 0.0]
POSE_APPROACH_DEG = [115, -90.0, -90.0, -90, 90.0, 0.0]
POSE_GRASP_DEG = [115, -123.0, -87.0, -60.0, 90.0, 0.0]
POSE_LIFT_DEG = [115, -90.0, -90.0, -90, 90.0, 0.0]
POSE_MOVE_DEG = POSE_READY_DEG.copy()
POSE_PLACE_RED_DEG = [5, -120.0, -90.0, -60, 90.0, 0.0]
POSE_PLACE_YELLOW_DEG = [7, -103.0, -122.0, -45, 90.0, 0.0]
POSE_PLACE_BLUE_DEG = [10, -70, -140.0, -55, 90.0, 0.0]

SUCTION_CUP_NAME = "suction_cup"
ATTACH_OFFSET_IN_CUP_FRAME = np.array([0.0, 0.0, 0.01], dtype=np.float64)
ATTACH_MATCH_CUP_ORIENTATION = False


# ================================
# place2amr 클래스
# ================================
class place2amr:
    def __init__(
        self,
        env_usd_path: str,
        robot_articulation_root: str,
        ee_link_path: str,
        camera_prim_path: str,
        graph_ur10: str,
        books: dict,
        book_spawn_pos: tuple,
        book_scale: tuple,
        book_spawn_interval_s: float,
        joint_names: list,
        poses: dict,
        suction_cup_name: str,
        attach_offset_in_cup_frame: np.ndarray,
        attach_match_cup_orientation: bool,
        start_delay_s: float,
        hold_approach_s: float,
        hold_grasp_s: float,
        hold_lift_s: float,
        hold_move_s: float,
        hold_place_s: float,
        debug_dir: str = "/tmp/camera_debug",
    ):
        self.env_usd_path = env_usd_path
        self.robot_articulation_root = robot_articulation_root
        self.ee_link_path = ee_link_path
        self.camera_prim_path = camera_prim_path
        self.graph_ur10 = graph_ur10

        self.books = books
        self.book_spawn_pos = book_spawn_pos
        self.book_scale = book_scale
        self.book_spawn_interval_s = float(book_spawn_interval_s)

        self.joint_names = joint_names
        self.poses = poses

        self.suction_cup_name = suction_cup_name
        self.attach_offset_in_cup_frame = attach_offset_in_cup_frame.astype(np.float64).copy()
        self.attach_match_cup_orientation = bool(attach_match_cup_orientation)

        self.start_delay_s = float(start_delay_s)
        self.hold_approach_s = float(hold_approach_s)
        self.hold_grasp_s = float(hold_grasp_s)
        self.hold_lift_s = float(hold_lift_s)
        self.hold_move_s = float(hold_move_s)
        self.hold_place_s = float(hold_place_s)

        self.debug_dir = debug_dir

        self.stage = None
        self.world = None
        self.ur10 = None
        self.ur10_indices = None
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

    @staticmethod
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

    @staticmethod
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
        op_r.Set(place2amr.wxyz_to_quatf(quat_wxyz))

    # ----------------------------
    # suction_cup prim 찾기
    # ----------------------------
    def resolve_suction_cup_path(self, stage: Usd.Stage) -> str:
        direct = self.ee_link_path + "/" + self.suction_cup_name
        if self.prim_exists(stage, direct):
            carb.log_warn(f"[CUP] found by direct path: {direct}")
            return direct

        ee = stage.GetPrimAtPath(self.ee_link_path)
        if ee and ee.IsValid():
            found = []
            for p in Usd.PrimRange(ee):
                if p.GetName() == self.suction_cup_name:
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
            if p.GetName() == self.suction_cup_name:
                global_found.append(str(p.GetPath()))

        if not global_found:
            raise RuntimeError("suction_cup prim not found anywhere.")

        prefer = [p for p in global_found if self.ee_link_path in p]
        pick = (prefer[0] if prefer else global_found[0])

        carb.log_warn(f"[CUP] found by global scan, choose: {pick}")
        return pick

    # ----------------------------
    # 카메라/색상 처리
    # ----------------------------
    def get_camera_image(self, stage: Usd.Stage, camera_prim_path: str) -> np.ndarray:
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

            carb.log_warn(
                f"[CAMERA] RGB data shape: {rgb_data.shape}, dtype: {rgb_data.dtype}, "
                f"range: [{rgb_data.min():.2f}, {rgb_data.max():.2f}]"
            )

            if rgb_data.max() <= 1.0:
                rgb_array = (rgb_data[:, :, :3] * 255).astype(np.uint8)
                carb.log_warn("[CAMERA] Normalized from 0~1 to 0~255")
            else:
                rgb_array = rgb_data[:, :, :3].astype(np.uint8)

            bgr_image = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            carb.log_warn("[CAMERA] ✓ 이미지 처리 완료 (RGB → BGR → uint8)")
            return bgr_image

        except Exception as e:
            carb.log_warn(f"[CAMERA] Error getting camera image: {e}")
            carb.log_warn(traceback.format_exc())
            return None

    def save_camera_image_with_masks(self, image: np.ndarray, book_name: str) -> None:
        try:
            os.makedirs(self.debug_dir, exist_ok=True)

            cv2.imwrite(f"{self.debug_dir}/{book_name}_00_original.png", image)
            carb.log_warn(f"[DEBUG] 원본 이미지 저장: {self.debug_dir}/{book_name}_00_original.png")

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            for idx, (book_color, book_info) in enumerate(self.books.items()):
                hsv_ranges = book_info["hsv_ranges"]

                combined_mask = None
                for hsv_lower, hsv_upper in hsv_ranges:
                    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
                    combined_mask = mask if combined_mask is None else cv2.bitwise_or(combined_mask, mask)

                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

                cv2.imwrite(
                    f"{self.debug_dir}/{book_name}_{idx+1:02d}_{book_color}_mask.png",
                    combined_mask
                )
                pixel_count = cv2.countNonZero(combined_mask)
                carb.log_warn(f"[DEBUG] {book_color} 마스크: {pixel_count} 픽셀")

            carb.log_warn("[DEBUG] ============================================")
            carb.log_warn(f"[DEBUG] 이미지 저장 완료: {self.debug_dir}/")
            carb.log_warn("[DEBUG] 확인:")
            carb.log_warn(f"[DEBUG]   eog {self.debug_dir}/ &  또는  ls -la {self.debug_dir}/")
            carb.log_warn("[DEBUG] ============================================")

        except Exception as e:
            carb.log_warn(f"[DEBUG] 이미지 저장 실패: {e}")

    def detect_book_color(self, image: np.ndarray) -> str:
        if image is None or image.size == 0:
            carb.log_warn("[COLOR] Invalid image")
            return "unknown"

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_count = defaultdict(int)

        carb.log_warn("[COLOR] ===== 색상 탐지 상세 정보 (HSV 기반) =====")
        carb.log_warn(
            f"[COLOR] HSV 이미지 범위: H[{hsv[:,:,0].min()}-{hsv[:,:,0].max()}], "
            f"S[{hsv[:,:,1].min()}-{hsv[:,:,1].max()}], "
            f"V[{hsv[:,:,2].min()}-{hsv[:,:,2].max()}]"
        )

        for book_name, book_info in self.books.items():
            hsv_ranges = book_info["hsv_ranges"]
            combined_mask = None

            for hsv_lower, hsv_upper in hsv_ranges:
                mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
                combined_mask = mask if combined_mask is None else cv2.bitwise_or(combined_mask, mask)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

            pixel_count = cv2.countNonZero(combined_mask)
            color_count[book_name] = pixel_count

            range_str = ", ".join([f"H[{h_lower[0]}-{h_upper[0]}]" for h_lower, h_upper in hsv_ranges])
            carb.log_warn(f"[COLOR]   {book_name:7s} ({range_str:20s}) : {pixel_count:6d} pixels")

        carb.log_warn("[COLOR] ==================================")

        if color_count:
            detected_color = max(color_count, key=color_count.get)
            max_pixels = color_count[detected_color]

            if max_pixels > 200:
                carb.log_warn(f"[COLOR] ✓ 탐지 완료: {detected_color} ({max_pixels} 픽셀)")
                return detected_color

            carb.log_warn(f"[COLOR] ✗ 탐지 실패: 최대값 {max_pixels} < 200 (임계값)")
            carb.log_warn("[COLOR] 디버깅: 각 색상의 픽셀 분포:")
            for color, count in sorted(color_count.items(), key=lambda x: x[1], reverse=True):
                carb.log_warn(f"[COLOR]   {color}: {count} pixels")

        carb.log_warn("[COLOR] ✗ 명확한 색상을 찾을 수 없음")
        return "unknown"

    def capture_and_detect_color(self, stage: Usd.Stage, camera_prim_path: str, book_name: str) -> str:
        carb.log_warn("[CAPTURE] 이미지 촬영 시작...")
        image = self.get_camera_image(stage, camera_prim_path)

        if image is None:
            carb.log_warn("[CAPTURE] ✗ 이미지 촬영 실패")
            return "unknown"

        carb.log_warn(f"[CAPTURE] ✓ 이미지 촬영 완료 ({image.shape[1]}x{image.shape[0]})")
        self.save_camera_image_with_masks(image, book_name)
        detected_color = self.detect_book_color(image)
        carb.log_warn(f"[CAPTURE] 최종 결과: {detected_color}")
        return detected_color

    # ----------------------------
    # Book 생성/물리
    # ----------------------------
    def create_book(self, stage: Usd.Stage, book_name: str) -> None:
        book_path = self.books[book_name]["path"]
        color = self.books[book_name]["color"]

        if self.prim_exists(stage, book_path):
            prim = stage.GetPrimAtPath(book_path)
            stage.RemovePrim(prim)

        cube = UsdGeom.Cube.Define(stage, book_path)
        cube.CreateSizeAttr(1.0)

        prim = stage.GetPrimAtPath(book_path)
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()

        op_t = xform.AddTranslateOp()
        op_r = xform.AddOrientOp()
        op_s = xform.AddScaleOp()

        op_t.Set(Gf.Vec3d(*self.book_spawn_pos))
        op_r.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        op_s.Set(Gf.Vec3f(*self.book_scale))

        prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([color])

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

    # ----------------------------
    # 조인트 처짐 방지
    # ----------------------------
    def fix_ur10_stiffness(self, stage: Usd.Stage):
        for prim in stage.Traverse():
            if prim.GetName() in self.joint_names:
                for drive_name in ["angular", "rotX", "rotY", "rotZ"]:
                    drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
                    if drive:
                        drive.GetStiffnessAttr().Set(1e7)
                        drive.GetDampingAttr().Set(1e6)

    # ----------------------------
    # Pick & Place 1개 책
    # ----------------------------
    def pick_and_place_book(self, book_name: str, book_path: str):
        stage = self.stage
        world = self.world
        ur10 = self.ur10
        ur10_indices = self.ur10_indices
        suction_cup_path = self.suction_cup_path

        carb.log_warn(f"[TASK] {book_name} pick & place 시작")

        place_poses = {
            "red": self.poses["place_red"],
            "blue": self.poses["place_blue"],
            "yellow": self.poses["place_yellow"],
        }

        current_action = None
        attached_quat_wxyz = None
        detected_color = "unknown"
        attached = False

        def move_robot_deg(q_deg):
            nonlocal current_action
            q_rad = self.deg2rad(q_deg)
            current_action = ArticulationAction(joint_positions=q_rad, joint_indices=ur10_indices)

        def hold_seconds(seconds: float, is_attached: bool):
            nonlocal attached_quat_wxyz

            t0 = time.time()
            while simulation_app.is_running() and (time.time() - t0) < seconds:
                if current_action is not None:
                    ur10.apply_action(current_action)

                world.step(render=True)

                if is_attached:
                    cup_pos, cup_quat = self.get_world_pose(stage, suction_cup_path)

                    R = self.quat_wxyz_to_rotmat(cup_quat)
                    offset_world = R @ self.attach_offset_in_cup_frame
                    book_pos = cup_pos + offset_world

                    if attached_quat_wxyz is None:
                        _, attached_quat_wxyz = self.get_world_pose(stage, book_path)

                    book_quat = attached_quat_wxyz if not self.attach_match_cup_orientation else cup_quat
                    self.teleport_prim_to_pose(stage, book_path, book_pos, book_quat)

        carb.log_warn(f">> {book_name} 0. READY (deg)")
        move_robot_deg(self.poses["ready"])
        hold_seconds(self.hold_approach_s, attached)

        carb.log_warn(f">> {book_name} 1-2. APPROACH (deg)")
        move_robot_deg(self.poses["approach"])
        hold_seconds(self.hold_approach_s, attached)

        move_robot_deg(self.poses["approach"])
        debug_start = time.time()
        detection_attempt = 0

        while simulation_app.is_running() and (time.time() - debug_start) < 3.0:
            if current_action is not None:
                ur10.apply_action(current_action)

            world.step(render=True)

            if int(time.time() - debug_start) != detection_attempt:
                detection_attempt = int(time.time() - debug_start)
                carb.log_warn(f">> {book_name} [DEBUG] [{detection_attempt}초] 색상 탐지 시도...")
                test_image = self.get_camera_image(stage, self.camera_prim_path)
                if test_image is not None:
                    self.detect_book_color(test_image)
                carb.log_warn(f">> {book_name} [DEBUG] [{detection_attempt}초] 완료")

        carb.log_warn(f">> {book_name} [DEBUG] 시야각 조정 완료. 최종 색상 탐지 수행...")
        detected_color = self.capture_and_detect_color(stage, self.camera_prim_path, book_name)
        carb.log_warn(f">> {book_name} [DEBUG] 최종 탐지 색상: {detected_color}")

        carb.log_warn(f">> {book_name} 2. GRASP (deg)")
        move_robot_deg(self.poses["grasp"])
        hold_seconds(self.hold_grasp_s, attached)

        attached = True
        attached_quat_wxyz = None

        carb.log_warn(f">> {book_name} 3. LIFT (deg)")
        move_robot_deg(self.poses["lift"])
        hold_seconds(self.hold_lift_s, attached)

        carb.log_warn(f">> {book_name} 4. MOVE (deg)")
        move_robot_deg(self.poses["move"])
        hold_seconds(self.hold_move_s, attached)

        place_pose = place_poses.get(detected_color, self.poses["place_blue"])
        carb.log_warn(f">> {book_name} 5. PLACE (deg) - detected color: {detected_color}")
        move_robot_deg(place_pose)
        hold_seconds(self.hold_place_s, attached)

        attached = False

        carb.log_warn(f">> {book_name} 6. PLACE 후 잠시 대기")
        move_robot_deg(place_pose)
        hold_seconds(1.0, attached)

        carb.log_warn(f">> {book_name} 0. READY (deg)")
        move_robot_deg(self.poses["ready"])
        hold_seconds(self.hold_place_s, attached)

        carb.log_warn(f"[TASK] {book_name} pick & place 완료 (detected: {detected_color})")

    # ----------------------------
    # 초기화/실행
    # ----------------------------
    def _open_stage_and_init(self):
        if not os.path.isfile(self.env_usd_path):
            raise FileNotFoundError(f"USD not found: {self.env_usd_path}")

        from isaacsim.core.utils.stage import open_stage
        carb.log_warn(f"[STAGE] open_stage: {self.env_usd_path}")
        open_stage(self.env_usd_path)
        simulation_app.update()

        self.stage = self.get_stage()

        og_prim = self.stage.GetPrimAtPath(self.graph_ur10)
        if og_prim and og_prim.IsValid():
            og_prim.SetActive(False)
            carb.log_warn("[INIT] OmniGraph 컨트롤러를 강제 비활성화했습니다.")

        self.fix_ur10_stiffness(self.stage)

        self.world = World(physics_dt=1/60, rendering_dt=1/60)
        self.ur10 = self.world.scene.add(
            SingleManipulator(
                prim_path=UR10_PRIM_PATH,
                name="ur10",
                end_effector_prim_path=self.ee_link_path,
            )
        )
        self.world.reset()

        self.suction_cup_path = self.resolve_suction_cup_path(self.stage)
        carb.log_warn(f"[CUP] final suction_cup path: {self.suction_cup_path}")

        self.ur10_indices = [self.ur10.get_dof_index(n) for n in self.joint_names]

        omni.timeline.get_timeline_interface().play()

        carb.log_warn(f"[WAIT] 로봇 초기 자세 그대로 {self.start_delay_s:.1f}초 대기...")
        for _ in range(int(self.start_delay_s * 60)):
            self.world.step(render=True)

    def run(self):
        self._open_stage_and_init()

        carb.log_warn("[RUN] Pick & Place 시작! (다중 책 + HSV 색상 탐지)")

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
        "ready": POSE_READY_DEG,
        "approach": POSE_APPROACH_DEG,
        "grasp": POSE_GRASP_DEG,
        "lift": POSE_LIFT_DEG,
        "move": POSE_MOVE_DEG,
        "place_red": POSE_PLACE_RED_DEG,
        "place_yellow": POSE_PLACE_YELLOW_DEG,
        "place_blue": POSE_PLACE_BLUE_DEG,
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
        debug_dir="/tmp/camera_debug",
    )

    try:
        app.run()
    except Exception as e:
        carb.log_error(f"[FATAL] Exception occurred: {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()