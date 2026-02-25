#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import traceback
import numpy as np
import cv2  # OpenCV 임포트

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.usd
import omni.timeline
import omni.kit.commands

from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics

from isaacsim.core.api import World
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera  # 카메라 API 추가


# ================================
# 설정 상수
# ================================
ENV_USD_PATH = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"

ROBOT_ARTICULATION_ROOT = "/Root/robot/run_robot"
UR10_PRIM_PATH = "/Root/robot/run_robot/ur10"
EE_LINK_PATH = "/Root/robot/run_robot/ur10/ee_link"
GRAPH_UR10 = "/Root/robot/run_robot/ur10/Graphs/Graphs/Position_Controller"

# 카메라 경로
CAMERA_PRIM_PATH = "/Root/robot/run_robot/ur10/ee_link/short_gripper/Camera"

# Book 초기 설정
BOOK_CREATE_POS = (-12.7, 5.2, 1.5)
BOOK_SCALE = (0.15, 0.25, 0.04)

# 스폰 설정 (딱 3번만 생성)
MAX_BOOKS = 3
SPAWN_INTERVAL = 10.0
BOOK_COLORS = [
    ("red", (1.0, 0.0, 0.0)),
    ("blue", (0.0, 0.0, 1.0)),
    ("yellow", (1.0, 1.0, 0.0))
]

# 동작 타이밍
START_DELAY_S = 3.0
HOLD_APPROACH_S = 2.0
HOLD_GRASP_S = 1.5
HOLD_LIFT_S = 2.0
HOLD_MOVE_S = 2.0
HOLD_PLACE_S = 1.5

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# ================================
# 관절 포즈 프리셋 (deg)
# - standalone_another_dimension.py의 값을 기반으로 변환
# ================================
POSE_READY_DEG    = [0, -90.0, -90.0, -90, 90.0, 0.0]
POSE_APPROACH_DEG = [115, -90.0, -90.0, -90, 90.0, 0.0]
POSE_GRASP_DEG    = [115, -123.0, -87.0, -60.0, 90.0, 0.0]
POSE_LIFT_DEG     = [115, -90.0, -90.0, -90, 90.0, 0.0]
POSE_MOVE_DEG     = [0, -90.0, -90.0, -90, 90.0, 0.0]

# 색상별 Place 목표 좌표 (deg)
POSE_PLACE_RED_DEG    = [5, -120.0, -90.0, -60, 90.0, 0.0]
POSE_PLACE_YELLOW_DEG = [7, -103.0, -122.0, -45, 90.0, 0.0]
POSE_PLACE_BLUE_DEG   = [10, -70, -140.0, -55, 90.0, 0.0]

# KLT 상자 이름 매핑
KLT_NAMES = {
    "red": "small_KLT",
    "blue": "small_KLT_01",
    "yellow": "small_KLT_02"
}

SUCTION_CUP_PATH = "/Root/robot/run_robot/ur10/ee_link/short_gripper/suction_cup"
ATTACH_OFFSET_IN_CUP_FRAME = np.array([0.0, 0.0, 0.05], dtype=np.float64)


# ================================
# 유틸리티 함수
# ================================
def deg2rad(deg_array):
    """deg 배열을 rad로 변환"""
    return np.deg2rad(np.array(deg_array, dtype=np.float64))

def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()

def quat_wxyz_to_rotmat(q_wxyz: tuple) -> np.ndarray:
    w, x, y, z = [float(v) for v in q_wxyz]
    n = (w*w + x*x + y*y + z*z) ** 0.5
    if n < 1e-12: return np.eye(3, dtype=np.float64)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


class UR10PickAndPlaceApp:
    def __init__(self):
        self.target_conveyor_x = -9.42 
        self.stage = None
        self.world = None
        self.robot = None
        self.camera = None
        self.ur10_indices = []
        self.current_action = None
        
        self.books_queue = []
        self.spawn_count = 0
        self.next_spawn_time = 0.0

    def setup_environment(self):
        from isaacsim.core.utils.stage import open_stage
        carb.log_warn(f"[SETUP] 스테이지 로딩 중... ({ENV_USD_PATH})")
        open_stage(ENV_USD_PATH)
        simulation_app.update()

        self.stage = get_stage()

        # 옴니그래프 비활성화 & 강성 펌핑
        og_prim = self.stage.GetPrimAtPath(GRAPH_UR10)
        if og_prim and og_prim.IsValid():
            og_prim.SetActive(False)
            
        for prim in self.stage.Traverse():
            if prim.GetName() in JOINT_NAMES:
                for drive_name in ["angular", "rotX", "rotY", "rotZ"]:
                    drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
                    if drive:
                        drive.GetStiffnessAttr().Set(1e7)
                        drive.GetDampingAttr().Set(1e6)

        self.world = World(physics_dt=1/60, rendering_dt=1/60)
        self.robot = self.world.scene.add(
            SingleManipulator(prim_path=ROBOT_ARTICULATION_ROOT, name="ur10", end_effector_prim_path=EE_LINK_PATH)
        )
        
        # 📸 카메라 센서 초기화
        self.camera = Camera(
            prim_path=CAMERA_PRIM_PATH,
            frequency=20,
            resolution=(640, 480)
        )
        self.camera.initialize()
        self.world.reset()

        self.ur10_indices = [self.robot.get_dof_index(n) for n in JOINT_NAMES]

        omni.timeline.get_timeline_interface().play()
        carb.log_warn("[SETUP] 시뮬레이션 시작. 로봇 초기 자세 안정화 중...")
        for _ in range(60):
            self.world.step(render=True)
            
        self.next_spawn_time = time.time() + 1.0 # 1초 뒤 첫 스폰

    def spawn_new_book(self):
        """정확히 3개의 책만 순서대로 생성합니다."""
        if self.spawn_count >= MAX_BOOKS:
            return

        color_name, rgb = BOOK_COLORS[self.spawn_count]
        new_path = f"/Root/{color_name}_book_{self.spawn_count}"

        cube = UsdGeom.Cube.Define(self.stage, new_path)
        cube.CreateSizeAttr(1.0)
        
        xform = UsdGeom.Xformable(cube)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*BOOK_CREATE_POS))
        xform.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        xform.AddScaleOp().Set(Gf.Vec3f(*BOOK_SCALE))

        prim = self.stage.GetPrimAtPath(new_path)
        prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([rgb])

        try: UsdPhysics.CollisionAPI.Apply(prim)
        except: pass
        try: UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
        except: pass
        try: UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.2)
        except: pass

        carb.log_warn(f"✨ [SPAWN] 큐브 생성 완료: {new_path}")
        self.books_queue.append(new_path)
        self.spawn_count += 1

    def detect_book_color(self) -> str:
        """OpenCV를 활용해 현재 카메라가 보고 있는 큐브의 색상을 반환합니다."""
        rgba = self.camera.get_rgba()
        if rgba is None:
            carb.log_warn("❌ [CAMERA_ERROR] 카메라 이미지를 가져올 수 없습니다. 기본값(red) 반환.")
            return "red"
            
        # RGBA -> RGB -> HSV 변환
        rgb = rgba[:, :, :3]
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        # 색상 범위 설정 (OpenCV HSV: H(0~179), S(0~255), V(0~255))
        lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
        lower_red2, upper_red2 = np.array([160, 100, 100]), np.array([179, 255, 255])
        lower_blue, upper_blue = np.array([100, 100, 100]), np.array([140, 255, 255])
        lower_yellow, upper_yellow = np.array([20, 100, 100]), np.array([40, 255, 255])

        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        counts = {
            "red": cv2.countNonZero(mask_red),
            "blue": cv2.countNonZero(mask_blue),
            "yellow": cv2.countNonZero(mask_yellow)
        }

        best_color = max(counts, key=counts.get)
        carb.log_warn(f"[COLOR_DEBUG] 색상 감지 픽셀 수 - RED: {counts['red']:5d}, BLUE: {counts['blue']:5d}, YELLOW: {counts['yellow']:5d}")
        
        if counts[best_color] > 500: # 픽셀 수가 임계치 이상일 때만 유효 판정
            carb.log_warn(f"✅ [COLOR_DETECTION] 인식된 색상: {best_color.upper()} (픽셀 수: {counts[best_color]})")
            return best_color
        
        carb.log_warn(f"⚠️ [COLOR_WARNING] 색상 인식이 불확실합니다. 기본값(red) 사용. (best: {counts[best_color]})")
        return "red" # 인식이 잘 안됐을 때의 기본값

    def _get_world_pose(self, prim_path: str):
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return np.zeros(3), (1.0, 0.0, 0.0, 0.0)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = xf.ExtractTranslation()
        rot = xf.ExtractRotationQuat()
        return np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64), \
               (float(rot.GetReal()), float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]), float(rot.GetImaginary()[2]))

    def _teleport_prim(self, prim_path, pos_xyz, quat_wxyz):
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid(): return
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        op_t = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
        op_r = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient), None)
        if not op_t or not op_r:
            xform.ClearXformOpOrder()
            op_t, op_r = xform.AddTranslateOp(), xform.AddOrientOp()
            xform.AddScaleOp()
        op_t.Set(Gf.Vec3d(float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])))
        w, x, y, z = quat_wxyz
        op_r.Set(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))

    def attach_book(self, active_book_path):
        prim = self.stage.GetPrimAtPath(active_book_path)
        if not prim.IsValid(): return
        
        rb = UsdPhysics.RigidBodyAPI(prim)
        if rb: rb.GetRigidBodyEnabledAttr().Set(False)
        simulation_app.update()
        
        new_path = SUCTION_CUP_PATH + "/grasped_book"
        omni.kit.commands.execute("MovePrim", path_from=active_book_path, path_to=new_path)
        simulation_app.update()
        
        new_prim = self.stage.GetPrimAtPath(new_path)
        xform = UsdGeom.Xformable(new_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.02))
        xform.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        xform.AddScaleOp().Set(Gf.Vec3f(*BOOK_SCALE))
        
        return new_path

    def detach_book(self, active_book_path):
        prim = self.stage.GetPrimAtPath(active_book_path)
        if not prim.IsValid(): return
            
        world_pos, world_quat = self._get_world_pose(active_book_path)
        new_path = "/Root/placed_book_" + str(time.time()).replace(".", "")
        
        omni.kit.commands.execute("MovePrim", path_from=active_book_path, path_to=new_path)
        simulation_app.update()
        
        self._teleport_prim(new_path, world_pos, world_quat)
        
        new_prim = self.stage.GetPrimAtPath(new_path)
        rb = UsdPhysics.RigidBodyAPI(new_prim)
        if rb: rb.GetRigidBodyEnabledAttr().Set(True)

    def set_target_pose_deg(self, q_deg: list):
        """deg 배열을 받아서 rad로 변환 후 설정"""
        q_rad = deg2rad(q_deg)
        self.current_action = ArticulationAction(joint_positions=q_rad, joint_indices=self.ur10_indices)

    def hold_seconds(self, seconds: float):
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < seconds:
            # 1. 스폰 타이머 체크 (3개까지만)
            if self.spawn_count < MAX_BOOKS and time.time() >= self.next_spawn_time:
                self.spawn_new_book()
                self.next_spawn_time += SPAWN_INTERVAL

            # 2. 로봇 제어 틱
            if self.current_action is not None:
                self.robot.apply_action(self.current_action)
            self.world.step(render=True)

    def run_sequence(self):
        carb.log_warn("===== 자율 분류 Pick & Place 시작 =====")
        
        # 첫 생성 대기
        self.hold_seconds(START_DELAY_S)

        while simulation_app.is_running():
            if len(self.books_queue) == 0:
                # 3개를 다 처리했고, 큐가 비었으면 종료
                if self.spawn_count >= MAX_BOOKS:
                    carb.log_warn("✅ 3개의 큐브를 모두 성공적으로 분류했습니다!")
                    break
                self.hold_seconds(0.1)
                continue

            target_book = self.books_queue[0]
            pos, _ = self._get_world_pose(target_book)
            
            # 컨베이어 도착 감지
            if pos[0] < self.target_conveyor_x:
                self.hold_seconds(0.1)
                continue

            # 🎯 큐브 도착
            self.books_queue.pop(0)
            carb.log_warn(f"\n🎯 [ARRIVAL] {target_book} 도착! 픽앤플레이스 시작")
            
            # 0. READY (초기 자세)
            carb.log_warn(">> 0. READY (deg)")
            self.set_target_pose_deg(POSE_READY_DEG)
            self.hold_seconds(HOLD_APPROACH_S)

            # 1. APPROACH (접근)
            carb.log_warn(">> 1. APPROACH (deg)")
            self.set_target_pose_deg(POSE_APPROACH_DEG)
            self.hold_seconds(HOLD_APPROACH_S)

            # 📸 2. 비전 카메라로 색상 인식
            detected_color = self.detect_book_color()
            carb.log_warn(f"👁️‍🗨️ [VISION] 인식된 책 색상: {detected_color.upper()}")

            # 3. GRASP (내려가서 잡기)
            carb.log_warn(">> 2. GRASP (deg)")
            self.set_target_pose_deg(POSE_GRASP_DEG)
            self.hold_seconds(HOLD_GRASP_S)

            # 🔗 부착
            active_book_path = self.attach_book(target_book)

            # 4. LIFT (들어올리기)
            carb.log_warn(">> 3. LIFT (deg)")
            self.set_target_pose_deg(POSE_LIFT_DEG)
            self.hold_seconds(HOLD_LIFT_S)

            # 5. MOVE (이동)
            carb.log_warn(">> 4. MOVE (deg)")
            self.set_target_pose_deg(POSE_MOVE_DEG)
            self.hold_seconds(HOLD_MOVE_S)

            # 6. PLACE (색상에 맞춰 KLT 상자로 이동)
            carb.log_warn(f">> 5. PLACE (deg) - {detected_color.upper()}")
            if detected_color == "red":
                target_place_pose = POSE_PLACE_RED_DEG
            elif detected_color == "yellow":
                target_place_pose = POSE_PLACE_YELLOW_DEG
            else:  # blue
                target_place_pose = POSE_PLACE_BLUE_DEG
            
            klt_name = KLT_NAMES[detected_color]
            
            # 디버깅 출력: 목표 좌표와 KLT 상자 이름
            carb.log_warn(f"📦 [PLACE_DEBUG] 색상: {detected_color.upper()}")
            carb.log_warn(f"📦 [PLACE_DEBUG] 목표 KLT 상자: {klt_name}")
            carb.log_warn(f"📦 [PLACE_DEBUG] 목표 관절각도(deg): J1={target_place_pose[0]:.1f}, J2={target_place_pose[1]:.1f}, J3={target_place_pose[2]:.1f}, J4={target_place_pose[3]:.1f}, J5={target_place_pose[4]:.1f}, J6={target_place_pose[5]:.1f}")
            carb.log_warn(f"📦 [PLACE_START] {detected_color.upper()} 상자({klt_name})로 이동합니다.")
            
            self.set_target_pose_deg(target_place_pose)
            self.hold_seconds(HOLD_PLACE_S)

            # 🔓 분리
            self.detach_book(active_book_path)
            carb.log_warn(f"✨ [PLACE_COMPLETE] {detected_color.upper()} 상자에 성공적으로 배치했습니다!")

            # 7. RETREAT (복귀)
            carb.log_warn(">> 6. RETREAT (deg)")
            self.set_target_pose_deg(POSE_LIFT_DEG)
            self.hold_seconds(HOLD_LIFT_S)

    def run(self):
        self.setup_environment()
        self.run_sequence()
        
        carb.log_warn("[IDLE] 모든 작업이 완료되었습니다. 시뮬레이션 유지...")
        while simulation_app.is_running():
            if self.current_action is not None:
                self.robot.apply_action(self.current_action)
            self.world.step(render=True)


if __name__ == "__main__":
    try:
        app = UR10PickAndPlaceApp()
        app.run()
    except Exception as e:
        carb.log_error(f"[FATAL] 오류 발생: {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()