#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
카메라 시야각 디버깅 Standalone 스크립트 (Advanced)
- 좌상단: 카메라 실시간 영상 (RGB)
- 우상단: 빨강 마스크 (Red Mask)
- 좌하단: 파랑 마스크 (Blue Mask)
- 우하단: 노랑 마스크 (Yellow Mask)
- 별도 창: 색상별 픽셀 수 실시간 그래프 (선택사항)

키 입력:
  [A] - APPROACH 포즈 (접근)
  [G] - GRASP 포즈 (내려가기)
  [L] - LIFT 포즈 (들어올리기)
  [R] - 홈 포즈 (복귀)
  [Q] - 종료
"""

import os
import time
import traceback
import numpy as np
import cv2
import threading
from collections import deque

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics

from isaacsim.core.api import World
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera


# ================================
# 설정 상수
# ================================
ENV_USD_PATH = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"

ROBOT_ARTICULATION_ROOT = "/Root/robot/run_robot"
UR10_PRIM_PATH = "/Root/robot/run_robot/ur10"
EE_LINK_PATH = "/Root/robot/run_robot/ur10/ee_link"
GRAPH_UR10 = "/Root/robot/run_robot/ur10/Graphs/Graphs/Position_Controller"

CAMERA_PRIM_PATH = "/Root/robot/run_robot/ur10/ee_link/short_gripper/Camera"

# Book 초기 설정
BOOK_CREATE_POS = (-12.7, 5.2, 1.5)
BOOK_SCALE = (0.15, 0.25, 0.04)

BOOK_COLORS = [
    ("red", (1.0, 0.0, 0.0)),
    ("blue", (0.0, 0.0, 1.0)),
    ("yellow", (1.0, 1.0, 0.0))
]

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# 관절 포즈 프리셋
POSE_APPROACH = np.array([-1.5, -1.20, 1.40, -1.80, -1.57, 0.20], dtype=np.float64)
POSE_GRASP    = np.array([-1.5, -1.05, 1.55, -2.05, -1.57, 0.25], dtype=np.float64)
POSE_LIFT     = POSE_APPROACH.copy()
POSE_HOME     = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], dtype=np.float64)

# ================================
# 유틸리티 함수
# ================================
def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()


class CameraDebugApp:
    def __init__(self):
        self.stage = None
        self.world = None
        self.robot = None
        self.camera = None
        self.ur10_indices = []
        self.current_action = None
        
        self.book_path = None
        self.running = True
        
        # 색상 감지 이력 (실시간 그래프용)
        self.history_length = 120
        self.color_history = {
            "red": deque(maxlen=self.history_length),
            "blue": deque(maxlen=self.history_length),
            "yellow": deque(maxlen=self.history_length)
        }
        self.frame_count = 0

    def setup_environment(self):
        """환경 설정 및 카메라 초기화"""
        from isaacsim.core.utils.stage import open_stage
        carb.log_warn(f"[SETUP] 스테이지 로딩 중... ({ENV_USD_PATH})")
        open_stage(ENV_USD_PATH)
        simulation_app.update()

        self.stage = get_stage()

        # 옴니그래프 비활성화
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

        # World & Robot 초기화
        self.world = World(physics_dt=1/60, rendering_dt=1/60)
        self.robot = self.world.scene.add(
            SingleManipulator(prim_path=ROBOT_ARTICULATION_ROOT, name="ur10", end_effector_prim_path=EE_LINK_PATH)
        )
        
        # 📸 카메라 센서 초기화
        self.camera = Camera(
            prim_path=CAMERA_PRIM_PATH,
            frequency=30,
            resolution=(640, 480)
        )
        self.camera.initialize()
        self.world.reset()

        self.ur10_indices = [self.robot.get_dof_index(n) for n in JOINT_NAMES]

        omni.timeline.get_timeline_interface().play()
        carb.log_warn("[SETUP] 시뮬레이션 시작. 로봇 초기 자세 안정화 중...")
        for _ in range(60):
            self.world.step(render=True)
            
        carb.log_warn("[SETUP] ✅ 카메라 준비 완료!")

    def spawn_test_books(self, num_books=1):
        """테스트용 책들 생성"""
        if self.book_path:
            return
            
        for i in range(min(num_books, len(BOOK_COLORS))):
            color_name, rgb = BOOK_COLORS[i]
            book_path = f"/Root/{color_name}_test_book_{i}"

            cube = UsdGeom.Cube.Define(self.stage, book_path)
            cube.CreateSizeAttr(1.0)
            
            # 각 색상별로 약간 다른 위치에 배치
            offset_y = -1.0 * i
            xform = UsdGeom.Xformable(cube)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(BOOK_CREATE_POS[0], BOOK_CREATE_POS[1] + offset_y, BOOK_CREATE_POS[2]))
            xform.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
            xform.AddScaleOp().Set(Gf.Vec3f(*BOOK_SCALE))

            prim = self.stage.GetPrimAtPath(book_path)
            prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([rgb])

            try:
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.2)
            except:
                pass

            if i == 0:
                self.book_path = book_path

            carb.log_warn(f"✨ [TEST] 테스트 큐브 생성: {book_path} (색상: {color_name.upper()})")

    def set_target_pose(self, q_rad: np.ndarray, pose_name: str = ""):
        """로봇 목표 포즈 설정"""
        self.current_action = ArticulationAction(joint_positions=q_rad, joint_indices=self.ur10_indices)
        if pose_name:
            carb.log_warn(f"🤖 [MOTION] {pose_name} 포즈로 이동 중...")

    def detect_book_color(self, rgba):
        """색상 감지"""
        if rgba is None:
            return None, None

        rgb = rgba[:, :, :3]
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        # 색상 범위 설정
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

        # 이력 저장
        self.color_history["red"].append(counts["red"])
        self.color_history["blue"].append(counts["blue"])
        self.color_history["yellow"].append(counts["yellow"])

        return counts, {
            "red": mask_red,
            "blue": mask_blue,
            "yellow": mask_yellow,
            "rgb": rgb
        }

    def create_display_window(self, rgba, counts, masks):
        """카메라 영상과 색상 마스크를 4분할로 표시"""
        if rgba is None or counts is None:
            return

        rgb = masks["rgb"]
        
        # 각 마스크를 3채널 BGR로 변환
        mask_red_bgr = cv2.cvtColor(masks["red"], cv2.COLOR_GRAY2BGR)
        mask_blue_bgr = cv2.cvtColor(masks["blue"], cv2.COLOR_GRAY2BGR)
        mask_yellow_bgr = cv2.cvtColor(masks["yellow"], cv2.COLOR_GRAY2BGR)

        # 텍스트 추가
        cv2.putText(rgb, "RGB Camera Feed", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(rgb, f"Frame: {self.frame_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

        cv2.putText(mask_red_bgr, f"Red Mask: {counts['red']:6d}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(mask_blue_bgr, f"Blue Mask: {counts['blue']:6d}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(mask_yellow_bgr, f"Yellow Mask: {counts['yellow']:6d}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # 임계값 라인 추가
        cv2.line(mask_red_bgr, (0, 240), (640, 240), (200, 200, 200), 1)  # 500px 기준
        cv2.line(mask_blue_bgr, (0, 240), (640, 240), (200, 200, 200), 1)
        cv2.line(mask_yellow_bgr, (0, 240), (640, 240), (200, 200, 200), 1)

        # 4분할 합치기
        top_row = np.hstack([rgb, mask_red_bgr])
        bottom_row = np.hstack([mask_blue_bgr, mask_yellow_bgr])
        combined = np.vstack([top_row, bottom_row])

        # 리사이징 (1280x960)
        combined = cv2.resize(combined, (1280, 960))

        cv2.imshow("🎥 Camera Debug View - RGB + Color Masks", combined)

    def display_info_overlay(self):
        """정보 오버레이 표시"""
        info_text = [
            "===== Camera Debug Console =====",
            f"Frame: {self.frame_count}",
            f"Red pixels: {list(self.color_history['red'])[-1] if self.color_history['red'] else 0}",
            f"Blue pixels: {list(self.color_history['blue'])[-1] if self.color_history['blue'] else 0}",
            f"Yellow pixels: {list(self.color_history['yellow'])[-1] if self.color_history['yellow'] else 0}",
            "",
            "Keyboard Controls:",
            "[A] APPROACH pose",
            "[G] GRASP pose",
            "[L] LIFT pose",
            "[R] HOME pose",
            "[Q] Quit"
        ]
        
        if self.frame_count % 30 == 0:  # 1초마다
            carb.log_warn(" | ".join(info_text[0:5]))

    def run(self):
        """메인 실행 루프"""
        self.setup_environment()
        self.spawn_test_books(num_books=3)
        
        carb.log_warn("=" * 60)
        carb.log_warn("📷 카메라 디버깅 모드 시작")
        carb.log_warn("=" * 60)
        carb.log_warn("키 입력:")
        carb.log_warn("  [A] - APPROACH 포즈 (카메라가 큐브 위에서 45도)")
        carb.log_warn("  [G] - GRASP 포즈 (카메라가 큐브 정면)")
        carb.log_warn("  [L] - LIFT 포즈 (들어올린 상태)")
        carb.log_warn("  [R] - HOME 포즈 (원위치)")
        carb.log_warn("  [Q] - 종료")
        carb.log_warn("=" * 60)
        
        try:
            while simulation_app.is_running() and self.running:
                # 카메라 영상 획득
                rgba = self.camera.get_rgba()
                
                if rgba is not None:
                    # 색상 감지
                    counts, masks = self.detect_book_color(rgba)
                    
                    if counts is not None:
                        self.frame_count += 1
                        
                        # OpenCV 윈도우 표시
                        self.create_display_window(rgba, counts, masks)
                        
                        # 콘솔 로그 (30프레임마다 = 1초마다)
                        if self.frame_count % 30 == 0:
                            best_color = max(counts, key=counts.get)
                            status = "✅" if counts[best_color] > 500 else "❌"
                            carb.log_warn(f"[FRAME {self.frame_count:04d}] RED: {counts['red']:5d}, BLUE: {counts['blue']:5d}, YELLOW: {counts['yellow']:5d} {status} Best: {best_color.upper()}")

                # 로봇 제어
                if self.current_action is not None:
                    self.robot.apply_action(self.current_action)
                
                self.world.step(render=True)

                # 키보드 입력 처리 (논블로킹)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('a') or key == ord('A'):
                    self.set_target_pose(POSE_APPROACH, "APPROACH (접근 포즈)")
                    
                elif key == ord('g') or key == ord('G'):
                    self.set_target_pose(POSE_GRASP, "GRASP (내려가기 포즈)")
                    
                elif key == ord('l') or key == ord('L'):
                    self.set_target_pose(POSE_LIFT, "LIFT (들어올리기 포즈)")
                    
                elif key == ord('r') or key == ord('R'):
                    self.set_target_pose(POSE_HOME, "HOME (복귀 포즈)")
                    
                elif key == ord('q') or key == ord('Q'):
                    carb.log_warn("[INPUT] 프로그램을 종료합니다...")
                    self.running = False
                    
        except Exception as e:
            carb.log_error(f"[ERROR] {e}")
            carb.log_error(traceback.format_exc())
        finally:
            cv2.destroyAllWindows()
            carb.log_warn("[CLEANUP] 시뮬레이션 종료...")


if __name__ == "__main__":
    try:
        app = CameraDebugApp()
        app.run()
    except Exception as e:
        carb.log_error(f"[FATAL] {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()
