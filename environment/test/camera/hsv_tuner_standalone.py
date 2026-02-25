#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HSV 색상 범위 튜닝 도구
- 트랙바를 이용해 실시간으로 HSV 범위 조정
- 마스크 결과를 실시간으로 확인
- 최종 값을 저장 가능

사용법:
  1. 프로그램 실행
  2. 각 색상(Red, Blue, Yellow) 탭 클릭
  3. 트랙바로 H, S, V 범위 조정
  4. 마스크 결과 확인
  5. 최종 값을 코드에 복사
"""

import numpy as np
import cv2
from collections import deque
import threading
import time

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

# 기본 HSV 범위값 (OpenCV HSV: H(0~179), S(0~255), V(0~255))
DEFAULT_HSV = {
    "red": {
        "lower1": (0, 100, 100),      # Red 범위 1 하단
        "upper1": (10, 255, 255),     # Red 범위 1 상단
        "lower2": (160, 100, 100),    # Red 범위 2 하단 (180도 넘음)
        "upper2": (179, 255, 255),    # Red 범위 2 상단
    },
    "blue": {
        "lower": (100, 100, 100),
        "upper": (140, 255, 255),
    },
    "yellow": {
        "lower": (20, 100, 100),
        "upper": (40, 255, 255),
    }
}

# 포즈
POSE_GRASP = np.array([-1.5, -1.05, 1.55, -2.05, -1.57, 0.25], dtype=np.float64)

# ================================
# 유틸리티 함수
# ================================
def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()


class HSVTunerApp:
    def __init__(self):
        self.stage = None
        self.world = None
        self.robot = None
        self.camera = None
        self.ur10_indices = []
        self.current_action = None
        
        # 현재 선택된 색상
        self.current_color = "red"
        self.current_mode = "lower1" if self.current_color == "red" else "lower"
        
        # 트랙바 값
        self.hsv_values = {}
        self._init_hsv_values()
        
        self.running = True

    def _init_hsv_values(self):
        """HSV 값 초기화"""
        self.hsv_values = {
            "red": {
                "lower1_h": DEFAULT_HSV["red"]["lower1"][0],
                "lower1_s": DEFAULT_HSV["red"]["lower1"][1],
                "lower1_v": DEFAULT_HSV["red"]["lower1"][2],
                "upper1_h": DEFAULT_HSV["red"]["upper1"][0],
                "upper1_s": DEFAULT_HSV["red"]["upper1"][1],
                "upper1_v": DEFAULT_HSV["red"]["upper1"][2],
                "lower2_h": DEFAULT_HSV["red"]["lower2"][0],
                "lower2_s": DEFAULT_HSV["red"]["lower2"][1],
                "lower2_v": DEFAULT_HSV["red"]["lower2"][2],
                "upper2_h": DEFAULT_HSV["red"]["upper2"][0],
                "upper2_s": DEFAULT_HSV["red"]["upper2"][1],
                "upper2_v": DEFAULT_HSV["red"]["upper2"][2],
            },
            "blue": {
                "lower_h": DEFAULT_HSV["blue"]["lower"][0],
                "lower_s": DEFAULT_HSV["blue"]["lower"][1],
                "lower_v": DEFAULT_HSV["blue"]["lower"][2],
                "upper_h": DEFAULT_HSV["blue"]["upper"][0],
                "upper_s": DEFAULT_HSV["blue"]["upper"][1],
                "upper_v": DEFAULT_HSV["blue"]["upper"][2],
            },
            "yellow": {
                "lower_h": DEFAULT_HSV["yellow"]["lower"][0],
                "lower_s": DEFAULT_HSV["yellow"]["lower"][1],
                "lower_v": DEFAULT_HSV["yellow"]["lower"][2],
                "upper_h": DEFAULT_HSV["yellow"]["upper"][0],
                "upper_s": DEFAULT_HSV["yellow"]["upper"][1],
                "upper_v": DEFAULT_HSV["yellow"]["upper"][2],
            }
        }

    def setup_environment(self):
        """환경 설정"""
        from isaacsim.core.utils.stage import open_stage
        carb.log_warn(f"[SETUP] 스테이지 로딩 중...")
        open_stage(ENV_USD_PATH)
        simulation_app.update()

        self.stage = get_stage()

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
        
        self.camera = Camera(
            prim_path=CAMERA_PRIM_PATH,
            frequency=30,
            resolution=(640, 480)
        )
        self.camera.initialize()
        self.world.reset()

        self.ur10_indices = [self.robot.get_dof_index(n) for n in JOINT_NAMES]

        omni.timeline.get_timeline_interface().play()
        carb.log_warn("[SETUP] 시뮬레이션 시작...")
        for _ in range(60):
            self.world.step(render=True)

        # 카메라 포즈 설정
        self.current_action = ArticulationAction(joint_positions=POSE_GRASP, joint_indices=self.ur10_indices)
            
        carb.log_warn("[SETUP] ✅ 준비 완료!")

    def spawn_test_books(self):
        """테스트 큐브 생성"""
        for i, (color_name, rgb) in enumerate(BOOK_COLORS):
            book_path = f"/Root/{color_name}_test_book_{i}"

            cube = UsdGeom.Cube.Define(self.stage, book_path)
            cube.CreateSizeAttr(1.0)
            
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

            carb.log_warn(f"✨ 큐브 생성: {color_name.upper()}")

    def create_trackbar_window(self):
        """트랙바 윈도우 생성"""
        window_name = f"HSV Tuner - {self.current_color.upper()}"
        cv2.namedWindow(window_name)
        
        if self.current_color == "red":
            # Red는 2개 범위
            cv2.createTrackbar("Lower1_H", window_name, 0, 179, self._on_trackbar)
            cv2.createTrackbar("Lower1_S", window_name, 100, 255, self._on_trackbar)
            cv2.createTrackbar("Lower1_V", window_name, 100, 255, self._on_trackbar)
            cv2.createTrackbar("Upper1_H", window_name, 10, 179, self._on_trackbar)
            cv2.createTrackbar("Upper1_S", window_name, 255, 255, self._on_trackbar)
            cv2.createTrackbar("Upper1_V", window_name, 255, 255, self._on_trackbar)
            cv2.createTrackbar("Lower2_H", window_name, 160, 179, self._on_trackbar)
            cv2.createTrackbar("Lower2_S", window_name, 100, 255, self._on_trackbar)
            cv2.createTrackbar("Lower2_V", window_name, 100, 255, self._on_trackbar)
            cv2.createTrackbar("Upper2_H", window_name, 179, 179, self._on_trackbar)
            cv2.createTrackbar("Upper2_S", window_name, 255, 255, self._on_trackbar)
            cv2.createTrackbar("Upper2_V", window_name, 255, 255, self._on_trackbar)
        else:
            # Blue, Yellow는 1개 범위
            cv2.createTrackbar("Lower_H", window_name, self.hsv_values[self.current_color]["lower_h"], 179, self._on_trackbar)
            cv2.createTrackbar("Lower_S", window_name, self.hsv_values[self.current_color]["lower_s"], 255, self._on_trackbar)
            cv2.createTrackbar("Lower_V", window_name, self.hsv_values[self.current_color]["lower_v"], 255, self._on_trackbar)
            cv2.createTrackbar("Upper_H", window_name, self.hsv_values[self.current_color]["upper_h"], 179, self._on_trackbar)
            cv2.createTrackbar("Upper_S", window_name, self.hsv_values[self.current_color]["upper_s"], 255, self._on_trackbar)
            cv2.createTrackbar("Upper_V", window_name, self.hsv_values[self.current_color]["upper_v"], 255, self._on_trackbar)

    def _on_trackbar(self, x):
        """트랙바 콜백 (더미)"""
        pass

    def get_current_hsv_ranges(self):
        """현재 트랙바 값으로 HSV 범위 가져오기"""
        window_name = f"HSV Tuner - {self.current_color.upper()}"
        
        if self.current_color == "red":
            lower1_h = cv2.getTrackbarPos("Lower1_H", window_name)
            lower1_s = cv2.getTrackbarPos("Lower1_S", window_name)
            lower1_v = cv2.getTrackbarPos("Lower1_V", window_name)
            upper1_h = cv2.getTrackbarPos("Upper1_H", window_name)
            upper1_s = cv2.getTrackbarPos("Upper1_S", window_name)
            upper1_v = cv2.getTrackbarPos("Upper1_V", window_name)
            lower2_h = cv2.getTrackbarPos("Lower2_H", window_name)
            lower2_s = cv2.getTrackbarPos("Lower2_S", window_name)
            lower2_v = cv2.getTrackbarPos("Lower2_V", window_name)
            upper2_h = cv2.getTrackbarPos("Upper2_H", window_name)
            upper2_s = cv2.getTrackbarPos("Upper2_S", window_name)
            upper2_v = cv2.getTrackbarPos("Upper2_V", window_name)
            
            return (
                np.array([lower1_h, lower1_s, lower1_v]),
                np.array([upper1_h, upper1_s, upper1_v]),
                np.array([lower2_h, lower2_s, lower2_v]),
                np.array([upper2_h, upper2_s, upper2_v])
            )
        else:
            lower_h = cv2.getTrackbarPos("Lower_H", window_name)
            lower_s = cv2.getTrackbarPos("Lower_S", window_name)
            lower_v = cv2.getTrackbarPos("Lower_V", window_name)
            upper_h = cv2.getTrackbarPos("Upper_H", window_name)
            upper_s = cv2.getTrackbarPos("Upper_S", window_name)
            upper_v = cv2.getTrackbarPos("Upper_V", window_name)
            
            return (
                np.array([lower_h, lower_s, lower_v]),
                np.array([upper_h, upper_s, upper_v])
            )

    def detect_with_ranges(self, rgb, lower, upper, lower2=None, upper2=None):
        """주어진 범위로 색상 검출"""
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        
        mask = cv2.inRange(hsv, lower, upper)
        
        if lower2 is not None and upper2 is not None:
            mask += cv2.inRange(hsv, lower2, upper2)
        
        count = cv2.countNonZero(mask)
        return mask, count

    def print_current_ranges(self):
        """현재 범위값 콘솔에 출력"""
        ranges = self.get_current_hsv_ranges()
        
        if self.current_color == "red":
            lower1, upper1, lower2, upper2 = ranges
            carb.log_warn(f"RED 범위 1: lower=({lower1[0]}, {lower1[1]}, {lower1[2]}), upper=({upper1[0]}, {upper1[1]}, {upper1[2]})")
            carb.log_warn(f"RED 범위 2: lower=({lower2[0]}, {lower2[1]}, {lower2[2]}), upper=({upper2[0]}, {upper2[1]}, {upper2[2]})")
            carb.log_warn(f"\n파이썬 코드:")
            carb.log_warn(f'lower_red1 = np.array([{lower1[0]}, {lower1[1]}, {lower1[2]}])')
            carb.log_warn(f'upper_red1 = np.array([{upper1[0]}, {upper1[1]}, {upper1[2]}])')
            carb.log_warn(f'lower_red2 = np.array([{lower2[0]}, {lower2[1]}, {lower2[2]}])')
            carb.log_warn(f'upper_red2 = np.array([{upper2[0]}, {upper2[1]}, {upper2[2]}])')
        else:
            lower, upper = ranges
            color_name = self.current_color
            carb.log_warn(f"{color_name.upper()} 범위: lower=({lower[0]}, {lower[1]}, {lower[2]}), upper=({upper[0]}, {upper[1]}, {upper[2]})")
            carb.log_warn(f"\n파이썬 코드:")
            carb.log_warn(f'lower_{color_name} = np.array([{lower[0]}, {lower[1]}, {lower[2]}])')
            carb.log_warn(f'upper_{color_name} = np.array([{upper[0]}, {upper[1]}, {upper[2]}])')

    def run(self):
        """메인 실행"""
        self.setup_environment()
        self.spawn_test_books()
        self.create_trackbar_window()
        
        carb.log_warn("=" * 60)
        carb.log_warn("🎨 HSV 색상 범위 튜닝 도구")
        carb.log_warn("=" * 60)
        carb.log_warn("[1] - RED 색상 튜닝")
        carb.log_warn("[2] - BLUE 색상 튜닝")
        carb.log_warn("[3] - YELLOW 색상 튜닝")
        carb.log_warn("[P] - 현재 범위값 출력")
        carb.log_warn("[Q] - 종료")
        carb.log_warn("=" * 60)

        try:
            while simulation_app.is_running() and self.running:
                rgba = self.camera.get_rgba()
                
                if rgba is not None:
                    rgb = rgba[:, :, :3]
                    
                    # 현재 색상의 범위로 검출
                    ranges = self.get_current_hsv_ranges()
                    
                    if self.current_color == "red":
                        lower1, upper1, lower2, upper2 = ranges
                        mask, count = self.detect_with_ranges(rgb, lower1, upper1, lower2, upper2)
                    else:
                        lower, upper = ranges
                        mask, count = self.detect_with_ranges(rgb, lower, upper)
                    
                    # 마스크를 3채널로 변환
                    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    
                    # 원본과 마스크 합치기
                    combined = np.hstack([rgb, mask_3ch])
                    combined = cv2.resize(combined, (1280, 480))
                    
                    # 정보 추가
                    cv2.putText(combined, f"Detected: {count} pixels", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(combined, f"Threshold: 500 pixels", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 1)
                    status = "OK" if count > 500 else "LOW"
                    cv2.putText(combined, f"Status: {status}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if count > 500 else (0, 0, 255), 1)
                    
                    cv2.imshow(f"Preview - {self.current_color.upper()}", combined)

                # 로봇 제어
                if self.current_action is not None:
                    self.robot.apply_action(self.current_action)
                
                self.world.step(render=True)

                # 키 입력
                key = cv2.waitKey(1) & 0xFF
                if key == ord('1'):
                    self.current_color = "red"
                    carb.log_warn("🔴 RED 색상 튜닝으로 전환")
                elif key == ord('2'):
                    self.current_color = "blue"
                    carb.log_warn("🔵 BLUE 색상 튜닝으로 전환")
                elif key == ord('3'):
                    self.current_color = "yellow"
                    carb.log_warn("🟡 YELLOW 색상 튜닝으로 전환")
                elif key == ord('p') or key == ord('P'):
                    self.print_current_ranges()
                elif key == ord('q') or key == ord('Q'):
                    carb.log_warn("[INPUT] 프로그램 종료")
                    self.running = False

        except Exception as e:
            carb.log_error(f"[ERROR] {e}")
            import traceback
            carb.log_error(traceback.format_exc())
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        app = HSVTunerApp()
        app.run()
    except Exception as e:
        carb.log_error(f"[FATAL] {e}")
        import traceback
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()
