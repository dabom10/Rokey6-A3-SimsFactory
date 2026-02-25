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
# 설정 상수
# ================================
ENV_USD_PATH = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"

ROBOT_ARTICULATION_ROOT = "/Root/robot/run_robot"
UR10_PRIM_PATH = "/Root/robot/run_robot/ur10"
EE_LINK_PATH = "/Root/robot/run_robot/ur10/ee_link"
CAMERA_PRIM_PATH = "/Root/robot/run_robot/ur10/ee_link/short_gripper/Camera"

GRAPH_UR10 = "/Root/robot/run_robot/ur10/Graphs/Graphs/Position_Controller"

# Book 초기 생성 위치
BOOK_SPAWN_POS = (-12.7, 5.2, 1.5)
BOOK_SCALE = (0.15, 0.25, 0.04)

# Book 정의 (색상, 경로, HSV 범위)
# HSV: Hue(0-179), Saturation(0-255), Value(0-255)
# 주의: Red는 HSV에서 0도 근처와 180도 근처 두 곳에 분포하므로 두 범위로 분할
BOOKS = {
    "red": {
        "path": "/Root/red_book",
        "color": (1.0, 0.0, 0.0),
        # Red: 두 범위로 분할 (0~10도, 170~179도)
        # Saturation/Value 최소값을 30으로 설정 (더 민감한 탐지)
        "hsv_ranges": [
            (np.array([0, 30, 30]), np.array([10, 255, 255])),      # 0~10도
            (np.array([170, 30, 30]), np.array([179, 255, 255])),   # 170~179도
        ],
    },
    "blue": {
        "path": "/Root/blue_book",
        "color": (0.0, 0.0, 1.0),
        # Blue: 100~130도
        "hsv_ranges": [
            (np.array([100, 30, 30]), np.array([130, 255, 255])),
        ],
    },
    "yellow": {
        "path": "/Root/yellow_book",
        "color": (1.0, 1.0, 0.0),
        # Yellow: 15~35도 (더 넓은 범위)
        "hsv_ranges": [
            (np.array([15, 30, 30]), np.array([35, 255, 255])),
        ],
    },
}

# Book spawn 간격 (초)
BOOK_SPAWN_INTERVAL_S = 1.0

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
# 카메라 이미지 획득 및 색상 탐지
# ================================
def get_camera_image(stage: Usd.Stage, camera_prim_path: str) -> np.ndarray:
    """
    카메라에서 이미지 획득
    처리 과정: RGB (0~1 또는 0~255) → BGR (uint8) → 반환
    반환: uint8 BGR 이미지 (OpenCV 포맷)
    """
    try:
        from omni.isaac.sensor import Camera
        
        camera_prim = stage.GetPrimAtPath(camera_prim_path)
        if not camera_prim or not camera_prim.IsValid():
            carb.log_error(f"[CAMERA] Camera prim not found: {camera_prim_path}")
            return None
        
        camera = Camera(prim_path=camera_prim_path)
        camera.initialize()
        
        # RGB 이미지 획득
        rgb_data = camera.get_rgb()
        
        if rgb_data is None or rgb_data.size == 0:
            carb.log_warn("[CAMERA] Failed to get RGB data")
            return None
        
        # 데이터 타입 및 범위 로깅
        carb.log_warn(f"[CAMERA] RGB data shape: {rgb_data.shape}, dtype: {rgb_data.dtype}, range: [{rgb_data.min():.2f}, {rgb_data.max():.2f}]")
        
        # 0~1 범위라면 0~255로 정규화
        if rgb_data.max() <= 1.0:
            rgb_array = (rgb_data[:, :, :3] * 255).astype(np.uint8)
            carb.log_warn(f"[CAMERA] Normalized from 0~1 to 0~255")
        else:
            rgb_array = rgb_data[:, :, :3].astype(np.uint8)
        
        # RGB to BGR 변환 (OpenCV 포맷)
        bgr_image = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
        
        carb.log_warn(f"[CAMERA] ✓ 이미지 처리 완료 (RGB → BGR → uint8)")
        
        return bgr_image
    
    except Exception as e:
        carb.log_warn(f"[CAMERA] Error getting camera image: {e}")
        carb.log_warn(traceback.format_exc())
        return None


def save_camera_image_with_masks(image: np.ndarray, book_name: str) -> None:
    """
    카메라 이미지와 색상 마스크를 저장 (디버깅용)
    /tmp/camera_debug/ 디렉토리에 저장됨
    """
    try:
        debug_dir = "/tmp/camera_debug"
        os.makedirs(debug_dir, exist_ok=True)
        
        # 원본 이미지 저장
        cv2.imwrite(f"{debug_dir}/{book_name}_00_original.png", image)
        carb.log_warn(f"[DEBUG] 원본 이미지 저장: {debug_dir}/{book_name}_00_original.png")
        
        # BGR to HSV 변환
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 각 색상별 마스크 저장
        for idx, (book_color, book_info) in enumerate(BOOKS.items()):
            hsv_ranges = book_info["hsv_ranges"]
            
            # 각 색상의 모든 범위를 합쳐서 마스크 생성
            combined_mask = None
            for hsv_lower, hsv_upper in hsv_ranges:
                mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
                if combined_mask is None:
                    combined_mask = mask
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, mask)
            
            # Morphology 연산
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            
            cv2.imwrite(f"{debug_dir}/{book_name}_{idx+1:02d}_{book_color}_mask.png", combined_mask)
            pixel_count = cv2.countNonZero(combined_mask)
            carb.log_warn(f"[DEBUG] {book_color} 마스크: {pixel_count} 픽셀")
        
        carb.log_warn(f"[DEBUG] ============================================")
        carb.log_warn(f"[DEBUG] 이미지 저장 완료: {debug_dir}/")
        carb.log_warn(f"[DEBUG] 확인 방법:")
        carb.log_warn(f"[DEBUG]   1. Linux: eog {debug_dir}/ &")
        carb.log_warn(f"[DEBUG]   2. 또는: ls -la {debug_dir}/")
        carb.log_warn(f"[DEBUG] ============================================")
    
    except Exception as e:
        carb.log_warn(f"[DEBUG] 이미지 저장 실패: {e}")


def detect_book_color(image: np.ndarray) -> str:
    """
    BGR 이미지에서 책의 색상을 탐지합니다.
    
    처리 과정:
    1. BGR to HSV 변환
    2. 각 색상별 HSV 범위로 마스크 생성 (여러 범위 지원)
    3. Morphology 연산으로 노이즈 제거
    4. 가장 많은 픽셀을 가진 색상 선택
    
    반환값: "red", "blue", "yellow", 또는 "unknown"
    """
    if image is None or image.size == 0:
        carb.log_warn("[COLOR] Invalid image")
        return "unknown"
    
    # BGR to HSV 변환
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    color_count = defaultdict(int)
    
    carb.log_warn("[COLOR] ===== 색상 탐지 상세 정보 (HSV 기반) =====")
    carb.log_warn(f"[COLOR] HSV 이미지 범위: H[{hsv[:,:,0].min()}-{hsv[:,:,0].max()}], S[{hsv[:,:,1].min()}-{hsv[:,:,1].max()}], V[{hsv[:,:,2].min()}-{hsv[:,:,2].max()}]")
    
    # 각 색상에 대해 마스크 생성
    for book_name, book_info in BOOKS.items():
        hsv_ranges = book_info["hsv_ranges"]
        
        # 각 색상은 여러 범위를 가질 수 있음 (예: Red는 0~10도와 170~179도)
        combined_mask = None
        
        for hsv_lower, hsv_upper in hsv_ranges:
            # 범위 내의 픽셀 마스크 생성
            mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
            
            if combined_mask is None:
                combined_mask = mask
            else:
                combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        # Morphology 연산으로 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)   # 작은 노이즈 제거
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)  # 구멍 채우기
        
        # 픽셀 개수 계산
        pixel_count = cv2.countNonZero(combined_mask)
        color_count[book_name] = pixel_count
        
        # 범위 정보 로깅
        range_str = ", ".join([f"H[{h_lower[0]}-{h_upper[0]}]" for h_lower, h_upper in hsv_ranges])
        carb.log_warn(f"[COLOR]   {book_name:7s} ({range_str:20s}) : {pixel_count:6d} pixels")
    
    carb.log_warn("[COLOR] ==================================")
    
    # 가장 많은 픽셀을 가진 색상 선택
    if color_count:
        detected_color = max(color_count, key=color_count.get)
        max_pixels = color_count[detected_color]
        
        # 최소 임계값 (노이즈 제거) - 200픽셀로 낮춤 (더 민감하게)
        if max_pixels > 200:
            carb.log_warn(f"[COLOR] ✓ 탐지 완료: {detected_color} ({max_pixels} 픽셀)")
            return detected_color
        else:
            carb.log_warn(f"[COLOR] ✗ 탐지 실패: 최대값 {max_pixels} < 200 (임계값)")
            carb.log_warn(f"[COLOR] 디버깅: 각 색상의 픽셀 분포:")
            for color, count in sorted(color_count.items(), key=lambda x: x[1], reverse=True):
                carb.log_warn(f"[COLOR]   {color}: {count} pixels")
    
    carb.log_warn("[COLOR] ✗ 명확한 색상을 찾을 수 없음")
    return "unknown"


def capture_and_detect_color(stage: Usd.Stage, camera_prim_path: str, book_name: str = "unknown") -> str:
    """카메라로 촬영하고 색상 탐지"""
    carb.log_warn(f"[CAPTURE] 이미지 촬영 시작...")
    image = get_camera_image(stage, camera_prim_path)
    
    if image is None:
        carb.log_warn("[CAPTURE] ✗ 이미지 촬영 실패")
        return "unknown"
    
    carb.log_warn(f"[CAPTURE] ✓ 이미지 촬영 완료 ({image.shape[1]}x{image.shape[0]})")
    
    # 디버깅용 이미지 저장
    save_camera_image_with_masks(image, book_name)
    
    detected_color = detect_book_color(image)
    carb.log_warn(f"[CAPTURE] 최종 결과: {detected_color}")
    
    return detected_color


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
    suction_cup_path: str,
    camera_prim_path: str,
):
    """
    특정 책에 대해 pick & place 작업을 수행합니다.
    Approach 시점에 카메라로 촬영하여 색상을 탐지하고,
    탐지된 색상에 맞는 위치에 배치합니다.
    """
    carb.log_warn(f"[TASK] {book_name} pick & place 시작")

    # 색상에 맞는 place pose 선택
    place_poses = {
        "red": POSE_PLACE_RED_DEG,
        "blue": POSE_PLACE_BLUE_DEG,
        "yellow": POSE_PLACE_YELLOW_DEG,
    }

    current_action = None
    attached_quat_wxyz = None
    detected_color = "unknown"

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

    # # [DEBUG] 카메라 시야 확인용 반복 루프
    # carb.log_warn(f">> {book_name} [DEBUG] ================================================")
    # carb.log_warn(f">> {book_name} [DEBUG] 카메라 시야각 조정 모드 진입 (10초)")
    # carb.log_warn(f">> {book_name} [DEBUG] 이 시간 동안 시뮬레이션에서:")
    # carb.log_warn(f">> {book_name} [DEBUG]   1. 카메라의 위치/회전을 조정하세요")
    # carb.log_warn(f">> {book_name} [DEBUG]   2. 매 1초마다 색상 탐지를 시도합니다")
    # carb.log_warn(f">> {book_name} [DEBUG]   3. 로그를 보고 각 색상의 픽셀 수를 확인하세요")
    # carb.log_warn(f">> {book_name} [DEBUG] ================================================")
    
    move_robot_deg(POSE_APPROACH_DEG)
    
    # 10초 동안 매 1초마다 색상 탐지 시도
    debug_start = time.time()
    detection_attempt = 0
    
    while simulation_app.is_running() and (time.time() - debug_start) < 3.0:
        if current_action is not None:
            ur10.apply_action(current_action)
        
        world.step(render=True)
        
        # 1초마다 색상 탐지 시도
        if int(time.time() - debug_start) != detection_attempt:
            detection_attempt = int(time.time() - debug_start)
            carb.log_warn(f">> {book_name} [DEBUG] [{detection_attempt}초] 색상 탐지 시도...")
            test_image = get_camera_image(stage, camera_prim_path)
            if test_image is not None:
                detect_book_color(test_image)
            carb.log_warn(f">> {book_name} [DEBUG] [{detection_attempt}초] 완료")
    
    carb.log_warn(f">> {book_name} [DEBUG] 시야각 조정 완료. 최종 색상 탐지 수행...")
    carb.log_warn(f">> {book_name} [DEBUG] ================================================")
    
    # [DEBUG] 최종 색상 탐지 수행
    detected_color = capture_and_detect_color(stage, camera_prim_path, book_name)
    
    carb.log_warn(f">> {book_name} [DEBUG] ================================================")
    carb.log_warn(f">> {book_name} [DEBUG] 최종 탐지 색상: {detected_color}")
    carb.log_warn(f">> {book_name} [DEBUG] 계속 진행합니다...")
    carb.log_warn(f">> {book_name} [DEBUG] ================================================")

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

    # 탐지된 색상에 맞는 place pose 선택
    place_pose = place_poses.get(detected_color, POSE_PLACE_BLUE_DEG)
    carb.log_warn(f">> {book_name} 5. PLACE (deg) - detected color: {detected_color}")
    move_robot_deg(place_pose)
    hold_seconds(HOLD_PLACE_S, attached)

    attached = False

    carb.log_warn(f">> {book_name} 6. PLACE 후 잠시 대기")
    move_robot_deg(place_pose)
    hold_seconds(1, attached)

    carb.log_warn(f">> {book_name} 0. READY (deg)")
    move_robot_deg(POSE_READY_DEG)
    hold_seconds(HOLD_PLACE_S, attached)

    carb.log_warn(f"[TASK] {book_name} pick & place 완료 (detected: {detected_color})")


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

    carb.log_warn("[RUN] Pick & Place 시작! (다중 책 + HSV 색상 탐지)")

    # 각 책을 순회하며 spawn & pick&place 수행
    books_order = ["red", "blue", "yellow"]

    for idx, book_name in enumerate(books_order):
        book_path = BOOKS[book_name]["path"]

        # 첫 번째 책은 즉시 spawn, 이후는 10초 간격
        if idx > 0:
            carb.log_warn(f"[WAIT] 다음 책 spawn 대기 ({BOOK_SPAWN_INTERVAL_S:.1f}초)...")
            for _ in range(int(BOOK_SPAWN_INTERVAL_S * 60)):
                world.step(render=True)

        # 책 생성
        create_book(stage, book_name)
        simulation_app.update()

        # Pick & Place 수행 (색상 탐지 포함)
        pick_and_place_book(
            stage=stage,
            world=world,
            ur10=ur10,
            ur10_indices=ur10_indices,
            book_name=book_name,
            book_path=book_path,
            suction_cup_path=suction_cup_path,
            camera_prim_path=CAMERA_PRIM_PATH,
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