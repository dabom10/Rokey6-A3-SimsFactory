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

from pxr import Usd, UsdGeom, Gf, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


# ================================
# 코드에서 장소 지정 (터미널 X)
# ================================
PLACE = "B"  # "A"=red, "B"=yellow, "C"=blue


# ================================
# 설정 상수 (클래스 밖)
# ================================
ENV_USD_PATH = "/home/kyb/Rokey6-A3-SimsFactory/project/environment_carter_shelf.usd"

# 이 USD 구조에서는 carter + ur10 이 하나의 articulation으로 묶여있음
# (Stage에서 /Root/robot/robot 이 ArticulationRoot 인 경우)
ROBOT_ARTICULATION_ROOT = "/Root/robot/robot"

# suction / world pose 계산에만 사용
EE_LINK_PATH = "/Root/robot/robot/nova_carter/ur10/ee_link"

# 이 씬에서는 run_robot/Graphs 경로가 없을 수 있으니 기본 None 처리
GRAPH_UR10 = None

# Carter ActionGraph(있는 경우) 비활성화해서 간섭 방지
DISABLE_CARTER_ACTIONGRAPHS = True
CARTER_ACTIONGRAPH_PATHS = [
    "/Root/robot/robot/nova_carter/ActionGraph_differential",
    "/Root/robot/robot/nova_carter/ActionGraph_tf_odom",
    "/Root/robot/robot/nova_carter/ActionGraph_lidar",
]

# 씬에 존재하는 책 prim 경로들
BOOKS = {
    "red": "/Root/red_book",
    "yellow": "/Root/yellow_book",
    "blue": "/Root/blue_book",
}

# UR10 6개 조인트 이름
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# 시작 자세(항상)
POSE_READY_DEG = [0, -90.0, -90.0, -90, 90.0, 0.0]

# 책을 잡는(픽업) 자세: 색상별
POSE_PICK_RED_DEG = [5, -127.0, -92.0, -53, 90.0, 0.0]
POSE_PICK_YELLOW_DEG = [7, -97.0, -118.0, -50, 90.0, 0.0]
POSE_PICK_BLUE_DEG = [10, -70, -140.0, -55, 90.0, 0.0]

POSE_PICK_BY_COLOR = {
    "red": POSE_PICK_RED_DEG,
    "yellow": POSE_PICK_YELLOW_DEG,
    "blue": POSE_PICK_BLUE_DEG,
}

# MID 자세: 색상별
POSE_MID_RED_DEG = [5, -110.0, -78.0, -79, 90.0, 0.0]
POSE_MID_YELLOW_DEG = [5, -110.0, -78.0, -79, 90.0, 0.0]
POSE_MID_BLUE_DEG = [0, 0, 0, 0, 0, 0]  # 필요하면 채우기

POSE_MID_BY_COLOR = {
    "red": POSE_MID_RED_DEG,
    "yellow": POSE_MID_YELLOW_DEG,
    "blue": POSE_MID_BLUE_DEG,
}

# 선반 이동/놓기 자세
POSE_2SHELF_DEG = [90, -120.0, -70.0, -90, 180.0, -15.0]
POSE_PLACE2SHELF_DEG = [90, -140.0, -70.0, -70, 180.0, -15.0]

SUCTION_CUP_NAME = "suction_cup"
ATTACH_OFFSET_IN_CUP_FRAME = np.array([0.0, 0.0, 0.01], dtype=np.float64)

# 타이밍
START_DELAY_S = 2.0
READY_BEFORE_ACTION_WAIT_S = 2.0
HOLD_MOVE_S = 2.0
HOLD_ATTACH_S = 0.6
HOLD_DETACH_S = 0.6


class amr2shelf:
    def __init__(
        self,
        env_usd_path: str,
        robot_articulation_root: str,
        ee_link_path: str,
        graph_ur10,
        joint_names: list,
        books: dict,
        place: str,
        pose_ready_deg: list,
        pose_pick_by_color: dict,
        pose_mid_by_color: dict,
        pose_2shelf_deg: list,
        pose_place2shelf_deg: list,
        suction_cup_name: str,
        attach_offset_in_cup_frame: np.ndarray,
        start_delay_s: float,
        ready_before_action_wait_s: float,
        hold_move_s: float,
        hold_attach_s: float,
        hold_detach_s: float,
        disable_carter_actiongraphs: bool,
        carter_actiongraph_paths: list,
    ):
        self.env_usd_path = env_usd_path
        self.robot_articulation_root = robot_articulation_root
        self.ee_link_path = ee_link_path
        self.graph_ur10 = graph_ur10
        self.joint_names = list(joint_names)

        self.books = dict(books)

        self.place = str(place).strip().upper()
        self.target_color = self.place_to_color(self.place)
        if self.target_color == "unknown":
            raise ValueError(f"PLACE must be A/B/C. got: {place}")

        if self.target_color not in self.books:
            raise ValueError(f"BOOKS has no key: {self.target_color}")
        self.book_path = self.books[self.target_color]

        self.pose_ready_deg = list(pose_ready_deg)

        self.pose_pick_by_color = dict(pose_pick_by_color)
        if self.target_color not in self.pose_pick_by_color:
            raise ValueError(f"POSE_PICK_BY_COLOR has no key: {self.target_color}")
        self.pose_pick_deg = list(self.pose_pick_by_color[self.target_color])

        self.pose_mid_by_color = dict(pose_mid_by_color)
        if self.target_color not in self.pose_mid_by_color:
            raise ValueError(f"POSE_MID_BY_COLOR has no key: {self.target_color}")
        self.pose_mid_deg = list(self.pose_mid_by_color[self.target_color])

        self.pose_2shelf_deg = list(pose_2shelf_deg)
        self.pose_place2shelf_deg = list(pose_place2shelf_deg)

        self.suction_cup_name = suction_cup_name
        self.attach_offset_in_cup_frame = attach_offset_in_cup_frame.astype(np.float64).copy()

        self.start_delay_s = float(start_delay_s)
        self.ready_before_action_wait_s = float(ready_before_action_wait_s)
        self.hold_move_s = float(hold_move_s)
        self.hold_attach_s = float(hold_attach_s)
        self.hold_detach_s = float(hold_detach_s)

        self.disable_carter_actiongraphs = bool(disable_carter_actiongraphs)
        self.carter_actiongraph_paths = list(carter_actiongraph_paths)

        self.stage = None
        self.world = None
        self.art = None
        self.arm_indices = None
        self.suction_cup_path = None

        self._attached = False
        self._original_parent_path = None
        self._attached_path = None
        self._book_name = None
        self._grasp_frame_path = None

    @staticmethod
    def place_to_color(place: str) -> str:
        p = str(place).strip().upper()
        if p == "A":
            return "red"
        if p == "B":
            return "yellow"
        if p == "C":
            return "blue"
        return "unknown"

    @staticmethod
    def deg2rad(deg_array):
        return np.deg2rad(np.array(deg_array, dtype=np.float64))

    @staticmethod
    def get_stage():
        return omni.usd.get_context().get_stage()

    @staticmethod
    def prim_exists(stage, prim_path):
        prim = stage.GetPrimAtPath(prim_path)
        return bool(prim and prim.IsValid())

    @staticmethod
    def get_world_xf(stage, prim_path) -> Gf.Matrix4d:
        prim = stage.GetPrimAtPath(prim_path)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return xf

    @staticmethod
    def _assert_prim(stage, prim_path: str, tag: str):
        prim = stage.GetPrimAtPath(prim_path)
        if (not prim) or (not prim.IsValid()):
            raise RuntimeError(f"[PATH] {tag} prim not found: {prim_path}")
        return prim

    def resolve_suction_cup_path(self, stage):
        direct = self.ee_link_path + "/" + self.suction_cup_name
        if self.prim_exists(stage, direct):
            return direct

        for p in stage.Traverse():
            if p.GetName() == self.suction_cup_name:
                return str(p.GetPath())

        raise RuntimeError("suction_cup prim not found")

    def _move_prim(self, src_path: str, dst_path: str) -> None:
        import omni.kit.commands as commands

        commands.execute("MovePrim", path_from=src_path, path_to=dst_path)

    def ensure_grasp_frame(self) -> str:
        stage = self.stage
        grasp_path = str(Sdf.Path(self.suction_cup_path).AppendChild("grasp_frame"))
        if self.prim_exists(stage, grasp_path):
            return grasp_path

        prim = stage.DefinePrim(grasp_path, "Xform")
        xformable = UsdGeom.Xformable(prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        xformable.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        xformable.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

        carb.log_warn(f"[GRASP_FRAME] created: {grasp_path}")
        return grasp_path

    def attach_book_to_cup(self):
        if self._attached:
            return

        stage = self.stage
        book_path = self.book_path

        if not self.prim_exists(stage, book_path):
            raise RuntimeError(f"[ATTACH] book prim not found: {book_path}")
        if not self.prim_exists(stage, self.suction_cup_path):
            raise RuntimeError(f"[ATTACH] cup prim not found: {self.suction_cup_path}")

        self._grasp_frame_path = self.ensure_grasp_frame()

        book_prim = stage.GetPrimAtPath(book_path)
        self._book_name = book_prim.GetName()
        self._original_parent_path = str(book_prim.GetParent().GetPath())

        book_world = self.get_world_xf(stage, book_path)

        new_path = str(Sdf.Path(self._grasp_frame_path).AppendChild(self._book_name))
        if self.prim_exists(stage, new_path):
            raise RuntimeError(f"[ATTACH] already exists under grasp_frame: {new_path}")

        self._move_prim(book_path, new_path)

        parent_world = self.get_world_xf(stage, self._grasp_frame_path)
        local_m = book_world * parent_world.GetInverse()

        local_m2 = Gf.Matrix4d(local_m)
        local_m2.SetTranslateOnly(
            local_m.ExtractTranslation()
            + Gf.Vec3d(
                float(self.attach_offset_in_cup_frame[0]),
                float(self.attach_offset_in_cup_frame[1]),
                float(self.attach_offset_in_cup_frame[2]),
            )
        )

        moved_prim = stage.GetPrimAtPath(new_path)
        xformable = UsdGeom.Xformable(moved_prim)
        xformable.ClearXformOpOrder()
        xformable.AddTransformOp().Set(local_m2)

        self._attached = True
        self._attached_path = new_path

        carb.log_warn(f"[ATTACH] PLACE={self.place} COLOR={self.target_color} : {book_path} -> {new_path}")

    def detach_book_from_cup(self):
        if not self._attached:
            return

        stage = self.stage
        attached_path = self._attached_path
        if not attached_path or not self.prim_exists(stage, attached_path):
            raise RuntimeError("[DETACH] attached prim missing")

        book_world = self.get_world_xf(stage, attached_path)

        dst_path = str(Sdf.Path(self._original_parent_path).AppendChild(self._book_name))
        if self.prim_exists(stage, dst_path):
            raise RuntimeError(f"[DETACH] destination already exists: {dst_path}")

        self._move_prim(attached_path, dst_path)

        parent_world = self.get_world_xf(stage, self._original_parent_path)
        local_m = book_world * parent_world.GetInverse()

        moved_prim = stage.GetPrimAtPath(dst_path)
        xformable = UsdGeom.Xformable(moved_prim)
        xformable.ClearXformOpOrder()
        xformable.AddTransformOp().Set(local_m)

        self.book_path = dst_path
        self._attached = False
        self._attached_path = None

        carb.log_warn(f"[DETACH] PLACE={self.place} COLOR={self.target_color} -> {dst_path}")

    def _apply_arm_deg(self, q_deg):
        q_rad = self.deg2rad(q_deg)
        action = ArticulationAction(joint_positions=q_rad, joint_indices=self.arm_indices)
        self.art.apply_action(action)

    def _hold(self, seconds, q_deg=None):
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < seconds:
            if q_deg is not None:
                self._apply_arm_deg(q_deg)
            self.world.step(render=True)

    def run_sequence(self):
        for c in ("red", "yellow", "blue"):
            p = self.books.get(c, "")
            if p:
                carb.log_warn(f"[CHECK] {c} exists? {self.prim_exists(self.stage, p)} : {p}")

        if not self.prim_exists(self.stage, self.book_path):
            raise RuntimeError(f"[TASK] target book prim not found: {self.book_path}")

        carb.log_warn(f"[RUN] PLACE={self.place}, target_color={self.target_color}, book={self.book_path}")

        carb.log_warn("0) READY")
        self._hold(self.hold_move_s, self.pose_ready_deg)

        carb.log_warn(f"0-1) READY wait {self.ready_before_action_wait_s:.1f}s")
        self._hold(self.ready_before_action_wait_s, self.pose_ready_deg)

        carb.log_warn("1) PICK pose")
        self._hold(self.hold_move_s, self.pose_pick_deg)

        # 아래 시퀀스는 필요할 때 주석 해제
        # carb.log_warn("2) ATTACH")
        # self.attach_book_to_cup()
        # self._hold(self.hold_attach_s, self.pose_pick_deg)

        # carb.log_warn("3) MID (by color)")
        # self._hold(self.hold_move_s, self.pose_mid_deg)

        # carb.log_warn("4) POSE_2SHELF_DEG")
        # self._hold(self.hold_move_s, self.pose_2shelf_deg)

        # carb.log_warn("5) POSE_PLACE2SHELF_DEG")
        # self._hold(self.hold_move_s, self.pose_place2shelf_deg)

        # carb.log_warn("6) DETACH")
        # self.detach_book_from_cup()
        # self._hold(self.hold_detach_s, self.pose_place2shelf_deg)

        # carb.log_warn("7) READY")
        # self._hold(self.hold_move_s, self.pose_ready_deg)

    def _init(self):
        if not os.path.isfile(self.env_usd_path):
            raise FileNotFoundError(self.env_usd_path)

        from isaacsim.core.utils.stage import open_stage

        carb.log_warn(f"[STAGE] open_stage: {self.env_usd_path}")
        open_stage(self.env_usd_path)
        simulation_app.update()

        self.stage = self.get_stage()

        self._assert_prim(self.stage, self.robot_articulation_root, "ROBOT_ARTICULATION_ROOT")
        self._assert_prim(self.stage, self.ee_link_path, "EE_LINK_PATH")

        if self.disable_carter_actiongraphs:
            for ag_path in self.carter_actiongraph_paths:
                ag = self.stage.GetPrimAtPath(ag_path)
                if ag and ag.IsValid():
                    ag.SetActive(False)
                    carb.log_warn(f"[INIT] ActionGraph disabled: {ag_path}")

        if self.graph_ur10:
            og_prim = self.stage.GetPrimAtPath(self.graph_ur10)
            if og_prim and og_prim.IsValid():
                og_prim.SetActive(False)
                carb.log_warn(f"[INIT] OmniGraph disabled: {self.graph_ur10}")

        self.world = World(physics_dt=1 / 60, rendering_dt=1 / 60)

        # 통합 articulation 등록 (carter + ur10)
        self.art = self.world.scene.add(
            SingleArticulation(
                prim_path=self.robot_articulation_root,
                name="carter_ur10",
            )
        )

        self.world.reset()

        self.suction_cup_path = self.resolve_suction_cup_path(self.stage)
        carb.log_warn(f"[CUP] suction_cup path: {self.suction_cup_path}")

        # ur10 조인트 인덱스만 추출
        self.arm_indices = [self.art.get_dof_index(n) for n in self.joint_names]
        carb.log_warn(f"[INIT] ur10 DOF indices: {list(zip(self.joint_names, self.arm_indices))}")

        omni.timeline.get_timeline_interface().play()

        for _ in range(int(self.start_delay_s * 60)):
            self.world.step(render=True)

    def run(self):
        self._init()
        self.run_sequence()

        while simulation_app.is_running():
            self.world.step(render=True)


if __name__ == "__main__":
    app = amr2shelf(
        env_usd_path=ENV_USD_PATH,
        robot_articulation_root=ROBOT_ARTICULATION_ROOT,
        ee_link_path=EE_LINK_PATH,
        graph_ur10=GRAPH_UR10,
        joint_names=JOINT_NAMES,
        books=BOOKS,
        place=PLACE,
        pose_ready_deg=POSE_READY_DEG,
        pose_pick_by_color=POSE_PICK_BY_COLOR,
        pose_mid_by_color=POSE_MID_BY_COLOR,
        pose_2shelf_deg=POSE_2SHELF_DEG,
        pose_place2shelf_deg=POSE_PLACE2SHELF_DEG,
        suction_cup_name=SUCTION_CUP_NAME,
        attach_offset_in_cup_frame=ATTACH_OFFSET_IN_CUP_FRAME,
        start_delay_s=START_DELAY_S,
        ready_before_action_wait_s=READY_BEFORE_ACTION_WAIT_S,
        hold_move_s=HOLD_MOVE_S,
        hold_attach_s=HOLD_ATTACH_S,
        hold_detach_s=HOLD_DETACH_S,
        disable_carter_actiongraphs=DISABLE_CARTER_ACTIONGRAPHS,
        carter_actiongraph_paths=CARTER_ACTIONGRAPH_PATHS,
    )

    try:
        app.run()
    except Exception as e:
        carb.log_error(f"[FATAL] {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()