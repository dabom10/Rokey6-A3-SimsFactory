import numpy as np
from scipy.spatial.transform import Rotation as R
import omni.usd

from isaacsim.examples.interactive.base_sample import BaseSample
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path, delete_prim
from omni.isaac.core.utils.viewports import set_camera_view

class KR210IK:
    def __init__(self, robot_prim_path):
        self.L_UPPER = 1.25
        self.L_FORE  = 1.30
        self.base_prim = XFormPrim(f"{robot_prim_path}/body/kr210_l150/base_link")
        self.shoulder_prim = XFormPrim(f"{robot_prim_path}/body/kr210_l150/link_2")

    def solve(self, target_pos_world, is_transit=False, z_offset=0.0):
        base_pos, base_quat = self.base_prim.get_world_pose()
        shoulder_pos, _ = self.shoulder_prim.get_world_pose()
        
        qw, qx, qy, qz = base_quat
        rot_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()
        
        local_from_base = rot_matrix.T @ (target_pos_world - base_pos)
        local_from_shoulder = rot_matrix.T @ (target_pos_world - shoulder_pos)

        x_b, y_b, _ = local_from_base
        x_s, y_s, z_s = local_from_shoulder
        
        if is_transit:
            theta1 = np.arctan2(y_b, x_b)
            return np.array([theta1, -0.5, 0.5, 0.0, 1.57, 0.0])
            
        theta1 = np.arctan2(y_b, x_b)
        r = np.sqrt(x_s**2 + y_s**2)
        z_adj = z_s + z_offset
        
        D_raw = (r**2 + z_adj**2 - self.L_UPPER**2 - self.L_FORE**2) / (2 * self.L_UPPER * self.L_FORE)
        D = np.clip(D_raw, -1.0, 1.0)
        
        theta3_math = np.arctan2(np.sqrt(1 - D**2), D)
        theta2_ik = np.arctan2(z_adj, r) - np.arctan2(
            self.L_FORE * np.sin(theta3_math),
            self.L_UPPER + self.L_FORE * np.cos(theta3_math)
        )
        
        return np.array([theta1, (np.pi / 2) - theta2_ik, -theta3_math, 0.0, 1.57, 0.0])


class KR210_ConveyorSample(BaseSample):
    def __init__(self):
        super().__init__()
        self.task_phase = 0
        self.wait_counter = 0
        self.book_scale = np.array([0.45509, 0.08313, 0.5]) 
        self.place_offset = np.array([-1.2, 0.0, 0.5])
        
        self.env_usd_path = "/home/kyb/Rokey6-A3-SimsFactory/environment/robot/test/kr210/basic.usd"
        self.robot_prim_path = "/Root/run_robot"

    def setup_scene(self):
        world = self.get_world()
        stage = omni.usd.get_context().get_stage()
        
        # 1. 서브레이어로 환경 통째로 로드
        stage.GetRootLayer().subLayerPaths.append(self.env_usd_path)

        # 2. 옴니그래프 완전 삭제 (주행 차단)
        if get_prim_at_path(f"{self.robot_prim_path}/Graphs"):
            delete_prim(f"{self.robot_prim_path}/Graphs")

        # 3. 책 스폰
        start_pos = np.array([4.5, -14.8, 3])
        
        world.scene.add(DynamicCuboid(
            prim_path="/World/RedBook", name="red_book",
            position=start_pos, scale=self.book_scale, color=np.array([1, 0, 0]), mass=1.0
        ))
        world.scene.add(DynamicCuboid(
            prim_path="/World/BlueBook", name="blue_book",
            position=start_pos + np.array([0, -1.0, 0]), scale=self.book_scale, color=np.array([0, 0, 1]), mass=1.0
        ))
        world.scene.add(DynamicCuboid(
            prim_path="/World/YellowBook", name="yellow_book",
            position=start_pos + np.array([0, -2.0, 0]), scale=self.book_scale, color=np.array([1, 1, 0]), mass=1.0
        ))

    async def setup_post_load(self):
        self._world = self.get_world()
        self.red_book = self._world.scene.get_object("red_book")
        
        self.robot = Articulation(prim_path=self.robot_prim_path, name="kr210")
        self._world.scene.add(self.robot)
        
        # 물리 엔진이 플레이 상태로 넘어갈 때까지 대기
        await self._world.play_async()
        
        # ✅ 플레이 상태 진입 직후! 여기서 카메라 뷰를 덮어씌워야 초기화되지 않습니다.
        set_camera_view(eye=np.array([4.1784, -6.2238, 4.7902]), target=np.array([2.3528, -15.7560, 2.3813]))

        self.robot.initialize()
        
        self.ik_solver = KR210IK(self.robot_prim_path)
        
        # 관절 및 바퀴 인덱스 캐싱
        joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        self.joint_indices = [self.robot.get_dof_index(n) for n in joint_names]
        
        wheel_names = ['wheel_back_left', 'wheel_front_right', 'wheel_front_left', 'wheel_back_right']
        self.wheel_indices = [self.robot.get_dof_index(n) for n in wheel_names]
        
        # 시작할 때의 바퀴 위치를 기억해서 주차 브레이크로 사용
        self.initial_wheel_pos = self.robot.get_joint_positions()[self.wheel_indices]
        
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        
        self.task_phase = 1
        self.wait_counter = 0
        self.prev_pos = self.red_book.get_world_pose()[0]

    def move_to_target(self, target_joints, speed=0.03):
        """팔 관절 이동과 바퀴 브레이크를 하나의 Action으로 합쳐서 전송!"""
        current_joints = self.robot.get_joint_positions()[self.joint_indices]
        diff = target_joints - current_joints
        dist = np.linalg.norm(diff)
        
        if dist < 0.05:
            action = ArticulationAction(
                joint_positions=np.concatenate([target_joints, self.initial_wheel_pos]), 
                joint_indices=self.joint_indices + self.wheel_indices
            )
            self.robot.apply_action(action)
            return True 
            
        step = diff * (speed / dist)
        next_joints = current_joints + step if np.linalg.norm(step) < dist else target_joints
        
        action = ArticulationAction(
            joint_positions=np.concatenate([next_joints, self.initial_wheel_pos]), 
            joint_indices=self.joint_indices + self.wheel_indices
        )
        self.robot.apply_action(action)
        return False

    def physics_step(self, step_size):
        if self.task_phase == 0:
            return

        # 1. 빨간 책 대기
        if self.task_phase == 1:
            self.target_joints = self.ik_solver.solve(np.zeros(3), is_transit=True)
            self.move_to_target(self.target_joints)
            
            curr_pos = self.red_book.get_world_pose()[0]
            moved = np.linalg.norm(curr_pos - self.prev_pos)
            
            if moved < 0.005:
                self.wait_counter += 1
            else:
                self.wait_counter = 0
            self.prev_pos = curr_pos
            
            if self.wait_counter > 120: 
                print("✅ 빨간 책 도착! Pick 동작 시작")
                self.target_joints = self.ik_solver.solve(curr_pos, z_offset=0.57)
                self.task_phase = 2

        # 2. Pick 위치로 이동
        elif self.task_phase == 2:
            if self.move_to_target(self.target_joints):
                self.wait_counter = 0
                self.task_phase = 3
                
        # 3. 임시 대기 (그리퍼 닫음 대기)
        elif self.task_phase == 3:
            self.wait_counter += 1
            if self.wait_counter > 60:
                print("✅ Pick 완료! 팔 접기...")
                curr_pos = self.red_book.get_world_pose()[0]
                self.target_joints = self.ik_solver.solve(curr_pos, is_transit=True)
                self.task_phase = 4

        # 4. 팔 접기 (Retract)
        elif self.task_phase == 4:
            if self.move_to_target(self.target_joints):
                print("✅ Retract 완료! 차체 뒤로 회전...")
                body = XFormPrim(f"{self.robot_prim_path}/body/body")
                b_pos, b_quat = body.get_world_pose()
                b_rot = R.from_quat([b_quat[1], b_quat[2], b_quat[3], b_quat[0]])
                
                self.place_world_pos = b_pos + b_rot.apply(self.place_offset)
                self.target_joints = self.ik_solver.solve(self.place_world_pos, is_transit=True)
                self.task_phase = 5

        # 5. 뒤로 회전 (Transit)
        elif self.task_phase == 5:
            if self.move_to_target(self.target_joints):
                print("✅ Transit 완료! 내려놓기...")
                self.target_joints = self.ik_solver.solve(self.place_world_pos, z_offset=0.57)
                self.task_phase = 6
                
        # 6. 내려놓기 (Place)
        elif self.task_phase == 6:
            if self.move_to_target(self.target_joints):
                self.wait_counter = 0
                self.task_phase = 7

        # 7. 임시 대기 (그리퍼 열기 대기)
        elif self.task_phase == 7:
            self.wait_counter += 1
            if self.wait_counter > 60:
                print("✅ Place 완료! 대기 자세로 복귀...")
                self.target_joints = self.ik_solver.solve(np.zeros(3), is_transit=True)
                self.task_phase = 8
                
        # 8. 대기 자세 복귀
        elif self.task_phase == 8:
            if self.move_to_target(self.target_joints):
                print("🎉 시퀀스 1회 완료!")
                self.task_phase = 0