# pick and place code

import numpy as np
from omni.isaac.dynamic_control import _dynamic_control
from scipy.spatial.transform import Rotation as R
from pxr import UsdPhysics
import omni.usd
import asyncio

class KR210PickAndPlace:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        
        self.art_path = "/Root/run_robot/body/kr210_l150"
        self.art_handle = self.dc.get_articulation(self.art_path)
        if self.art_handle == _dynamic_control.INVALID_HANDLE:
            self.art_path = "/Root/run_robot"
            self.art_handle = self.dc.get_articulation(self.art_path)
            
        self.shoulder_handle = self.dc.get_rigid_body("/Root/run_robot/body/kr210_l150/link_2")
        self.base_handle     = self.dc.get_rigid_body("/Root/run_robot/body/kr210_l150/base_link")
        self.cube_handle     = self.dc.get_rigid_body("/Root/PickCube")
        
        self.dc.wake_up_articulation(self.art_handle)

        self.L_UPPER = 1.25
        self.L_FORE  = 1.30
        
        self.dof_handles = {}
        self._map_and_fix_joints()

    def _map_and_fix_joints(self):
        dof_count = self.dc.get_articulation_dof_count(self.art_handle)
        print(f"\n🔧 DOF 총 개수: {dof_count}")
        for i in range(dof_count):
            dof  = self.dc.get_articulation_dof(self.art_handle, i)
            name = self.dc.get_dof_name(dof)
            if name.startswith("joint_a"):
                self.dof_handles[name] = dof
                dof_path  = self.dc.get_dof_path(dof)
                drive_api = UsdPhysics.DriveAPI.Get(self.stage.GetPrimAtPath(dof_path), "angular")
                if drive_api:
                    drive_api.GetStiffnessAttr().Set(10000000.0)
                    drive_api.GetDampingAttr().Set(1000000.0)
        print(f"   매핑된 조인트: {list(self.dof_handles.keys())}")

    def get_local_target_pos(self, target_handle, label="타겟"):
        base_pose     = self.dc.get_rigid_body_pose(self.base_handle)
        shoulder_pose = self.dc.get_rigid_body_pose(self.shoulder_handle)
        target_pose   = self.dc.get_rigid_body_pose(target_handle)

        base_pos     = np.array([base_pose.p.x,    base_pose.p.y,    base_pose.p.z])
        shoulder_pos = np.array([shoulder_pose.p.x, shoulder_pose.p.y, shoulder_pose.p.z])
        target_pos   = np.array([target_pose.p.x,   target_pose.p.y,  target_pose.p.z])
        base_rot_quat = [base_pose.r.x, base_pose.r.y, base_pose.r.z, base_pose.r.w]

        print(f"\n📍 base_link 월드    : X={base_pos[0]:.3f}, Y={base_pos[1]:.3f}, Z={base_pos[2]:.3f}")
        print(f"💪 shoulder 월드     : X={shoulder_pos[0]:.3f}, Y={shoulder_pos[1]:.3f}, Z={shoulder_pos[2]:.3f}")
        print(f"📦 [{label}] 월드    : X={target_pos[0]:.3f}, Y={target_pos[1]:.3f}, Z={target_pos[2]:.3f}")

        rot_matrix = R.from_quat(base_rot_quat).as_matrix()

        local_from_base     = rot_matrix.T @ (target_pos - base_pos)
        local_from_shoulder = rot_matrix.T @ (target_pos - shoulder_pos)

        print(f"🔄 base 기준 로컬    : X={local_from_base[0]:.3f}, Y={local_from_base[1]:.3f}, Z={local_from_base[2]:.3f}")
        print(f"🔄 shoulder 기준 로컬: X={local_from_shoulder[0]:.3f}, Y={local_from_shoulder[1]:.3f}, Z={local_from_shoulder[2]:.3f}")

        r_horiz = np.sqrt(local_from_shoulder[0]**2 + local_from_shoulder[1]**2)
        reach   = self.L_UPPER + self.L_FORE
        status  = "✅ 도달 가능" if r_horiz <= reach else f"❌ 사정거리 초과"
        print(f"   수평 거리(r)={r_horiz:.3f}  최대 도달={reach:.3f}  {status}")

        return local_from_base, local_from_shoulder

    def solve_ik_kr210(self, local_from_base, local_from_shoulder, is_transit=False, label=""):
        x_b, y_b, _ = local_from_base
        x_s, y_s, z_s = local_from_shoulder

        if is_transit:
            theta1 = np.arctan2(y_b, x_b)
            result = {
                'joint_a1': theta1,
                'joint_a2': -0.5,
                'joint_a3':  0.5,
                'joint_a4':  0.0,
                'joint_a5':  1.57,
                'joint_a6':  0.0
            }
            print(f"\n🔁 [{label}] transit  θ1={np.degrees(theta1):.1f}°")
            return result

        theta1 = np.arctan2(y_b, x_b)
        r      = np.sqrt(x_s**2 + y_s**2)
        z_adj  = z_s

        D     = (r**2 + z_adj**2 - self.L_UPPER**2 - self.L_FORE**2) / (2 * self.L_UPPER * self.L_FORE)
        D_raw = D
        D     = np.clip(D, -1.0, 1.0)

        theta3 = np.arctan2(-np.sqrt(1 - D**2), D)

        theta2_ik = np.arctan2(z_adj, r) - np.arctan2(
            self.L_FORE * np.sin(theta3),
            self.L_UPPER + self.L_FORE * np.cos(theta3)
        )

        # KR210: joint_a2 0도 = 수직 세운 상태, 양수 = 앞으로 숙임
        # IK theta2: 수평 기준(0=수평), KR210은 수직 기준(0=수직)
        # → π/2 에서 빼서 좌표계 맞춤
        theta2 = (np.pi / 2) - theta2_ik

        print(f"\n🦾 [{label}] IK 계산")
        print(f"   shoulder 기준: x={x_s:.3f}, y={y_s:.3f}, z={z_s:.3f}")
        print(f"   r={r:.3f}, z_adj={z_adj:.3f}")
        print(f"   D(raw)={D_raw:.4f}")
        print(f"   θ2_ik={np.degrees(theta2_ik):.2f}°  →  θ2(KR210)={np.degrees(theta2):.2f}°")
        print(f"   θ1={np.degrees(theta1):.2f}°  θ2={np.degrees(theta2):.2f}°  θ3={np.degrees(theta3):.2f}°")

        if abs(D_raw) > 1.0:
            print(f"   ⚠️  도달 불가!")

        return {
            'joint_a1': theta1,
            'joint_a2': theta2,
            'joint_a3': theta3,
            'joint_a4': 0.0,
            'joint_a5': 1.57,
            'joint_a6': 0.0
        }

    def set_targets(self, targets_dict):
        for name, angle in targets_dict.items():
            if name in self.dof_handles:
                self.dc.set_dof_position_target(self.dof_handles[name], angle)
                print(f"   → {name} = {np.degrees(angle):.2f}°")
            else:
                print(f"   ⚠️  {name} 핸들 없음!")

    async def wait_for_move(self, seconds=2.0):
        await asyncio.sleep(seconds)

    async def run(self):
        print("\n" + "="*55)
        print("🚀 KR210 Pick & Place 시퀀스 시작")
        print("="*55)

        base_local, shoulder_local = self.get_local_target_pos(self.cube_handle, label="PickCube")

        print("\n1️⃣  큐브를 향해 팔을 뻗습니다.")
        targets = self.solve_ik_kr210(base_local, shoulder_local, is_transit=False, label="PICK")
        self.set_targets(targets)
        await self.wait_for_move(3.0)

        print("\n2️⃣  충돌 방지: 팔을 접습니다.")
        targets = self.solve_ik_kr210(base_local, shoulder_local, is_transit=True, label="RETRACT")
        self.set_targets(targets)
        await self.wait_for_move(2.0)

        place_base     = np.array([-2.0, 1.0, 0.0])
        place_shoulder = np.array([-2.0, 1.0, -0.744])

        print("\n3️⃣  목적지를 향해 베이스를 돌립니다.")
        targets = self.solve_ik_kr210(place_base, place_shoulder, is_transit=True, label="TRANSIT_PLACE")
        self.set_targets(targets)
        await self.wait_for_move(2.0)

        print("\n4️⃣  목적지로 팔을 뻗어 내려놓습니다.")
        targets = self.solve_ik_kr210(place_base, place_shoulder, is_transit=False, label="PLACE")
        self.set_targets(targets)
        await self.wait_for_move(2.5)

        print("\n" + "="*55)
        print("✅ 시퀀스 완료!")
        print("="*55)

controller = KR210PickAndPlace()
asyncio.ensure_future(controller.run())