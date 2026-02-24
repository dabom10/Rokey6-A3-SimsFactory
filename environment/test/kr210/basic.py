# pick and place code

import numpy as np
from omni.isaac.dynamic_control import _dynamic_control
from scipy.spatial.transform import Rotation as R
from pxr import UsdPhysics, Gf
import omni.usd
import omni.kit.app
import asyncio

class KR210PickAndPlace:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        # articulation root 탐색
        self.art_handle = None
        for candidate in ["/Root/run_robot/body", "/Root/run_robot"]:
            self.art_path = candidate
            handle = self.dc.get_articulation(candidate)
            if handle != _dynamic_control.INVALID_HANDLE:
                self.art_handle = handle
                print(f"✅ articulation 핸들 획득: {candidate}")
                break
        if self.art_handle is None:
            print("❌ articulation 핸들 획득 실패 — Play 상태 확인 필요")
            return

        self.shoulder_handle = self.dc.get_rigid_body("/Root/run_robot/body/kr210_l150/link_2")
        self.base_handle     = self.dc.get_rigid_body("/Root/run_robot/body/kr210_l150/base_link")
        self.ee_handle       = self.dc.get_rigid_body("/Root/run_robot/body/kr210_l150/tool0")

        # 큐브 경로 탐색
        self.cube_handle = None
        for cube_path in ["/Root/PickCube", "/BlackGrid/Grid_Cubes/Cube_04"]:
            handle = self.dc.get_rigid_body(cube_path)
            if handle != _dynamic_control.INVALID_HANDLE:
                print(f"✅ 큐브 핸들 획득: {cube_path}")
                self.cube_handle = handle
                break
        if self.cube_handle is None:
            print("❌ 큐브 핸들 획득 실패 — 경로 확인 필요")

        self.dc.wake_up_articulation(self.art_handle)

        self.L_UPPER = 1.25
        self.L_FORE  = 1.30

        self.dof_handles = {}
        self.wheel_dofs  = []
        self._map_and_fix_joints()

    def _map_and_fix_joints(self):
        dof_count = self.dc.get_articulation_dof_count(self.art_handle)
        print(f"\n🔧 DOF 총 개수: {dof_count}")

        for i in range(dof_count):
            dof      = self.dc.get_articulation_dof(self.art_handle, i)
            name     = self.dc.get_dof_name(dof)
            dof_path = self.dc.get_dof_path(dof)
            prim     = self.stage.GetPrimAtPath(dof_path)

            if name.startswith("joint_a"):
                self.dof_handles[name] = dof
                drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
                if not drive_api:
                    drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive_api.GetStiffnessAttr().Set(1e7)
                drive_api.GetDampingAttr().Set(1e6)
                print(f"   ✅ {name} stiffness=1e7 damping=1e6")

            elif name.startswith("wheel"):
                self.wheel_dofs.append(dof)
                # DriveAPI.Apply 하지 않음 — USD에 쓰면 Stop→Play 후에도
                # Drive가 남아서 차체가 저절로 움직이는 현상 발생
                # velocity target 0만 매 프레임 주는 방식으로 브레이크
                self.dc.set_dof_velocity_target(dof, 0.0)
                print(f"   🛑 {name} velocity_target=0 (Drive USD 미기록)")

        print(f"   매핑된 KR210 조인트: {list(self.dof_handles.keys())}")
        self._fix_kr210_mass()

    def _fix_kr210_mass(self):
        link_masses = {
            "/Root/run_robot/body/kr210_l150/base_link": 150.0,
            "/Root/run_robot/body/kr210_l150/Link1":      80.0,
            "/Root/run_robot/body/kr210_l150/link_2":    120.0,
            "/Root/run_robot/body/kr210_l150/link_3":     60.0,
            "/Root/run_robot/body/kr210_l150/link_4":     40.0,
            "/Root/run_robot/body/kr210_l150/link_5":     20.0,
            "/Root/run_robot/body/kr210_l150/link_6":     10.0,
            "/Root/run_robot/body/kr210_l150/tool0":       2.0,
        }
        for path, mass in link_masses.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            sdf_path = prim.GetPath()
            mass_api = UsdPhysics.MassAPI.Get(self.stage, sdf_path)
            if not mass_api:
                mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.GetMassAttr().Set(mass)
            inertia = mass * 0.1
            mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(inertia, inertia, inertia))
            print(f"   ⚙️  {path.split('/')[-1]} mass={mass}kg inertia={inertia:.1f}")

    def _brake_wheels(self):
        """매 프레임 바퀴 velocity target 0 유지"""
        for dof in self.wheel_dofs:
            self.dc.set_dof_velocity_target(dof, 0.0)

    def print_ee_error(self, label=""):
        """EE와 큐브 간 실제 오차 출력 — X/Y 튜닝용"""
        if self.ee_handle is None or self.cube_handle is None:
            return
        ee_pose   = self.dc.get_rigid_body_pose(self.ee_handle)
        cube_pose = self.dc.get_rigid_body_pose(self.cube_handle)
        ee_pos    = np.array([ee_pose.p.x,   ee_pose.p.y,   ee_pose.p.z])
        cube_pos  = np.array([cube_pose.p.x, cube_pose.p.y, cube_pose.p.z])
        diff      = cube_pos - ee_pos
        dist_xy   = np.sqrt(diff[0]**2 + diff[1]**2)
        print(f"\n📏 [{label}] EE ↔ 큐브 오차")
        print(f"   EE   : X={ee_pos[0]:.3f}, Y={ee_pos[1]:.3f}, Z={ee_pos[2]:.3f}")
        print(f"   큐브 : X={cube_pos[0]:.3f}, Y={cube_pos[1]:.3f}, Z={cube_pos[2]:.3f}")
        print(f"   diff : dX={diff[0]:.3f}, dY={diff[1]:.3f}, dZ={diff[2]:.3f}")
        print(f"   수평 오차(XY): {dist_xy:.3f}m")

    def get_local_target_pos(self, target_handle, label="타겟"):
        base_pose     = self.dc.get_rigid_body_pose(self.base_handle)
        shoulder_pose = self.dc.get_rigid_body_pose(self.shoulder_handle)
        target_pose   = self.dc.get_rigid_body_pose(target_handle)

        base_pos      = np.array([base_pose.p.x,     base_pose.p.y,     base_pose.p.z])
        shoulder_pos  = np.array([shoulder_pose.p.x,  shoulder_pose.p.y,  shoulder_pose.p.z])
        target_pos    = np.array([target_pose.p.x,    target_pose.p.y,    target_pose.p.z])
        base_rot_quat = [base_pose.r.x, base_pose.r.y, base_pose.r.z, base_pose.r.w]

        print(f"\n📍 base_link 월드    : X={base_pos[0]:.3f}, Y={base_pos[1]:.3f}, Z={base_pos[2]:.3f}")
        print(f"💪 shoulder 월드     : X={shoulder_pos[0]:.3f}, Y={shoulder_pos[1]:.3f}, Z={shoulder_pos[2]:.3f}")
        print(f"📦 [{label}] 월드    : X={target_pos[0]:.3f}, Y={target_pos[1]:.3f}, Z={target_pos[2]:.3f}")

        rot_matrix          = R.from_quat(base_rot_quat).as_matrix()
        local_from_base     = rot_matrix.T @ (target_pos - base_pos)
        local_from_shoulder = rot_matrix.T @ (target_pos - shoulder_pos)

        print(f"🔄 base 기준 로컬    : X={local_from_base[0]:.3f}, Y={local_from_base[1]:.3f}, Z={local_from_base[2]:.3f}")
        print(f"🔄 shoulder 기준 로컬: X={local_from_shoulder[0]:.3f}, Y={local_from_shoulder[1]:.3f}, Z={local_from_shoulder[2]:.3f}")

        r_horiz = np.sqrt(local_from_shoulder[0]**2 + local_from_shoulder[1]**2)
        reach   = self.L_UPPER + self.L_FORE
        status  = "✅ 도달 가능" if r_horiz <= reach else "❌ 사정거리 초과"
        print(f"   수평 거리(r)={r_horiz:.3f}  최대 도달={reach:.3f}  {status}")

        return local_from_base, local_from_shoulder

    def solve_ik_kr210(self, local_from_base, local_from_shoulder, is_transit=False, label="", z_offset=0.0):
        x_b, y_b, _   = local_from_base
        x_s, y_s, z_s = local_from_shoulder

        if is_transit:
            theta1 = np.arctan2(y_b, x_b)
            print(f"\n🔁 [{label}] transit  θ1={np.degrees(theta1):.1f}°")
            return {
                'joint_a1': theta1,
                'joint_a2': -0.5,
                'joint_a3':  0.5,
                'joint_a4':  0.0,
                'joint_a5':  1.57,
                'joint_a6':  0.0
            }

        theta1    = np.arctan2(y_b, x_b)
        r         = np.sqrt(x_s**2 + y_s**2)
        z_adj     = z_s + z_offset  # 양수=위에서 접근, 음수=아래로 내려감
        D_raw     = (r**2 + z_adj**2 - self.L_UPPER**2 - self.L_FORE**2) / (2 * self.L_UPPER * self.L_FORE)
        D         = np.clip(D_raw, -1.0, 1.0)
        theta3    = np.arctan2(-np.sqrt(1 - D**2), D)
        theta2_ik = np.arctan2(z_adj, r) - np.arctan2(
            self.L_FORE * np.sin(theta3),
            self.L_UPPER + self.L_FORE * np.cos(theta3)
        )
        theta2 = (np.pi / 2) - theta2_ik

        print(f"\n🦾 [{label}] IK 계산")
        print(f"   shoulder 기준: x={x_s:.3f}, y={y_s:.3f}, z={z_s:.3f}")
        print(f"   r={r:.3f}, z_adj={z_adj:.3f}, D(raw)={D_raw:.4f}")
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

    async def lerp_to(self, targets_dict, steps=60):
        """프레임마다 조금씩 target을 올려서 순간 토크 폭발 방지"""
        app = omni.kit.app.get_app()

        current = {}
        for name, dof in self.dof_handles.items():
            state = self.dc.get_dof_state(dof, _dynamic_control.STATE_POS)
            current[name] = state.pos

        for step in range(1, steps + 1):
            t = step / steps
            for name, target_angle in targets_dict.items():
                if name not in self.dof_handles:
                    continue
                interp = current[name] + (target_angle - current[name]) * t
                self.dc.set_dof_position_target(self.dof_handles[name], interp)
            self._brake_wheels()
            await app.next_update_async()

        print(f"   lerp 완료 ({steps} frames)")

    async def settle(self, frames=40):
        """관성 안정화 대기"""
        app = omni.kit.app.get_app()
        for _ in range(frames):
            self._brake_wheels()
            await app.next_update_async()

    async def run(self):
        print("\n" + "="*55)
        print("🚀 KR210 Pick & Place 시퀀스 시작")
        print("="*55)

        base_local, shoulder_local = self.get_local_target_pos(self.cube_handle, label="PickCube")

        # z_offset: 양수일수록 EE가 높은 위치 목표
        # 줄이면 더 아래로 내려감
        CUBE_HALF  = 0.10  # PICK 시 EE Z offset
        PRE_OFFSET = 0.50  # PRE-PICK 시 위 여유 높이

        print("\n1️⃣  Pre-Pick: 큐브 위로 접근합니다.")
        targets = self.solve_ik_kr210(base_local, shoulder_local, is_transit=False,
                                      label="PRE-PICK", z_offset=PRE_OFFSET)
        await self.lerp_to(targets, steps=90)
        await self.settle(60)

        print("\n2️⃣  Pick: 큐브로 내려갑니다.")
        targets = self.solve_ik_kr210(base_local, shoulder_local, is_transit=False,
                                      label="PICK", z_offset=CUBE_HALF)
        await self.lerp_to(targets, steps=60)
        await self.settle(40)

        # PICK 완료 후 EE ↔ 큐브 실측 오차 출력
        # X/Y 방향으로 diff가 크면 L_FORE 또는 L_UPPER 튜닝 필요
        self.print_ee_error(label="PICK 완료")

        print("\n3️⃣  충돌 방지: 팔을 접습니다.")
        targets = self.solve_ik_kr210(base_local, shoulder_local, is_transit=True, label="RETRACT")
        await self.lerp_to(targets, steps=60)
        await self.settle(40)

        place_base     = np.array([-2.0, 1.0, 0.0])
        place_shoulder = np.array([-2.0, 1.0, -0.744])

        print("\n4️⃣  목적지를 향해 베이스를 돌립니다.")
        targets = self.solve_ik_kr210(place_base, place_shoulder, is_transit=True, label="TRANSIT_PLACE")
        await self.lerp_to(targets, steps=60)
        await self.settle(40)

        print("\n5️⃣  목적지로 팔을 뻗어 내려놓습니다.")
        targets = self.solve_ik_kr210(place_base, place_shoulder, is_transit=False, label="PLACE")
        await self.lerp_to(targets, steps=90)
        await self.settle(60)

        print("\n" + "="*55)
        print("✅ 시퀀스 완료!")
        print("="*55)

controller = KR210PickAndPlace()
asyncio.ensure_future(controller.run())