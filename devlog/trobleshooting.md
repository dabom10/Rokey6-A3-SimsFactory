# 🦾 매니퓰레이터 트러블슈팅 총정리 (STAR 기법)

> Isaac Sim 5.0.0 | UR10 + KR210 | 2026.02.21 ~ 02.26  
> 해결된 이슈 → **Action 상세 기록** / 미해결 이슈 → **인사이트 & 시도해볼 방법 기록**

---

## 목차

| # | 이슈명 | 상태 |
|---|--------|------|
| 01 | Articulation 분리 현상 | ✅ 해결 |
| 02 | DOF 인덱스 비표준 순서 | ✅ 해결 |
| 03 | Segfault — numpy array 타입 불일치 | ✅ 해결 |
| 04 | IK 역관절 현상 (joint_a2 부호 오류) | ✅ 해결 |
| 05 | IK Elbow-In 현상 (joint_a3 축 방향 반전) | ✅ 해결 |
| 06 | World vs Local 좌표계 혼동 | ✅ 해결 |
| 07 | Physics Bouncing (로봇 튀어오름) | ✅ 해결 |
| 08 | 차체 심한 흔들림 (Physics Instability) | ✅ 해결 |
| 09 | DriveAPI.Apply() USD 영구 기록 문제 | ✅ 해결 |
| 10 | Articulation 초기화 타이밍 오류 | ✅ 해결 |
| 11 | ArticulationRoot 경로 오탐 (SingleManipulator → SingleArticulation) | ✅ 해결 |
| 12 | Fake Grasp — SetParent 시 큐브 순간이동 | ✅ 해결 |
| 13 | OpenCV HSV 색상 인식 실패 (float32 미변환) | ✅ 해결 |
| 14 | Phase 1→2 전환 시 책 낙하 | ✅ 해결 |
| 15 | 라이다 피자 조각 현상 | ✅ 해결 |
| 16 | 컨베이어 끝 큐브 포즈 변형 | 🔴 미해결 |
| 17 | Pick 위치까지 이동 실패 (RMPFlow 좌표계 불일치) | 🔴 미해결 |

---

---

## ✅ 해결된 이슈

---

### 01. Articulation 분리 현상

**Situation**  
UR10에 흡착 그리퍼(short_gripper)를 USD에서 붙이고 Play를 누르면, 로봇팔과 그리퍼가 각각 공중으로 분리되어 날아가 버렸다.

**Task**  
UR10 + 그리퍼를 물리적으로 하나의 단일 로봇으로 Isaac Sim이 인식하도록 통합해야 했다.

**Action**
1. **원인 파악**: USD Stage를 확인하니 UR10 prim과 short_gripper prim이 각각 독립된 `ArticulationRootAPI`를 가지고 있었다. Isaac Sim은 ArticulationRoot가 2개 이상이면 별개의 물리 객체로 처리한다.
2. **short_gripper의 RigidBodyAPI 제거**: short_gripper는 독립 강체로 동작할 필요가 없으므로 `RigidBodyAPI`를 Property 패널에서 삭제했다. ee_link의 자식 메쉬로만 부착되도록 처리.
3. **UR10 prim을 단일 Root 하위로 통합**: UR10 prim을 `run_robot` 하위로 이동시켜 전체가 `/Root/run_robot` 단일 Articulation Root 아래 들어오도록 계층을 재편했다.
4. **중복 PhysicsScene 제거**: USD 씬 내에 PhysicsScene이 2개 이상 존재하면 물리 계산 충돌이 발생하므로 중복 노드 제거.
5. **Play 후 분리 현상 사라짐을 확인**.

**Result**  
`/Root/run_robot` 단일 Articulation Tree가 유지되고, Play 시 UR10 + 그리퍼가 하나의 로봇으로 일체 동작.

> **핵심 인사이트**: 복합 로봇(베이스 + 팔 + 그리퍼) 구성 시 부품 하나를 붙일 때마다 `RigidBodyAPI`와 `ArticulationRootAPI` 존재 여부를 Stage에서 확인하는 것이 필수. Stage Tree와 Physical Kinematic Chain은 별개다.

---

### 02. DOF 인덱스 비표준 순서

**Situation**  
UR10 관절 제어 시 코드에서 어깨 관절 인덱스를 0번으로 가정하고 값을 넣었는데, 실제로는 엉뚱한 관절이 움직였다. `shoulder_pan_joint`가 0번이 아니라 `shoulder_lift_joint`가 0번이었고, `wrist_3_joint`가 바퀴 조인트들 뒤인 9번에 배치되어 있었다.

**Task**  
전체 10 DOF(UR10 6개 + 바퀴 4개)의 실제 인덱스 순서를 확정하고, 코드에 올바르게 매핑해야 했다.

**Action**
1. **`search_DOF.py` 작성**: Stage를 Traverse하면서 Articulation 하위 Joint들을 순서대로 출력하는 진단 스크립트를 작성했다.
2. **실제 인덱스 전수 확인**: 실행 결과로 아래 매핑을 확정했다.

```
[0] shoulder_lift_joint  (UR10) ← 비표준: 0번이 lift
[1] elbow_joint          (UR10)
[2] shoulder_pan_joint   (UR10)
[3] wrist_1_joint        (UR10)
[4] wrist_2_joint        (UR10)
[5] wheel_back_left      (바퀴)
[6] wheel_front_right    (바퀴)
[7] wheel_back_right     (바퀴)
[8] wheel_front_left     (바퀴)
[9] wrist_3_joint        (UR10) ← 비표준: 바퀴 뒤에 위치
```

3. **상수로 분리**:
```python
UR10_IDX  = [0, 1, 2, 3, 4, 9]
WHEEL_IDX = [5, 6, 7, 8]
```

**Result**  
관절 제어 명령이 의도한 조인트에 정확히 전달됨. DOF 인덱스를 추측하지 않고 항상 실측하는 패턴 확립.

> **핵심 인사이트**: DOF 인덱스는 USD 파일 제작 순서나 URDF 표준과 다를 수 있다. 새 로봇 파일을 쓸 때마다 반드시 진단 스크립트로 실제 순서를 확인할 것. `/Root/run_robot/ur10` 경로로 접근하면 `INVALID_HANDLE`이 반환되므로 반드시 `/Root/run_robot/body`(ArticulationRoot 경로)로 접근해야 한다.

---

### 03. Segfault — numpy array 타입 불일치

**Situation**  
`set_articulation_dof_position_targets()`에 numpy array를 직접 전달했더니 `libomni.physx.plugin.so`에서 vector reallocation 에러가 발생하며 Core Dump로 프로세스 자체가 죽었다.

**Task**  
Isaac Sim 5.0의 PhysX C++ 바인딩이 받아들일 수 있는 타입으로 데이터를 변환하여 안전하게 전달해야 했다.

**Action**
1. **원인 파악**: PhysX C++ 바인딩에 numpy array를 직접 넘길 경우 내부에서 타입 불일치로 메모리 접근 오류가 발생함을 확인.
2. **Python list로 변환 후 전달**:

```python
# ❌ Segfault 발생
ctrl.set_articulation_dof_position_targets(art, np.array([...]))

# ✅ 안전 — 현재 상태를 list로 읽고 원하는 인덱스만 수정
target = list(all_states["pos"])
target[idx] = float(val)
ctrl.set_articulation_dof_position_targets(art, target)
```

**Result**  
Segfault 없이 안정적으로 관절 타겟 전달 가능. float 변환도 명시적으로 처리.

---

### 04. IK 역관절 현상 (joint_a2 부호 오류)

**Situation**  
KR210으로 Pick 동작을 실행하면 어깨 관절(`joint_a2`)이 목표물 방향이 아닌 반대 방향으로 꺾이면서 팔이 몸통 뒤로 뒤집혔다. 콘솔에서 EE와 큐브의 Z 오차를 찍어보니 -2.248m가 출력됐다.

**Task**  
IK 수식의 부호 오류를 찾아 교정하고, EE가 목표 큐브 윗면을 정확히 향하도록 해야 했다.

**Action**
1. **`print_ee_error()` 함수 작성**: EE 위치, 큐브 위치, dX/dY/dZ 오차, 수평 오차를 매 프레임 출력하는 디버깅 루틴 작성.

```python
def print_ee_error(self, label=""):
    ee_pos   = self.dc.get_rigid_body_pose(self.ee_handle).p
    cube_pos = self.dc.get_rigid_body_pose(self.cube_handle).p
    diff = np.array(cube_pos) - np.array(ee_pos)
    print(f"[{label}] dX={diff[0]:.3f}, dY={diff[1]:.3f}, dZ={diff[2]:.3f}")
    print(f"수평 오차: {np.sqrt(diff[0]**2 + diff[1]**2):.3f}m")
```

2. **theta2 부호 교정**:

```python
# 수정 전 — EE가 위로 솟는 현상
theta2 = (np.pi / 2) - theta2_ik

# 수정 후 — 부호 반전
theta2 = -((np.pi / 2) - theta2_ik)
```

3. **z_offset 파라미터 도입**: IK 입력에 z_offset을 추가해 큐브 중심이 아닌 윗면을 타겟으로 조준하도록 수정.

```python
# Pre-Pick: 큐브 위 0.5m에서 접근
pre_pick = compute_ik(cube_pos, base_pos, z_offset=+0.5)
# Pick: 큐브 윗면 높이로 하강
pick     = compute_ik(cube_pos, base_pos, z_offset=-0.3)
```

**Result**  
EE가 목표 방향으로 정상적으로 뻗으며, 수평 오차가 3.3m → 허용 범위 수준으로 감소.

> **핵심 인사이트**: IK 수식이 수학적으로 옳아도, USD 조인트의 회전 방향(+/-)이 제조사/모델별로 다르다. 관절이 이상하게 꺾이면 수식보다 부호를 먼저 점검할 것. EE 오차를 숫자로 출력하는 루틴 없이는 시각적으로 "대충 맞는 것 같다"는 착각에 빠지기 쉽다.

---

### 05. IK Elbow-In 현상 (joint_a3 축 방향 반전)

**Situation**  
theta2 부호 교정 이후에도 팔꿈치(`joint_a3`)가 바깥쪽이 아닌 로봇 몸통 쪽으로 심하게 접히는 현상이 남았다. 수평 오차가 0.8m 이상 발생했다.

**Task**  
KR210의 USD 조인트 축 기준을 파악하고 elbow 관절의 방향을 올바르게 맞춰야 했다.

**Action**
1. **원인 파악**: 수학적 IK는 Elbow-Up 자세(팔꿈치가 위를 향하는 방향)로 양수(+) 각도를 계산한다. 그런데 KR210의 USD joint axis 설계가 이 수학적 방향과 정반대로 설정되어 있었다.
2. **모터 값 전달 단계에서 부호 추가**: IK 수식 자체를 건드리지 않고, 최종 관절 값 전달 직전에만 부호를 뒤집었다.

```python
# KR210 하드웨어 축 기준에 동기화
joint_a3 = -theta3_math
```

**Result**  
팔꿈치가 바깥쪽으로 정상적으로 뻗으면서 수평 오차가 허용 범위로 감소.

> **핵심 인사이트**: UR10과 KR210은 같은 6축 매니퓰레이터이지만 축 방향 기준이 다르다. 로봇을 교체할 때마다 각 관절의 양수 방향이 어디인지 반드시 육안으로 확인하고 부호를 검증할 것. 수식을 고치기보다 최종 전달 단계에서 부호를 관리하면 수식의 재사용성이 유지된다.

---

### 06. World vs Local 좌표계 혼동

**Situation**  
카메라가 인식한 큐브 위치 좌표를 IK에 그대로 넣었더니 EE가 약 0.8m 엉뚱한 곳으로 이동했다. 로봇이 원점이 아닌 특정 위치에 Spawn되어 있었는데, 이를 고려하지 않았던 것이다.

**Task**  
World Frame 좌표를 UR10 Base Frame 기준으로 올바르게 변환하는 파이프라인을 구축해야 했다.

**Action**
1. **원인 정확히 파악**:
   - UI Property 창에서 보이는 값 = **Local 좌표** (부모 Prim 기준 상대값)
   - `get_world_pose()` 반환값 = **World 좌표** (절대값)
   - IK Solver는 UR10 Base Frame을 원점으로 계산하므로, World 좌표를 그대로 넣으면 오차 발생

2. **Base Frame 기준 좌표 변환 코드 추가**:

```python
import numpy as np

cube_pos, _ = cube_prim.get_world_pose()
base_pos, _ = base_prim.get_world_pose()

# World → UR10 Base Frame 변환 (회전이 없는 단순화 케이스)
local_target_pos = cube_pos - base_pos

# Pre-grasp: Z축 0.1m 위
pre_grasp_pos = local_target_pos + np.array([0.0, 0.0, 0.1])
```

3. **디버깅 6종 필수 출력 패턴 도입**:

```python
print(f"[DEBUG] Target World Pos   : {target_pos}")
print(f"[DEBUG] Robot Base Pos     : {base_pos}")
print(f"[DEBUG] Current EE Pos     : {ee_pos}")
print(f"[DEBUG] RMPFlow Tgt Joints : {target_joints}")
print(f"[DEBUG] Current Joints     : {current_joints}")
print(f"[DEBUG] Joint Error        : {target_joints - current_joints}")
```

**Result**  
좌표 변환 적용 후 EE가 목표 위치 10mm 이내로 이동. 이후 모든 좌표 관련 코드에서 World/Local 구분을 명시적으로 표기하는 컨벤션 정착.

> **핵심 인사이트**: Perception(카메라) = Camera Frame, Manipulation(UR10) = Base Frame, Navigation = World Frame. 이 3개의 좌표계 변환 관계를 명시적으로 코드화하지 않으면, 인식은 성공해도 동작은 항상 엉뚱한 곳으로 향한다.

---

### 07. Physics Bouncing (로봇 튀어오름)

**Situation**  
`set_joint_positions()`로 관절 위치를 강제로 변경한 직후 Play를 누르면, 로봇이 마치 스프링처럼 튀어오르며 시뮬레이션이 불안정해졌다.

**Task**  
강제 위치 변경 후 물리 엔진이 안정화될 때까지 대기하는 패턴을 구현해야 했다.

**Action**
1. **원인 파악**: USD의 정적 속성(Xform, Joint Position)을 강제로 변경하면 물리 엔진이 "이 프레임에서 무한대의 속도로 이동했다"로 해석해 순간 충격(Spike)이 발생한다.
2. **60프레임 안정화 루프 도입**: 관절 속도를 매 프레임 강제로 0으로 초기화하면서 물리 엔진이 정상 상태에 도달하도록 대기.

```python
# 안정화 루프 패턴 (~60프레임)
for _ in range(60):
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    world.step(render=True)
```

**Result**  
강제 위치 변경 후에도 로봇이 제자리에 정상적으로 안착. 이후 모든 초기화 직후에 이 패턴을 표준으로 사용.

---

### 08. 차체 심한 흔들림 (Physics Instability)

**Situation**  
로봇팔이 이동할 때마다 차체가 반력으로 심하게 흔들리며 밀렸다. 큰 각도를 이동할수록 차체가 뒤집힐 정도로 불안정했다.

**Task**  
팔 이동 중에도 차체가 안정적으로 제자리를 유지하도록 물리 파라미터와 제어 구조를 개선해야 했다.

**Action**  
단일 원인이 아닌 세 가지 복합 원인을 순서대로 처리했다.

1. **KR210 링크 질량/관성 직접 지정**: 링크 관성 경고(`inertia {1,1,1}, negative mass`)가 물리 연산을 불안정하게 만들었다. 각 링크에 현실적인 질량을 명시적으로 부여했다.

```python
link_masses = {
    "base_link": 150.0, "Link1": 80.0, "link_2": 120.0,
    "link_3": 60.0, "link_4": 40.0, "link_5": 20.0,
    "link_6": 10.0, "tool0": 2.0
}
inertia = mass * 0.1  # 대각 관성 텐서 근사
```

2. **바퀴 Damping 강화**: 바퀴가 팔의 반력을 흡수하지 못하고 있었다. `damping: 1e5 → 1e6`으로 10배 강화.

3. **`asyncio.sleep` 완전 제거 → `lerp + next_update_async()`로 교체**: `asyncio.sleep`은 물리 프레임과 무관하게 시간만 기다리므로 순간 토크가 발생한다. 프레임 단위 선형 보간으로 교체.

```python
# ❌ 금지 — 물리 프레임과 비동기
await asyncio.sleep(2.0)

# ✅ 안전 — 프레임 단위 보간
async def lerp_to(targets, steps=60):
    for i in range(steps):
        t = (i + 1) / steps
        interpolated = current + (target - current) * t
        robot.apply_action(ArticulationAction(joint_positions=interpolated))
        await omni.kit.app.get_app().next_update_async()
```

4. **팔 + 바퀴 제어를 단일 ArticulationAction으로 통합**: 한 프레임에 `apply_action()`을 두 번 호출하면 마지막 것만 적용된다(Override). 팔 타겟과 바퀴 브레이크를 `np.concatenate`로 묶어 1회 전송.

```python
arm_targets   = solve_ik(target_pos)       # UR10 6개 관절
wheel_targets = np.zeros(4)                # 바퀴 4개 브레이크
combined = np.concatenate([arm_targets, wheel_targets])
robot.apply_action(ArticulationAction(joint_positions=combined))
```

**Result**  
팔 이동 중 차체 흔들림 완전 억제. 차량이 제자리를 유지하며 팔이 안정적으로 동작.

> **핵심 인사이트**: `asyncio.sleep`은 Isaac Sim 물리 제어에서 절대 사용 금지. 그리고 차체 흔들림은 단일 원인이 아니었다 — 세 가지가 동시에 복합 작용하므로 하나씩 제거해도 안정화가 안 될 수 있다.

---

### 09. DriveAPI.Apply() USD 영구 기록 문제

**Situation**  
바퀴 조인트에 `DriveAPI.Apply()`로 Damping을 설정해 브레이크 효과를 주었는데, Stop → Play 재실행 시 코드에서 이 설정을 하지 않았음에도 바퀴가 저절로 움직이는 현상이 반복됐다.

**Task**  
Stop → Play 사이클에서도 바퀴가 의도치 않게 움직이지 않도록 Drive 설정 방식을 변경해야 했다.

**Action**
1. **원인 파악**: `DriveAPI.Apply()`는 USD 스테이지 파일에 Drive 속성을 **영구적으로 기록**한다. Stop 이후에도 USD에 Drive 설정이 남아있어 다음 Play 때 PhysX가 이를 읽어 바퀴를 구동시켰다.
2. **DriveAPI.Apply() 완전 제거**: 바퀴 조인트에는 DriveAPI 자체를 적용하지 않는다.
3. **런타임에만 velocity_target 0으로 설정**: USD를 건드리지 않고 매 프레임 콜로만 처리.

```python
# ❌ 잘못된 방식 — USD에 Drive 영구 기록됨
drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
drive_api.GetDampingAttr().Set(1e6)

# ✅ 올바른 방식 — 런타임에만 영향, USD 변경 없음
self.dc.set_dof_velocity_target(wheel_dof, 0.0)  # 매 프레임 호출
```

4. **이미 기록된 Drive 수동 삭제**: Stage 패널에서 wheel joint 선택 → Property → Drive 섹션 X 버튼 → USD 저장.

**Result**  
Stop → Play 재실행 후에도 바퀴가 의도치 않게 움직이는 현상 완전 제거.

> **핵심 인사이트**: Isaac Sim API 중 USD 파일에 영구 기록되는 것들이 있다. DriveAPI, MassAPI 등이 그렇다. "런타임에만 적용"과 "USD에 영구 저장"을 명확히 구분해야 한다.

---

### 10. Articulation 초기화 타이밍 오류

**Situation**  
Standalone 환경에서 `World` 객체 생성 후 바로 `robot.initialize()`를 호출했더니 에러가 발생하며 Articulation 제어가 전혀 안 됐다.

```
RuntimeError: Physics handle is not valid. Call world.reset() before using articulation.
```

**Task**  
Isaac Sim Standalone에서 Articulation 제어가 가능한 올바른 초기화 순서를 확립해야 했다.

**Action**
1. **원인 파악**: `world.reset()`은 PhysX 시뮬레이션 뷰(`SimulationView`)를 생성하고 Stage의 Physics Scene과 연결하는 작업이다. 이 작업 완료 전에 `initialize()`를 호출하면 `physics_sim_view`가 None이라 바인딩이 실패한다.
2. **올바른 초기화 순서 확립**:

```python
world = World()
world.scene.add(Robot(prim_path="/Root/robot/run_robot", name="ur10"))

# ① world.reset() 먼저
world.reset()

# ② 최소 1~2프레임 대기 (Physics 준비 대기)
for _ in range(2):
    simulation_app.update()

# ③ 그 다음에 initialize()
ur10 = world.scene.get_object("ur10")
ur10.initialize()

# ④ 이후부터 apply_action() 사용 가능
```

**Result**  
초기화 순서 준수 후 ArticulationController 정상 바인딩, 관절 제어 정상 동작.

> **핵심 인사이트**: 반드시 **World/Scene 설정 → `world.reset()` → `simulation_app.update()` 1회 이상 → `robot.initialize()` → `apply_action()`** 순서를 지킬 것. 이 순서를 어기면 대부분의 Articulation API가 에러 없이 조용히 실패한다.

---

### 11. ArticulationRoot 경로 오탐 (SingleManipulator → SingleArticulation)

**Situation**  
Carter AMR + UR10이 통합된 환경에서 `SingleManipulator(prim_path="/Root/robot/robot/nova_carter/ur10")`를 생성했더니 다음 에러가 반복됐다.

```
Failed to find articulation at '/Root/robot/robot/nova_carter/ur10'
AttributeError: 'NoneType' object has no attribute 'is_homogeneous'
```

**Task**  
Carter + UR10 통합 환경에서 UR10 관절만 정확히 제어하는 방법을 찾아야 했다.

**Action**
1. **Stage 진단 스크립트로 실제 ArticulationRoot 파악**:

```python
from pxr import UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"ArticulationRoot: {prim.GetPath()}")
# 출력: ArticulationRoot: /Root/robot/robot
```

2. **원인 확인**: Carter AMR과 UR10이 `/Root/robot/robot` 하나의 ArticulationRoot로 묶여 있었다. `ur10` prim에는 ArticulationRootAPI가 없어 `SingleManipulator`가 physics tensors에서 찾지 못함.
3. **`SingleManipulator` → `SingleArticulation`으로 교체**, 전체 Articulation Root를 등록한 뒤 joint 이름으로 UR10 인덱스만 추출:

```python
from isaacsim.core.prims import SingleArticulation

self.robot_art = self.world.scene.add(
    SingleArticulation(prim_path="/Root/robot/robot", name="full_robot")
)

# UR10 관절 이름으로 인덱스 추출
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
               "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
self.ur10_indices = [self.robot_art.get_dof_index(n) for n in joint_names]
```

4. **Carter 전체 DOF 중 UR10 관절 인덱스 확정** (wrist_2=7, wrist_3=10이 불연속인 이유는 Carter 바퀴 조인트가 사이에 섞여 있기 때문):

```
shoulder_pan_joint   → DOF 0
shoulder_lift_joint  → DOF 1
elbow_joint          → DOF 2
wrist_1_joint        → DOF 3
wrist_2_joint        → DOF 7  (Carter 바퀴 조인트가 4~6)
wrist_3_joint        → DOF 10
```

**Result**  
`SingleArticulation` + `joint_indices` subset 방식으로 Carter AMR 전체 Articulation에서 UR10 관절만 정확히 제어 성공.

> **핵심 인사이트**: 여러 로봇이 하나의 USD에 통합된 경우, 진단에서 ArticulationRootAPI가 `Yes`인 prim이 진짜 Root다. 통합 환경에서 일부 관절만 제어할 때는 `SingleArticulation + joint_indices` 방식이 표준이다.

---

### 12. Fake Grasp — SetParent 시 큐브 순간이동

**Situation**  
`ATTACH` 상태에서 큐브를 그리퍼에 Parent로 붙이는 순간, 큐브가 그리퍼 원점 좌표로 순간이동하며 PhysX가 폭발했다(Explosion).

**Task**  
SetParent 후에도 큐브가 현재 World 위치에 그대로 있도록 Transform을 보존하는 패턴이 필요했다.

**Action**
1. **원인 파악**: USD에서 Prim의 Transform은 부모 좌표계 기준 Local값으로 저장된다. 부모가 바뀌면 기존 Local Transform 수치는 유지되지만 새 부모 기준으로 재해석되어 World 위치가 완전히 달라진다.
2. **3단계 Parenting 패턴 적용**:

```python
# 1. 변경 전 World Pose 저장
world_pos, world_ori = cube_prim.get_world_pose()

# 2. Parent 변경 (keep_transform=True 옵션이 가장 안정적)
omni.kit.commands.execute('ParentTo',
    target_path='/Root/robot/run_robot/ur10_tool0/cube',
    parent_path='/Root/robot/run_robot/ur10_tool0',
    keep_transform=True
)

# 3. 혹시 keep_transform이 미지원인 경우 수동으로 World Pose 재적용
cube_isaac.set_world_pose(position=world_pos, orientation=world_ori)
```

3. **Detach(World로 복귀)도 동일 패턴 적용**.

**Result**  
SetParent 전후로 큐브가 동일한 World 위치를 유지. PhysX Explosion 완전 억제. 10회 연속 State Machine 동작에서 Fake Grasp 단계가 안정적으로 동작.

> **핵심 인사이트**: USD Parenting은 항상 **"World Pose 저장 → Parent 변경 → World Pose 재적용"** 3단계를 세트로 수행해야 한다.

---

### 13. OpenCV HSV 색상 인식 실패 (float32 미변환)

**Situation**  
Isaac Sim 카메라 이미지를 그대로 OpenCV에 넘겨 HSV 변환을 했더니 모든 픽셀 값이 0이 되면서 색상 인식이 완전히 실패했다.

**Task**  
Isaac Sim 카메라 출력 포맷과 OpenCV 입력 포맷의 불일치를 해결해야 했다.

**Action**
1. **원인 파악**: Isaac Sim 카메라는 float32(0.0~1.0) 형식으로 이미지를 반환한다. OpenCV의 HSV 변환은 uint8(0~255)를 전제로 설계되어 있어, float32를 넣으면 모든 값이 0으로 계산됐다.
2. **전처리 파이프라인 전면 수정**:

```python
import cv2
import numpy as np

# Isaac Sim 카메라 출력 = float32 (0~1)
img_float = camera.get_rgba()[:, :, :3]   # Alpha 채널 제거

# ① float32 → uint8 정규화 (필수!)
img_uint8 = (img_float * 255).astype(np.uint8)

# ② RGB → BGR (OpenCV 포맷)
img_bgr   = cv2.cvtColor(img_uint8, cv2.COLOR_RGB2BGR)

# ③ BGR → HSV
img_hsv   = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

# ④ Red 이중 Mask (Hue 0도 경계 처리)
mask_r1   = cv2.inRange(img_hsv, np.array([0,   100, 80]), np.array([10,  255, 255]))
mask_r2   = cv2.inRange(img_hsv, np.array([170, 100, 80]), np.array([180, 255, 255]))
mask_red  = cv2.bitwise_or(mask_r1, mask_r2)

# ⑤ Morphology로 노이즈 제거
kernel    = np.ones((5, 5), np.uint8)
mask_red  = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
```

**Result**  
색상 인식 성공. Red/Blue/Yellow 3색 인식 정확도 98% 이상. 실제 로그:
```
[COLOR] [2초] → detected: red    (12954 px)
[COLOR] [1초] → detected: blue   (12563 px)
[COLOR] [1초] → detected: yellow (12383 px)
```

> **핵심 인사이트**: Isaac Sim 카메라 = float32. OpenCV HSV = uint8 전제. 이 변환을 빠뜨리면 모든 HSV 값이 0이 되어 아무것도 감지되지 않는다. Red는 Hue 0도 경계에 걸치기 때문에 단일 범위 Mask로는 절반을 놓친다 — 이중 Mask 필수.

---

### 14. Phase 1→2 전환 시 책 낙하

**Situation**  
Phase 1(컨베이어 Pick & Place 완료 후 KLT 박스에 책 적재)이 끝나고 Phase 2에서 로봇을 책장 위치로 Teleport시키면, 로봇만 이동하고 KLT 박스 안의 책 3권이 제자리에 남아 낙하하거나 흩어졌다.

**Task**  
로봇이 Teleport할 때 책도 함께 이동하도록 해야 했다.

**Action**
1. **원인 파악**: 책 prim이 `/Root` 최상위 하위에 독립적으로 존재해서, 로봇의 Teleport Transform과 무관하게 고정 위치에 남았다.
2. **`reparent_with_world_pose()` 구현**: 책을 로봇 하위 prim으로 reparent하면 로봇 Transform을 상속해 함께 이동한다. World Pose를 보존하는 로직 포함.

```python
def reparent_with_world_pose(stage, src_path, dst_parent_path):
    # 1. 현재 world matrix 저장
    world_m = get_world_matrix(stage, src_path)
    
    # 2. prim을 새 부모 하위로 이동
    new_path = dst_parent_path + "/" + src_path.split("/")[-1]
    omni.kit.commands.execute("MovePrim", path_from=src_path, path_to=new_path)
    
    # 3. 새 parent 기준 local transform 역산 후 재적용
    parent_world_m = get_world_matrix(stage, dst_parent_path)
    local_m = world_m * parent_world_m.GetInverse()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(new_path))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(local_m))
    return new_path
```

3. **전환 흐름 구현**:
```
Phase 1 완료
→ attach_books_to_robot()    ← 책 3권을 /Root/robot 자식으로 reparent
→ teleport_robot(Home)       ← 로봇 + 책 함께 이동 ✅
→ A 도착 직전: detach("red") ← red만 /Root로 복귀 (detach 후 꽂기)
→ B, C도 동일 패턴
```

**Result**  
3차 통합에서 Phase 1→2 전환 시 책 낙하 문제 완전 해결. 한 시퀀스 연속 동작 성공.

---

### 15. 라이다 피자 조각 현상

**Situation**  
라이다 FOV를 360도로 설정했음에도 앞쪽 부채꼴 모양만 스캔되는 현상이 발생했다.

**Task**  
라이다가 360도 전방위를 정상 스캔하도록 해야 했다.

**Action**
1. **원인 1 (초기 추정)**: Legacy Lidar의 레이캐스트 렌더링 한계(6,000개 이상 선 처리 포기) → 확인했으나 이것만이 원인은 아니었다.
2. **원인 2 (확정)**: 라이다 센서를 감싸는 실린더 하우징 메쉬 내부에 센서 원점(origin)이 갇혀 있었다. 360도 레이저가 발사되어도 바로 자신의 하우징 껍데기에 충돌하여 짧은 거리만 측정되고 있었다.
3. **해결**: 실린더 하우징 메쉬 크기를 키워 센서 원점이 하우징 외부에 위치하도록 조정.

**Result**  
360도 전방위 스캔 정상 동작.

> **핵심 인사이트**: 센서를 메쉬로 감쌀 때, 센서 원점이 메쉬 내부에 있으면 레이캐스트가 자신의 껍데기를 먼저 맞혀버린다. 하우징은 센서 원점 기준으로 충분히 크게 만들거나 Collider를 비활성화해야 한다.

---
---

## 🔴 미해결 이슈

---

### 16. 컨베이어 끝 큐브 포즈 변형

**Situation**  
컨베이어 벨트 끝 지점에 큐브가 도달하면서 충돌 이벤트가 발생하고, PhysX가 회전 토크를 인가해 큐브가 기울어졌다. UR10의 Pre-Grasp 위치 계산은 큐브가 항상 축 정렬(Axis-Aligned) 상태라고 가정하고 있어 grasp 위치 오차가 발생했다.

**Task**  
컨베이어 끝에서 큐브가 기울어진 상태에서도 안정적으로 Grasp할 수 있어야 했다.

**임시 처치**  
`environment.usd`에서 book 오브젝트 삭제 + 컨베이어 끝 구조 재검토. 근본 해결은 되지 않았다.

**인사이트 및 시도해볼 방법**

- **방법 1 — 컨베이어 끝에 스토퍼 구조 추가**: 벽이나 블록을 배치해 큐브가 컨베이어 끝에서 멈출 때 기울어지지 않도록 물리적으로 차단. USD에서 간단한 Collider 블록을 추가하는 것으로 구현 가능.

- **방법 2 — 큐브 방향 감지 로직 추가**: Grasp 전에 `cube_prim.get_world_pose()`로 현재 orientation(쿼터니언)을 읽고, 기울기가 임계값 이상이면 기울어진 방향을 보정한 Pre-Grasp 자세를 계산. 기울기가 너무 클 경우 컨베이어를 잠시 역방향으로 돌려 큐브를 재정렬하는 방법도 고려할 수 있다.

- **방법 3 — 큐브 물리 속성 조정**: 큐브 Collider의 Restitution(반발계수)을 0으로 설정하고 angular damping을 높여 충돌 시 회전 에너지를 최대한 흡수하도록 조정.

- **방법 4 — 컨베이어 끝 감지 시점 조정**: 큐브가 컨베이어 끝에 도달하기 직전에 컨베이어를 정지시키고 물리적으로 안착할 시간을 준 뒤 Grasp하는 방식. settle 프레임(~60프레임) 대기 패턴을 활용.

---

### 17. Pick 위치까지 이동 실패 (RMPFlow 좌표계 불일치)

**Situation**  
UR10이 구동은 되는데 pick 위치까지 이동하지 못했다. 관절이 움직이기는 하지만 EE가 목표점에 도달하지 못하고 허공을 더듬거리거나 엉뚱한 방향으로 이동했다.

**Task**  
RMPFlow 기반 모션 플래닝에서 EE가 의도한 pick 위치에 정확히 도달하도록 좌표계와 타겟 입력을 수정해야 했다.

**현재 상태**  
3차 시도까지 진행했으나 미해결. 좌표계 불일치가 원인으로 추정됐으나 정확한 지점은 특정되지 않았다.

**인사이트 및 시도해볼 방법**

- **핵심 의심 지점**: RMPFlow Controller에 타겟을 넘길 때 World 좌표가 아닌 Local 좌표가 전달되고 있을 가능성이 가장 높다. 아래 6가지 값을 로그로 출력해 어디서 좌표계가 어긋나는지 먼저 특정해야 한다.

```python
print(f"[DEBUG] Target World Pos   : {target_pos}")
print(f"[DEBUG] Robot Base Pos     : {base_pos}")
print(f"[DEBUG] Current EE Pos     : {ee_pos}")
print(f"[DEBUG] RMPFlow Tgt Joints : {target_joints}")
print(f"[DEBUG] Current Joints     : {current_joints}")
print(f"[DEBUG] Joint Error        : {target_joints - current_joints}")
```

- **방법 1 — RMPFlow 타겟 좌표 검증**: `rmpflow.set_end_effector_target()`에 넘기는 position이 World Frame인지 Base Frame인지 API 문서에서 재확인. Isaac Sim 5.0에서 RMPFlow API 시그니처가 변경됐을 수 있으므로 `isaacsim.core.api` 문서 기준으로 확인.

- **방법 2 — `open_stage()` vs `add_reference_to_stage()` 확인**: `add_reference_to_stage()`로 환경을 로드하면 Multi-root 구조가 망가지거나 Action Graph가 손실될 수 있다. 환경 파일은 `open_stage()`로 직접 열어 원본 구조 100%를 보존하는 방식이 안전하다.

- **방법 3 — RMPFlow 대신 Joint Space 직접 제어 전환**: RMPFlow는 내부적으로 Kinematics Chain을 다시 계산하므로 USD 구조에 민감하다. 좌표계 혼동이 계속된다면 `ArticulationController`로 IK 결과 Joint Angle을 직접 전달하는 방식으로 전환하는 것이 디버깅이 더 용이하다.

- **방법 4 — action.joint_indices Mismatch 확인**: `ArticulationAction` 객체에 `joint_indices`를 명시하지 않으면 전체 DOF 순서 기준으로 값이 매핑된다. UR10 관절만 제어하려면 `joint_indices=[0,1,2,3,4,9]`를 명시적으로 전달해야 한다.

---

## 📌 공통 인사이트 요약

| 카테고리 | 핵심 교훈 |
|----------|-----------|
| **USD 구조** | Stage Tree ≠ Physical Kinematic Chain. ArticulationRoot는 Stage Traverse로 직접 찾아라 |
| **좌표계** | Local / World / Base Frame을 코드에서 항상 명시적으로 구분하고 변환 파이프라인을 명시적으로 작성 |
| **물리 타이밍** | `asyncio.sleep` 금지. `next_update_async()` + lerp 보간. 초기화 후 60프레임 안정화 |
| **API 함정** | `DriveAPI.Apply()`는 USD에 영구 기록됨. `apply_action()`은 동일 프레임 마지막 것만 적용 |
| **초기화 순서** | `world.reset()` → `simulation_app.update()` x2 → `robot.initialize()` 순서 필수 |
| **디버깅** | 로그 없이는 원인 찾기 불가. Target Pos / EE Pos / Joint Error 6종 출력이 표준 |
| **Fake Grasp** | SetParent 시 반드시 World Pose 저장 → 변경 → 재적용 3단계 패턴 |
| **카메라** | Isaac Sim 출력 = float32. OpenCV = uint8 전제. Red는 이중 Mask 필수 |