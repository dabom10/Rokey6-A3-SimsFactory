# UR10 테스트 제어 성공

- basic.webm 영상 참고

1. ur10의 사이즈를 키웠기 때문에 제대로 동작되지 않을 수 있음에 유의하여 테스트했음
- 1차로 show by type에서 joints로 모든 관절의 정확한 위치에 조인트들이 붙어있는지,
- 2차로 GUI 상의 joint 각도 변경으로 ur10이 구동됨을,
- 3차로 python script 상의 코드로 동작이 되는지로 최종 확인하였음.

2. usd 파일에서 GUI 설정 일부도 변경하였음(stiffness, 차체 mass 등)

- basic.usd

3. python script 코드

```
import numpy as np
import omni.isaac.dynamic_control._dynamic_control as dc
import asyncio
import omni.kit.app
async def move_ur10_home_safely():
    ctrl = dc.acquire_dynamic_control_interface()
    art_path = "/Root/run_robot/body"
    art = ctrl.get_articulation(art_path)
    # 시뮬레이션이 재생 중이 아닐 경우 예외 처리
    if art == dc.INVALID_HANDLE:
        print(f"[Error] Articulation을 찾을 수 없습니다: {art_path}")
        print("반드시 좌측 상단의 'Play(재생)' 버튼을 먼저 누른 후 스크립트를 실행하세요!")
        return
    dof_count = ctrl.get_articulation_dof_count(art)
    print(f"[UR10] 전체 DOF 수: {dof_count}")
    # 직접 확인하신 UR10 관절 인덱스
    UR10_IDX = [0, 1, 2, 3, 4, 9]
    # [수정됨] 차체와 겹치지 않도록 하늘로 곧게 뻗는 '촛대 자세' (단위: Radian)
    HOME = np.array([
         np.pi / 2,  # [0] shoulder_lift (위로 뻗음)
         0.0,        # [1] elbow (일직선으로 폄)
         0.0,        # [2] shoulder_pan (정면)
        -np.pi / 2,  # [3] wrist_1
         0.0,        # [4] wrist_2
         0.0,        # [9] wrist_3
    ])
    # 현재 전체 DOF 위치 읽기
    all_states = ctrl.get_articulation_dof_states(art, dc.STATE_POS)
    current_pos = np.array(all_states["pos"], dtype=np.float32)
    target_pos = current_pos.copy()
    print(f"[UR10] 현재 전체 DOF (deg): {np.round(np.degrees(current_pos), 1)}")
    
    # [핵심] 보간(Interpolation) 설정: 150 프레임에 걸쳐 천천히 이동
    steps = 150  
    print("[UR10] 🚀 물리 폭발 방지: 부드러운 이동 시작...")
    for step in range(1, steps + 1):
        for i, idx in enumerate(UR10_IDX):
            # 현재 위치에서 목표 위치까지 아주 잘게 쪼개서(Lerp) 타겟을 업데이트
            target_pos[idx] = current_pos[idx] + (HOME[i] - current_pos[idx]) * (step / steps)
            
        # 쪼개진 타겟 위치를 로봇에 전송
        ctrl.set_articulation_dof_position_targets(art, target_pos)
        
        # ⭐️ 다음 물리 프레임까지 대기 (이 줄이 없으면 0초만에 이동해서 튕깁니다)
        await omni.kit.app.get_app().next_update_async()
    print(f"[UR10] 명령할 전체 DOF (deg): {np.round(np.degrees(target_pos), 1)}")
    print("[UR10] ✅ 홈 포지션 안전하게 도착 완료! 이제 차체가 날아가지 않습니다.")
# 스크립트 에디터에서 비동기 함수를 백그라운드로 실행
asyncio.ensure_future(move_ur10_home_safely())
```

# Isaac Sim 5.0 - 모바일 매니퓰레이터 (UR10) 개발 노트

## 1. Stage 구조

```
/Root/
└── run_robot/                    # Xform
    ├── body/                     # Xform  ← ArticulationRoot (바퀴+UR10 통합)
    │   └── base_link             # RigidBody
    ├── ur10/                     # Xform  ← ArticulationRoot (별도, dynamic_control에서 미인식)
    │   ├── joints/
    │   │   ├── shoulder_pan_joint   (RevoluteJoint)
    │   │   ├── shoulder_lift_joint  (RevoluteJoint)
    │   │   ├── elbow_joint          (RevoluteJoint)
    │   │   ├── wrist_1_joint        (RevoluteJoint)
    │   │   ├── wrist_2_joint        (RevoluteJoint)
    │   │   ├── wrist_3_joint        (RevoluteJoint)
    │   │   ├── ee_joint             (FixedJoint)
    │   │   └── Grippier_joint       (FixedJoint)
    │   ├── base_link / shoulder_link / upper_arm_link / ...
    │   └── ee_link
    ├── short_gripper/
    │   ├── SurfaceGripper           (IsaacSurfaceGripper)  ← 실제 gripper prim
    │   ├── suction_cup/Suction_Joint
    │   └── Camera, Lights, ...
    ├── wheel_back_left/
    ├── wheel_front_right/
    ├── wheel_back_right/
    ├── wheel_front_left/
    ├── Lidar/                    # Mesh
    └── Graphs/
        └── Velocity_Controller   # OmniGraph ← 주의: 매 프레임 관절 target 덮어씀
```

---

## 2. DOF 인덱스 매핑

`/Root/run_robot/body` articulation 기준 전체 DOF 10개:

| 인덱스 | 관절 이름 | 종류 |
| --- | --- | --- |
| 0 | shoulder_lift_joint | UR10 |
| 1 | elbow_joint | UR10 |
| 2 | shoulder_pan_joint | UR10 |
| 3 | wrist_1_joint | UR10 |
| 4 | wrist_2_joint | UR10 |
| 5 | wheel_back_left | 바퀴 |
| 6 | wheel_front_right | 바퀴 |
| 7 | wheel_back_right | 바퀴 |
| 8 | wheel_front_left | 바퀴 |
| 9 | wrist_3_joint | UR10 |

```
UR10_IDX  = [0, 1, 2, 3, 4, 9]   # UR10 관절
WHEEL_IDX = [5, 6, 7, 8]          # 바퀴
```

> ⚠️ `dynamic_control`에서 `/Root/run_robot/ur10`는 핸들을 못 가져옴.
반드시 `/Root/run_robot/body`로 접근해야 함.
> 

---

## 3. 핵심 트러블슈팅

### 3-1. Physics Explosion & Core Dump

**원인**: position_target을 현재값에서 목표값으로 한 번에 주면
Stiffness가 높은 관절이 순간적으로 무한대 토크를 발생 → 차체 날아감 + segfault

**해결**: 보간(Lerp)으로 여러 프레임에 걸쳐 잘게 쪼개서 전송

```
steps = 150
for step in range(1, steps + 1):
    for i, idx in enumerate(UR10_IDX):
        target_pos[idx] = current_pos[idx] + (HOME[i] - current_pos[idx]) * (step / steps)
    ctrl.set_articulation_dof_position_targets(art, target_pos)
    await omni.kit.app.get_app().next_update_async()  # ← 필수! 프레임 대기
```

### 3-2. set_articulation_dof_position_targets segfault

**원인**: numpy array를 직접 전달하면 PhysX 내부 타입 불일치로 크래시

```
# ❌ 크래시
ctrl.set_articulation_dof_position_targets(art, np.array([...]))

# ✅ 안전
target = list(all_states["pos"])          # 파이썬 list로 변환
target[idx] = float(val)                  # float 명시
ctrl.set_articulation_dof_position_targets(art, target)
```

### 3-3. 로봇이 아무 반응 없음

**원인 1**: OmniGraph `Velocity_Controller`가 매 프레임 target을 덮어씀

- Stage에서 `/Root/run_robot/Graphs` 우클릭 → Deactivate

**원인 2**: Joint Drive Stiffness가 0이라 모터 힘이 없음

```
# Stop 상태에서 실행
from pxr import UsdPhysics, Usd
ur10_joints_scope = stage.GetPrimAtPath("/Root/run_robot/ur10/joints")
for prim in Usd.PrimRange(ur10_joints_scope):
    if prim.GetName() in UR10_JOINTS:
        drive = UsdPhysics.DriveAPI(prim, "angular")
        drive.GetStiffnessAttr().Set(1e6)
        drive.GetDampingAttr().Set(1e4)
```

### 3-4. World.instance() = None

**원인**: GUI에서 USD를 열면 Python World 객체가 생성되지 않음
**해결**: `dynamic_control` API 직접 사용

```
import omni.isaac.dynamic_control._dynamic_control as dc
ctrl = dc.acquire_dynamic_control_interface()
art  = ctrl.get_articulation("/Root/run_robot/body")
```

### 3-5. DofState 접근 방식

```
# 단일 DOF
state = ctrl.get_dof_state(dof, dc.STATE_POS)
pos = state.pos                           # .pos 속성

# 전체 DOF (numpy structured array)
all_states = ctrl.get_articulation_dof_states(art, dc.STATE_POS)
pos_array = np.array(all_states["pos"])   # ["pos"] 키
```

### 3-6. 충돌체 겹침 (Play 하자마자 날아감)

**원인**: UR10 base_link와 차체 콜라이더가 겹친 상태로 스폰
**해결**:

- GUI에서 ur10 prim의 Translate Z 올려서 차체 위로 띄우기
- 또는 Collision Group으로 ur10↔body 간 충돌 무시 설정

### 3-7. 관절 방향 부호

UR10 에셋마다 0도 기준과 +/- 방향이 다를 수 있음.
위로 뻗어야 하는데 아래로 가면 부호 반전:

```
# shoulder_lift: 아래로 가면 np.pi/2 → -np.pi/2 로 변경
```

---

## 4. 작동하는 최종 코드 - UR10 홈 포지션

```
import numpy as np
import omni.isaac.dynamic_control._dynamic_control as dc
import asyncio
import omni.kit.app

async def move_ur10_home():
    ctrl = dc.acquire_dynamic_control_interface()
    art  = ctrl.get_articulation("/Root/run_robot/body")

    if art == dc.INVALID_HANDLE:
        print("[Error] Play 버튼을 먼저 누르세요!")
        return

    UR10_IDX = [0, 1, 2, 3, 4, 9]
    # DOF 순서: [0]=shoulder_lift, [1]=elbow, [2]=shoulder_pan,
    #           [3]=wrist_1, [4]=wrist_2, [9]=wrist_3

    HOME = np.array([
         np.pi / 2,  # [0] shoulder_lift → 위로
         0.0,        # [1] elbow
         0.0,        # [2] shoulder_pan
        -np.pi / 2,  # [3] wrist_1
         0.0,        # [4] wrist_2
         0.0,        # [9] wrist_3
    ])

    all_states  = ctrl.get_articulation_dof_states(art, dc.STATE_POS)
    current_pos = np.array(all_states["pos"], dtype=np.float32)
    target_pos  = current_pos.copy()

    steps = 150  # 프레임 수 (많을수록 부드럽고 안전)
    for step in range(1, steps + 1):
        for i, idx in enumerate(UR10_IDX):
            target_pos[idx] = current_pos[idx] + (HOME[i] - current_pos[idx]) * (step / steps)
        ctrl.set_articulation_dof_position_targets(art, target_pos)
        await omni.kit.app.get_app().next_update_async()

    print("[UR10] ✅ 홈 포지션 도착!")

asyncio.ensure_future(move_ur10_home())
```

---

## 5. 차량 무게 설정

```
from pxr import UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()
prim  = stage.GetPrimAtPath("/Root/run_robot/body/base_link")

if prim.HasAPI(UsdPhysics.MassAPI):
    mass_api = UsdPhysics.MassAPI(prim)
else:
    mass_api = UsdPhysics.MassAPI.Apply(prim)

mass_api.GetMassAttr().Set(150.0)  # kg
print(f"Mass → {mass_api.GetMassAttr().Get()} kg")
```

> Stop 상태에서 실행해야 Play 후에도 유지됨.
> 

---

## 6. 물리 설정 3대 필수 요소 (요약)

| 요소 | 역할 | 설정 위치 |
| --- | --- | --- |
| **ArticulationRoot** | "이 하위 구조가 하나의 로봇"임을 선언 | 최상위 Xform prim |
| **Joint Drive (Stiffness/Damping)** | 관절 모터 힘 부여. UR10은 `1e6` 이상 필요 | 각 RevoluteJoint |
| **Fixed Joint** | 팔 베이스와 차체 고정. 없으면 중력에 분리됨 | 팔-차체 연결 joint |

---

## 7. 유용한 진단 코드

```
# ArticulationRoot 위치 찾기
from pxr import Usd, UsdPhysics
import omni.usd
stage = omni.usd.get_context().get_stage()
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"ArticulationRoot: {prim.GetPath()}")

# DOF 이름 전체 출력
import omni.isaac.dynamic_control._dynamic_control as dc
ctrl = dc.acquire_dynamic_control_interface()
art  = ctrl.get_articulation("/Root/run_robot/body")
for i in range(ctrl.get_articulation_dof_count(art)):
    dof = ctrl.get_articulation_dof(art, i)
    print(f"[{i}] {ctrl.get_dof_name(dof)}")
```