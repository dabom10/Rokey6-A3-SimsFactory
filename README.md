# 📚 Digital Twin 기반 이동형 도서 분류 및 이송 시스템

> NVIDIA Isaac Sim 5.0.0 환경에서 구현한 도서관 자동 반납 처리 로봇 시스템

**Team Sims Factory** | dabom10 · unknownbinfile · jowenchoi  
**기간**: 2026. 02. 21 ~ 02. 26

![Isaac Sim](https://img.shields.io/badge/NVIDIA-Isaac_Sim_5.0.0-76B900?style=flat-square&logo=nvidia)
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat-square&logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat-square&logo=ubuntu)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=flat-square&logo=python)

---

## 🎯 프로젝트 개요

컨베이어 벨트로 반납된 도서를 **UR10 로봇팔**이 색상 기반으로 인식·분류하고, **Nova Carter AMR**이 해당 책장까지 자율 이동하여 배가하는 End-to-End 자동화 파이프라인을 Isaac Sim 5.0 환경 내에서 구현합니다.

### 동작 플로우

```
[Phase 1] 반납 도서 수거
  컨베이어 → UR10 카메라로 색상 인식 → Pick & Place → KLT 박스 적재
          (red → A구역 / yellow → B구역 / blue → C구역)

[전환] 책 3권을 /Root/robot 자식으로 reparent (AMR 이동 시 낙하 방지)

[Phase 2] AMR 자율 주행 + 책 배가
  Home → A 책장(red) → B 책장(yellow) → C 책장(blue) → Home
```

---

## 🤖 시스템 구성

| 항목 | 내용 |
|------|------|
| 시뮬레이터 | NVIDIA Isaac Sim 5.0.0 |
| 미들웨어 | ROS2 Humble / Ubuntu 22.04 |
| 매니퓰레이터 | UR10 (6-DOF) |
| 모바일 베이스 | Nova Carter AMR |
| 색상 인식 | OpenCV HSV 기반 |
| 그리핑 | Fake Grasp (USD reparent 방식) |

---

## 🚀 실행 방법

### 사전 요구 사항

- NVIDIA Isaac Sim 5.0.0 설치
- ROS2 Humble 환경 구성

### 실행

```bash
cd ~/isaacsim
./python.sh ~/Rokey6-A3-SimsFactory/project/robot_move.py
```

내부적으로 아래 USD 환경 파일을 자동으로 로드합니다.

```
/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd
```

---

## 📁 프로젝트 구조

```
Rokey6-A3-SimsFactory/
├── project/                        # 메인 실행 파일 및 USD 환경
│   ├── robot_move.py               # ★ 통합 메인 스크립트 (Phase 1 + 2)
│   ├── environment.usd             # 전체 시뮬레이션 환경
│   ├── environment_carter.usd      # Nova Carter 포함 환경
│   ├── environment_carter_shelf.usd
│   ├── robot.usd                   # 로봇 단독 USD
│   ├── robot_move_origin.py        # 초기 버전
│   ├── standalone_first_ver.py
│   ├── standalone_totheshelf.py
│   └── usd_search.md
│
├── map_test/                       # Nav2 기반 자율 주행 테스트 패키지
│   └── src/environment/
│       ├── environment/
│       │   └── robot_sim_main.py
│       ├── launch/
│       │   ├── jjang_navigation.launch.py
│       │   ├── jjang_navigation_isaacsim.launch.py
│       │   └── jjang_navigation_goal.launch.py
│       ├── maps/
│       │   ├── jjang_map.png
│       │   └── jjang_map.yaml
│       ├── params/jjang_params.yaml
│       ├── rviz/jjang_navigation.rviz
│       └── usd/
│
├── robot_move_pkg/                 # ROS2 이동 제어 패키지
│   └── robot_move_pkg/
│       ├── move_robot_to_pose.py
│       └── pure_pursuit_move_node.py
│
└── devlog/                         # 개발일지 (2/21 ~ 2/26)
    ├── 260221.md ~ 260226.md
    └── IsaacSim_API.pdf
```

---

## ⚙️ 주요 구현 내용

### Phase 1 — Pick & Place (UR10)

- **상태 머신**: `READY → APPROACH → GRASP → LIFT → MOVE → PLACE → READY`
- **색상 인식**: 카메라 이미지를 OpenCV로 HSV 변환하여 red / blue / yellow 책 분류
- **가상 그리핑(Fake Grasp)**: `SurfaceGripper` 물리 불안정 대신 USD `reparent` 방식으로 책을 EE 자식 노드로 attach/detach

### Phase 2 — AMR 자율 이동 + 배가

- **Teleport 기반 이동**: Nav2 스택으로 ROS2 연동, `teleport_robot()` 함수로 목표 위치까지 이동
- **낙하 방지**: AMR 이동 시 책을 `/Root/robot` 하위로 reparent → AMR과 함께 이동 → 도착 후 `/Root`로 복귀
- **다중 카메라 전환**: 각 책장(A/B/C) 도착 시 해당 구역 카메라로 뷰포트 전환
- **책장 배가 시퀀스**: `READY → PICK → ATTACH → MID → TO_SHELF → PLACE → DETACH → READY`

---

## 🛠️ 주요 트러블슈팅

### 🔴 Articulation 구조 문제

**① Articulation 분리 현상** (2/24)

- **증상**: UR10과 그리퍼를 붙이면 Play 시 두 파트가 분리되어 날아감
- **원인**: UR10과 그리퍼가 각각 독립된 `ArticulationRoot`로 선언됨
- **해결**: UR10 prim을 `run_robot` 하위로 이동, `short_gripper`의 `RigidBodyAPI` 제거, 중복 `PhysicsScene` 제거
- **교훈**: 복합 로봇은 반드시 단일 `ArticulationRoot` 아래 모든 링크가 종속되어야 함. 부품 추가마다 `RigidBodyAPI` / `ArticulationRootAPI` 존재 여부 확인 필수

**② ArticulationRoot 위치 vs Joint Body 경로 불일치** (2/22)

- **증상**: wheel joint에 Drive 설정했으나 바퀴가 전혀 굴러가지 않음
- **원인**: `ArticulationRoot`가 Xform 컨테이너에 있고 wheel joint의 Body0이 실제 RigidBody prim을 가리키지 않음
- **해결**: DC API 접근 경로(`/Root/run_robot`)와 Joint Graph 무결성(Body0/Body1 경로)을 별개로 관리
- **교훈**: Stage Tree ≠ Physical Kinematic Chain

**③ DOF 인덱스 비표준 순서** (2/21)

- **증상**: joint position 설정 시 엉뚱한 관절이 움직임
- **원인**: `shoulder_lift_joint`가 인덱스 0번, `wrist_3_joint`가 바퀴 뒤인 9번으로 비표준 배치
- **해결**: `search_DOF.py`로 실제 인덱스 전수 조사 후 `UR10_IDX = [0,1,2,3,4,9]`, `WHEEL_IDX = [5,6,7,8]` 확정

---

### 🔴 역기구학(IK) 좌표계 문제

**④ IK 부호 오류 — 역관절 및 땅파기 현상** (2/22)

- **증상**: Pick 동작 시 `joint_a2`가 목표물 반대 방향으로 꺾임, dZ 오차 -2.248m
- **원인**: `theta2` 계산 시 불필요한 음수(-) 적용 + EE_OFFSET이 음수 방향으로 더해짐
- **해결**: `theta2 = -((np.pi / 2) - theta2_ik)` 로 부호 수정

**⑤ joint_a3 축 방향 반전 — Elbow-In 현상** (2/22)

- **증상**: 팔꿈치가 바깥이 아닌 몸통 쪽으로 심하게 접힘
- **원인**: KR210 USD의 조인트 축 방향이 수학적 IK 방향과 반대로 설정됨
- **해결**: 최종 모터 값 전달 단계에서 `joint_a3 = -theta3_math` 로 부호 반전
- **교훈**: 수학적 IK 수식이 완벽해도 제조사/모델별로 관절 회전 방향 기준이 다름

**⑥ World vs Local 좌표계 혼동** (2/24, 2/25)

- **증상**: 로봇이 pick 위치까지 이동 못 하거나 허공을 더듬거림
- **원인**: UI Property 창 값(Local)과 `get_world_pose()` 값(World)을 혼용
- **해결**: `RMPFlow` / `ArticulationController`에 타겟 전달 시 반드시 World 좌표 사용 원칙 확립
- **디버깅 필수 출력**: Target World Pos / Robot Base Pos / Current EE Pos / RMPFlow Target Joints / Current Joints / Joint Error

---

### 🔴 물리 엔진 안정화 문제

**⑦ Physics Bouncing — 로봇 튀어오름** (2/24)

- **증상**: 강제 위치 변경(`set_joint_positions`) 후 로봇이 튀어오르는 현상
- **원인**: USD 정적 속성 강제 변경 후 물리 엔진이 순간 속도를 무한대로 계산
- **해결**: 60프레임 안정화 루프 도입

```python
for _ in range(60):
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    world.step(render=True)
```

**⑧ 차체 심한 흔들림 — Physics Instability** (2/22)

- **증상**: 로봇팔 이동 시 차체가 반력으로 심하게 흔들리며 밀림
- **원인**: KR210 링크 관성 오류 + 바퀴 damping 부족 + `asyncio.sleep` 사용
- **해결**: 링크별 질량/관성 직접 지정, 바퀴 damping `1e5 → 1e6` 강화, `asyncio.sleep` 제거 후 `lerp + next_update_async()` 대체, 팔+바퀴 제어를 `np.concatenate`로 묶어 단일 `ArticulationAction` 전송

**⑨ DriveAPI.Apply()의 USD 영구 기록** (2/22)

- **증상**: Stop → Play 재실행 후 바퀴가 저절로 이동
- **원인**: `DriveAPI.Apply()`가 USD 스테이지에 Drive 속성을 영구 기록 → Play 재실행에도 유지
- **해결**: wheel 조인트에 `DriveAPI.Apply` 자체를 하지 않음. 런타임에만 `dc.set_dof_velocity_target(dof, 0.0)` 호출

---

### 🔴 라이다 / 센서 문제

**⑩ 라이다 피자 조각 현상** (2/24)

- **증상**: FOV 360도 설정에도 앞쪽 부채꼴 모양만 스캔됨
- **원인**: 라이다 센서를 감싸는 실린더 하우징 메쉬 안에 센서 원점이 갇혀, 360도 레이저가 자신의 껍데기를 먼저 맞힘
- **해결**: 실린더 메쉬 크기를 키워 센서 origin이 하우징 외부에 위치하도록 조정
- **교훈**: 센서 하우징 설계 시 센서 origin이 메쉬 내부에 갇히면 레이캐스트가 자기 자신의 껍데기를 먼저 맞춤

---

### 🔴 네비게이션 (Nav2 / ROS2 TF) 문제

**⑪ LiDAR 기울어짐 + 스캔 데이터 떨림 (jittering)** (2/26)

- **증상**: RViz 상에서 LiDAR 센서가 기울어진 상태로 표시되고, 스캔 데이터가 지속적으로 떨리는 현상 발생
- **원인**: Isaac Sim 내부 LiDAR 센서의 transform과 ROS2 TF tree 간 불일치. timestamp 딜레이로 인해 토픽을 적절하게 수신하지 못하는 것이 핵심 이슈
- **해결 시도**: 네트워크 문제 가능성을 파악하여 `tolerance`(지연 허용 범위)와 데이터 처리 속도를 미세 조정. 라이다 하우징 실린더와 센서 간 스케일 조정 과정에서 발생한 조인트 충돌 및 물리적 떨림 문제는 메쉬 크기 확장으로 해결

**⑫ Transform data too old — timestamp 지연 에러** (2/26)

- **증상**: Nav2 스택에서 `"Transform data too old"` 경고 발생, RViz의 Goal 명령이 실제 이동으로 변환되는 데 1초 이상 지연
- **원인**: Isaac Sim의 시뮬레이션 클럭과 ROS2 시스템 클럭 간 timestamp 동기화 불일치. 토픽 발행 주기와 TF 갱신 주기 차이로 인한 데이터 지연
- **해결 시도**: `jjang_params.yaml` 내 `transform_tolerance` 값 상향 조정, ActionGraph 발행 주기 튜닝
- **잔존 이슈**: 근본적 해결 미완료 → Nav2 완전 안정화가 향후 과제로 남음

**⑬ 커스텀 로봇 내부 파츠 간 충돌** (2/23 ~ 2/26)

- **증상**: 라이다 하우징 실린더와 로봇 차체 메쉬가 물리 엔진에서 충돌을 일으켜 AMR이 제자리에서 떨리거나 의도치 않은 방향으로 밀림
- **원인**: 커스텀 로봇 조립 시 시각적으로는 겹치지 않아 보여도, PhysX collision mesh가 실제 geometry보다 크게 설정되어 내부 파츠끼리 상시 접촉 상태
- **해결**: 문제가 되는 파츠의 `Collision Enabled` 속성을 선택적으로 비활성화하거나, collision mesh approximation을 `Convex Hull → Convex Decomposition`으로 변경하여 내부 간섭 제거

**⑭ Nova Carter 전환 후 prim 경로 전면 무효화** (2/26)

- **증상**: 환경 파일을 Nova Carter 기반으로 교체 후 기존 코드의 모든 prim 경로에서 에러 발생
- **원인**: `/Root/robot/nova_carter` 구조에서 중간 계층 `/Root/robot/robot`이 추가됨

```python
# 변경 전
ROBOT_ARTICULATION_ROOT = "/Root/robot/nova_carter"
# 변경 후
ROBOT_ARTICULATION_ROOT = "/Root/robot/robot"
```

- **교훈**: USD 구조 변경 시 최우선으로 prim 경로 진단 스크립트 실행 필수

---

### 🔴 USD 씬 구성 문제

**⑮ USD 파일 로드 방식 — add_reference vs open_stage** (2/24)

- **증상**: `add_reference_to_stage()`로 환경을 붙이면 Multi-root 구조가 망가지거나 Action Graph 손실
- **해결**: 환경 파일은 `open_stage()`로 직접 열어 원본 구조 100% 보존
- **원칙**: 개별 객체 추가 → `Reference`, 전체 환경 맵(다중 루트) → `Sublayer`

---

## 📅 개발 일지 요약

| 날짜 | 주요 내용 |
|------|-----------|
| 2/21 | 4륜 AMR + UR10 통합 articulation 구성, IK 첫 구현, DOF 인덱스 매핑 |
| 2/22 | KR210 마이그레이션, Extension 아키텍처 전환, USD Composition 원칙 정립 |
| 2/23 | UR10 원본 스케일 롤백, 환경 전면 재구성, Depth Camera → Stage Ground-Truth 방식 전환 |
| 2/24 | 흡착 그리퍼 + 카메라 + 라이다 장착, Pick & Place 시도 (좌표계 문제 미해결) |
| 2/25 | Standalone 전환, Fake Grasp 안정화, HSV 색상 인식 파이프라인 완성 |
| 2/26 | Nova Carter 전환, Phase 1 + 2 최종 통합, 전체 시퀀스 동작 확인 |

---

## 📋 실행 로그 예시

```
[PHASE 1]  책 → KLT 박스 담기 시작
[P1] red → READY → APPROACH → GRASP → LIFT → MOVE → PLACE (red) → 완료
[P1] blue → ... → 완료
[P1] yellow → ... → 완료
[P1] 모든 책 KLT 박스 담기 완료!

[전환]  책 3권 → /Root/robot 자식으로 reparent (낙하 방지)

[PHASE 2]  AMR 순회 + 책장 꽂기 시작
  Home → A(red) → B(yellow) → C(blue) → Home
[P2] ★ [A] 책꽂기 완료
[P2] ★ [B] 책꽂기 완료
[P2] ★ [C] 책꽂기 완료
[PHASE 2]  전체 완료!
```