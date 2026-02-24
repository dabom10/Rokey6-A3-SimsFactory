# 🤖 Isaac Sim Stage 진단 분석 리포트

**파일 경로:** `.../environment/robot/test/UR10 test/jjang/jonna_jjang_basic.usd`

이 문서는 Isaac Sim 씬에 로드된 로봇(모바일 매니퓰레이터 추정)의 계층 구조 및 물리 엔진(PhysX) 설정 상태를 분석한 결과입니다.

---

## 1. 아티큘레이션 루트 (Articulation Root)
* **경로:** `/Root/robot/run_robot`
* **타입:** `Xform`
* **상태:** ✅ **활성화됨**
* **분석:** 로봇의 최상위 부모(Base)에 Articulation Root가 정상적으로 적용되어 있습니다. 이는 Isaac Sim의 물리 엔진이 하위의 모든 링크와 조인트를 하나의 다관절 시스템(Articulation)으로 묶어서 계산하고 있음을 의미합니다.

## 2. 주행부 / 하부 조인트 (Base Joints)
* **대상:** `RevoluteJoint`, `RevoluteJoint_01` ~ `03`
* **타입:** `PhysicsRevoluteJoint`
* **주요 속성:**
  * `stiffness`: 0.0
  * `damping`: 100,000.0
  * `maxForce`: 무한대 (inf)
* **분석:** 이 관절들은 바퀴 구동부 또는 자유 회전부로 보입니다. Stiffness가 0이고 Damping이 매우 높게 설정되어 있는 것으로 보아, 위치 제어(Position Control)가 아닌 **속도 제어(Velocity Control)** 기반으로 동작하도록 세팅되어 있습니다.

## 3. 매니퓰레이터부 (UR10 Arm)
* **대상:** `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1` ~ `3`
* **타입:** `PhysicsRevoluteJoint`
* **조인트별 물리 속성 요약:**

| Joint Name | Stiffness | Damping | Max Force | Target Position |
| :--- | :--- | :--- | :--- | :--- |
| **shoulder_pan** | 153,634.48 | 10,000.0 | 330.0 | 0.0 |
| **shoulder_lift** | 100,000.0 | 10,000.0 | 330.0 | -90.0 |
| **elbow** | 831,589.25 | 10,000.0 | 150.0 | 90.0 |
| **wrist_1** | 165,646.51 | 10,000.0 | 56.0 | 0.0 |
| **wrist_2** | 157,650.18 | 10,000.0 | 56.0 | 0.0 |
| **wrist_3** | 155,870.89 | 10,000.0 | 56.0 | 0.0 |

* **분석:** * 높은 Stiffness 값과 특정 `targetPosition`(-90, 90 등)이 입력된 것으로 보아 전형적인 **위치 제어(PD Control)** 모드로 작동 중입니다.
  * 각 관절의 허용 최대 토크(`maxForce`)가 UR10의 실제 하드웨어 스펙(어깨 330Nm, 팔꿈치 150Nm, 손목 56Nm)을 정확히 반영하고 있습니다.
  * `jointFriction: 0.0`으로 설정되어 있어 관절의 기본 마찰 저항은 없는 상태입니다.

## 4. 엔드이펙터 / 그리퍼 (Suction Cup)
* **대상:** `/ur10/ee_link/short_gripper/suction_cup/Suction_Joint`
* **타입:** `PhysicsJoint` (다자유도 조인트)
* **분석:**
  * X, Y, Z 축 회전(`rotX`, `rotY`, `rotZ`)과 Z축 병진(`transZ`)에 대해 드라이브가 걸려 있습니다.
  * `stiffness: 1000.0`, `damping: 100.0`으로 설정되어 있습니다. 이는 UR10 팔에 비해 현저히 낮고 부드러운 수치입니다.
  * 흡착 패드(Suction Cup) 특유의 고무나 스프링 같은 **컴플라이언스(유연성/탄성)**를 물리적으로 시뮬레이션하기 위해 의도적으로 부드러운 스프링-댐퍼 시스템을 구성한 것으로 보입니다.

## 5. 고정 조인트 (Fixed Joints)
* **대상:** `small_KLT`, `ur10/ee_joint`, `base_link`, `Lidar` 등에 포함된 `PhysicsFixedJoint`
* **분석:** 구조물, 센서(라이다), 베이스 링크 간의 결합을 담당하며, 물리적 움직임이 없으므로 별도의 드라이브 속성이 없습니다. 정상적인 상태입니다.

---

## 📌 종합 결론 및 권장 사항
1. **정상 작동 중:** UR10의 스펙이 잘 반영되어 있으며, 모바일 베이스와 매니퓰레이터 간의 제어 방식(속도 제어 vs 위치 제어)이 올바르게 분리되어 설정되어 있습니다.
2. **튜닝 요소:** 만약 팔을 움직일 때 덜덜 떨리거나(Oscillation) 목표 위치에 제대로 도달하지 못한다면, UR10 관절의 `Damping` 값(현재 일괄 10,000)을 조절하거나, 시뮬레이션의 Physics Step 설정(예: 1/60 또는 1/120)을 확인해 볼 필요가 있습니다.

---

### 📥 원본 진단 출력 데이터

```text
========== Isaac Sim Stage 진단 시작 ==========
Stage 루트: file:/home/rokey/Rokey6-A3-SimsFactory/environment/robot/test/UR10%20test/jjang/jonna_jjang_basic.usd

■ [Prim] /Root/robot/run_robot
  ├─ Type: Xform
  ├─ Articulation Root: ✅ 활성화됨
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - (관련 물리 속성 없음)
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/joints/RevoluteJoint_02
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 100000.0
      - drive:angular:physics:maxForce: inf
      - drive:angular:physics:stiffness: 0.0
      - drive:angular:physics:targetPosition: 0.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/joints/RevoluteJoint_03
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 100000.0
      - drive:angular:physics:maxForce: inf
      - drive:angular:physics:stiffness: 0.0
      - drive:angular:physics:targetPosition: 0.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/joints/RevoluteJoint
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 100000.0
      - drive:angular:physics:maxForce: inf
      - drive:angular:physics:stiffness: 0.0
      - drive:angular:physics:targetPosition: 0.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/joints/RevoluteJoint_01
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 100000.0
      - drive:angular:physics:maxForce: inf
      - drive:angular:physics:stiffness: 0.0
      - drive:angular:physics:targetPosition: 0.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/small_KLT/FixedJoint
  ├─ Type: PhysicsFixedJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - (관련 물리 속성 없음)
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/small_KLT_01/FixedJoint
  ├─ Type: PhysicsFixedJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - (관련 물리 속성 없음)
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/small_KLT_02/FixedJoint
  ├─ Type: PhysicsFixedJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - (관련 물리 속성 없음)
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/joints/shoulder_pan_joint
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 10000.0
      - drive:angular:physics:maxForce: 330.0
      - drive:angular:physics:stiffness: 153634.484375
      - drive:angular:physics:targetPosition: 0.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
      - physxJoint:jointFriction: 0.0
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/joints/shoulder_lift_joint
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 10000.0
      - drive:angular:physics:maxForce: 330.0
      - drive:angular:physics:stiffness: 100000.0
      - drive:angular:physics:targetPosition: -90.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
      - physxJoint:jointFriction: 0.0
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/joints/elbow_joint
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 10000.0
      - drive:angular:physics:maxForce: 150.0
      - drive:angular:physics:stiffness: 831589.25
      - drive:angular:physics:targetPosition: 90.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
      - physxJoint:jointFriction: 0.0
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/joints/wrist_1_joint
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 10000.0
      - drive:angular:physics:maxForce: 56.0
      - drive:angular:physics:stiffness: 165646.515625
      - drive:angular:physics:targetPosition: 0.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
      - physxJoint:jointFriction: 0.0
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/joints/wrist_2_joint
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 10000.0
      - drive:angular:physics:maxForce: 56.0
      - drive:angular:physics:stiffness: 157650.1875
      - drive:angular:physics:targetPosition: 0.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
      - physxJoint:jointFriction: 0.0
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/joints/wrist_3_joint
  ├─ Type: PhysicsRevoluteJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:angular:physics:damping: 10000.0
      - drive:angular:physics:maxForce: 56.0
      - drive:angular:physics:stiffness: 155870.890625
      - drive:angular:physics:targetPosition: 0.0
      - drive:angular:physics:targetVelocity: 0.0
      - drive:angular:physics:type: force
      - physxJoint:jointFriction: 0.0
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/ee_joint
  ├─ Type: PhysicsFixedJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - (관련 물리 속성 없음)
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/base_link/FixedJoint
  ├─ Type: PhysicsFixedJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - (관련 물리 속성 없음)
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/ur10/ee_link/short_gripper/suction_cup/Suction_Joint
  ├─ Type: PhysicsJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - drive:rotX:physics:damping: 100.0
      - drive:rotX:physics:maxForce: inf
      - drive:rotX:physics:stiffness: 1000.0
      - drive:rotX:physics:targetPosition: 0.0
      - drive:rotX:physics:targetVelocity: 0.0
      - drive:rotX:physics:type: force
      - drive:rotY:physics:damping: 100.0
      - drive:rotY:physics:maxForce: inf
      - drive:rotY:physics:stiffness: 1000.0
      - drive:rotY:physics:targetPosition: 0.0
      - drive:rotY:physics:targetVelocity: 0.0
      - drive:rotY:physics:type: force
      - drive:rotZ:physics:damping: 100.0
      - drive:rotZ:physics:maxForce: inf
      - drive:rotZ:physics:stiffness: 1000.0
      - drive:rotZ:physics:targetPosition: 0.0
      - drive:rotZ:physics:targetVelocity: 0.0
      - drive:rotZ:physics:type: force
      - drive:transZ:physics:damping: 100.0
      - drive:transZ:physics:maxForce: inf
      - drive:transZ:physics:stiffness: 1000.0
      - drive:transZ:physics:targetPosition: 0.0
      - drive:transZ:physics:targetVelocity: 0.0
      - drive:transZ:physics:type: force
      - physxLimit:transZ:damping: 0.0
      - physxLimit:transZ:stiffness: 0.0
--------------------------------------------------
■ [Prim] /Root/robot/run_robot/base_footprint/base_link/base_scan/Lidar/FixedJoint
  ├─ Type: PhysicsFixedJoint
  ├─ Articulation Root: ❌ 아님
  └─ 주요 물리/조인트 속성 (Stiffness, Damping, Friction 등):
      - (관련 물리 속성 없음)
--------------------------------------------------