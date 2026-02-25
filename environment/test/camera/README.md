# 📦 UR10 Pick & Place with Camera-based Color Detection

## 📋 프로젝트 개요

이 프로젝트는 Isaac Sim의 UR10 로봇이 **카메라 비전**을 이용해 **색상별로 큐브를 자동 분류**하는 시스템입니다.

### 주요 특징
- 🎥 실시간 카메라 영상 처리
- 🌈 OpenCV 기반 색상 인식 (Red, Blue, Yellow)
- 🤖 UR10 로봇 자동 제어
- 📊 실시간 디버깅 및 모니터링
- 🎛️ HSV 범위값 정밀 튜닝

---

## 📁 파일 구조

```
outputs/
├── 📄 README.md                          ← 이 파일
├── 📄 QUICK_START.md                     ← 빠른 시작 가이드 ⭐ 여기서 시작!
├── 📄 CAMERA_DEBUG_GUIDE.md              ← 상세 카메라 디버깅 가이드
├── 📄 DEBUG_CHANGES.md                   ← 디버깅 로그 설명
│
├── 🐍 ur10_pick_place.py                 ← 메인 프로그램
│                                          (색상 분류 + 배치)
│
├── 🎥 카메라 디버깅 스크립트들:
│   ├── camera_debug_standalone_v2.py     ← ⭐ 추천 (4분할 카메라 뷰)
│   ├── camera_debug_standalone.py        ← 기본 버전
│   └── hsv_tuner_standalone.py           ← 색상 범위 정밀 튜닝
```

---

## 🚀 빠른 시작 (5분)

### Step 1: 카메라 시야각 확인
```bash
cd /mnt/user-data/outputs
python3 camera_debug_standalone_v2.py
```
**목표**: 카메라가 큐브를 올바르게 보고 있는지 확인

**키 입력**:
- `A` - APPROACH 포즈
- `G` - GRASP 포즈
- `L` - LIFT 포즈
- `R` - HOME 포즈
- `Q` - 종료

### Step 2: HSV 색상 범위 튜닝
```bash
python3 hsv_tuner_standalone.py
```
**목표**: 각 색상의 인식 정확도 최적화

**키 입력**:
- `1` - RED 튜닝
- `2` - BLUE 튜닝
- `3` - YELLOW 튜닝
- `P` - 최적값 출력
- `Q` - 종료

### Step 3: 메인 프로그램 실행
```bash
python3 ur10_pick_place.py
```
**결과**: 3개의 큐브를 색상별로 자동 분류하여 KLT 상자에 배치

---

## 📺 화면 구성

### camera_debug_standalone_v2.py 실행 시
```
┌─────────────────────────────────────┬──────────────────────────────────────┐
│                                     │                                       │
│     RGB Camera Feed                 │     Red Mask                         │
│     Frame: 120                      │     Red Mask: 567 px                │
│                                     │                                       │
│     [카메라 실시간 영상]            │     [빨강만 추출]                  │
│                                     │                                       │
├─────────────────────────────────────┼──────────────────────────────────────┤
│                                     │                                       │
│     Blue Mask                       │     Yellow Mask                      │
│     Blue Mask: 234 px               │     Yellow Mask: 89 px               │
│                                     │                                       │
│     [파랑만 추출]                  │     [노랑만 추출]                  │
│                                     │                                       │
└─────────────────────────────────────┴──────────────────────────────────────┘
```

---

## 🎯 색상 분류 규칙

| 색상 | Red Cube | Blue Cube | Yellow Cube |
|-----|----------|----------|------------|
| 감지 픽셀 | 최고값 | 최고값 | 최고값 |
| 배치 위치 | small_KLT | small_KLT_01 | small_KLT_02 |
| 관절각도 J1 | -3.400 | -3.200 | -3.000 |

---

## 🔧 시스템 구성

### 하드웨어
- **로봇**: UR10 (6축 협작로봇)
- **그리퍼**: Suction Cup (흡입식)
- **카메라**: Eye-in-Hand (EE Link에 장착)

### 소프트웨어
- **시뮬레이터**: NVIDIA Isaac Sim 2024
- **언어**: Python 3.10+
- **라이브러리**: OpenCV, NumPy, Isaac Sim SDK

### 동작 시퀀스
```
1. 큐브 스폰
   ↓
2. 컨베이어에서 수집
   ↓
3. APPROACH 포즈 (접근)
   ↓
4. 카메라로 색상 인식
   ↓
5. GRASP 포즈 (내려가기)
   ↓
6. 흡입으로 픽업
   ↓
7. LIFT 포즈 (들어올리기)
   ↓
8. MOVE 포즈 (이동)
   ↓
9. PLACE 포즈 (색상별 배치)
   ↓
10. 분리
   ↓
11. RETREAT 포즈 (복귀)
   ↓
[반복]
```

---

## 📊 로그 메시지 해석

### 색상 인식 로그
```
[FRAME 0030] RED:   567, BLUE:   123, YELLOW:    45 ✅ Best: RED
```
- **RED/BLUE/YELLOW**: 각 색상의 감지 픽셀 수
- **✅**: 인식 성공 (픽셀 수 > 500)
- **Best**: 최고값 색상

### 배치 로그
```
📦 [PLACE_DEBUG] 색상: RED
📦 [PLACE_DEBUG] 목표 KLT 상자: small_KLT
📦 [PLACE_DEBUG] 목표 관절각도: J1=-3.400, J2=-1.050, ...
📦 [PLACE_START] RED 상자(small_KLT)로 이동합니다.
✨ [PLACE_COMPLETE] RED 상자에 성공적으로 배치했습니다!
```

---

## 🐛 문제 해결

### Issue 1: 카메라 영상이 보이지 않음
```
원인: CAMERA_PRIM_PATH가 잘못됨
해결: environment.usd에서 실제 카메라 경로 확인
     CAMERA_PRIM_PATH = "/Root/robot/run_robot/ur10/ee_link/short_gripper/Camera"
```

### Issue 2: 색상이 전혀 인식되지 않음
```
원인: HSV 범위 설정 오류
해결: 
  1. camera_debug_standalone_v2.py 실행
  2. 마스크 윈도우 확인 (완전히 검은색이면 범위 확대)
  3. hsv_tuner_standalone.py에서 정밀 조정
```

### Issue 3: 여러 색상이 동시에 인식됨
```
원인: HSV 범위가 너무 넓음
해결:
  1. hsv_tuner_standalone.py 실행
  2. [1]/[2]/[3]로 각 색상 선택
  3. 트랙바로 S(채도) 범위 좁히기
  4. [P] 키로 값 확인
```

### Issue 4: 프레임율이 낮음
```
원인: 카메라 해상도가 높거나 처리 속도 느림
해결:
  - resolution=(320, 240)로 변경
  - cv2.waitKey(1) → cv2.waitKey(5)로 변경
```

---

## 📖 문서 가이드

| 문서 | 용도 | 읽어야 할 때 |
|-----|------|-----------|
| **QUICK_START.md** | 빠른 시작 | 처음 시작할 때 ⭐ |
| **CAMERA_DEBUG_GUIDE.md** | 상세 설명 | 자세히 알고 싶을 때 |
| **DEBUG_CHANGES.md** | 디버깅 로그 | 로그를 해석할 때 |
| **README.md** | 전체 개요 | 전체 구조를 알 때 |

---

## 🎨 HSV 색상 범위 기본값

### OpenCV HSV 범위 (H: 0~179, S: 0~255, V: 0~255)

```python
# RED (0-10, 160-179)
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])

# BLUE (100-140)
lower_blue = np.array([100, 100, 100])
upper_blue = np.array([140, 255, 255])

# YELLOW (20-40)
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])
```

---

## 🔄 작업 흐름

### 초기 셋업 (처음 한 번)
```
1. environment.usd에서 카메라 위치 확인
   └─ 필요 시 카메라 위치/각도 조정
2. camera_debug_standalone_v2.py로 시야각 확인
3. hsv_tuner_standalone.py로 색상 범위 최적화
4. 튜닝 결과를 ur10_pick_place.py에 반영
```

### 반복 테스트 (매번)
```
1. ur10_pick_place.py 실행
2. 색상 인식 및 배치 결과 확인
3. 문제 발생 시 camera_debug_standalone_v2.py로 진단
4. 필요 시 hsv_tuner_standalone.py로 재조정
```

---

## 💻 시스템 요구사항

- **OS**: Linux (Ubuntu 20.04+)
- **Python**: 3.10 이상
- **Isaac Sim**: 2024.x
- **GPU**: NVIDIA GPU (권장)
- **메모리**: 16GB 이상
- **디스크**: 20GB 이상

---

## 📝 주요 상수 정리

### 로봇 포즈 (관절각도)
```python
POSE_APPROACH = [-1.5, -1.20, 1.40, -1.80, -1.57, 0.20]  # 접근
POSE_GRASP    = [-1.5, -1.05, 1.55, -2.05, -1.57, 0.25]  # 내려가기
POSE_LIFT     = [-1.5, -1.20, 1.40, -1.80, -1.57, 0.20]  # 들어올리기
POSE_MOVE     = [-2.5, -1.20, 1.35, -1.75, -1.57, -0.30]  # 이동

# 색상별 배치 포즈
POSES_PLACE = {
    "red":    [-3.40, -1.05, 1.50, -2.00, -1.57, -0.28],  # small_KLT
    "blue":   [-3.20, -1.05, 1.50, -2.00, -1.57, -0.28],  # small_KLT_01
    "yellow": [-3.00, -1.05, 1.50, -2.00, -1.57, -0.28]   # small_KLT_02
}
```

### 타이밍 (초 단위)
```python
START_DELAY_S = 3.0      # 시작 대기
HOLD_APPROACH_S = 2.0    # APPROACH 홀드
HOLD_GRASP_S = 1.5       # GRASP 홀드
HOLD_LIFT_S = 2.0        # LIFT 홀드
HOLD_MOVE_S = 2.0        # MOVE 홀드
HOLD_PLACE_S = 1.5       # PLACE 홀드
```

---

## 🎓 학습 리소스

1. **OpenCV HSV 색상 공간**
   - https://opencv-python-tutroals.readthedocs.io/

2. **Isaac Sim 공식 문서**
   - https://docs.omniverse.nvidia.com/

3. **UR 로봇 API**
   - https://docs.isaacsim.phased.ai/

---

## 📞 문제 보고

문제가 발생하면:
1. **콘솔 로그** 확인
2. **QUICK_START.md** 의 문제 해결 섹션 확인
3. **camera_debug_standalone_v2.py** 로 카메라 상태 진단
4. **hsv_tuner_standalone.py** 로 색상 범위 재조정

---

## 📌 체크리스트

- [ ] QUICK_START.md 읽음
- [ ] camera_debug_standalone_v2.py 실행 및 카메라 확인
- [ ] hsv_tuner_standalone.py로 색상 범위 조정
- [ ] 최적값을 ur10_pick_place.py에 반영
- [ ] ur10_pick_place.py 최종 테스트
- [ ] 배치 결과 확인 및 로그 검토

---

## 🎉 완성!

모든 단계를 완료하면 UR10 로봇이 **카메라 비전으로 색상을 인식하고 자동으로 배치**합니다!

---

**Last Updated**: 2026-02-25  
**Version**: 2.0  
**Status**: ✅ Production Ready
