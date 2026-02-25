# ✅ 카메라 디버깅 Standalone 스크립트 - 완성!

## 📦 생성된 파일 (총 9개)

### 📚 문서 (5개) - 먼저 읽기
```
1. README.md                    ← 전체 개요 (처음 읽기)
2. QUICK_START.md               ← 실제 작업 (여기서 시작!) ⭐
3. CAMERA_DEBUG_GUIDE.md        ← 상세 설명
4. DEBUG_CHANGES.md             ← 로그 설명
5. FILES_OVERVIEW.txt           ← 파일 요약
```

### 🐍 Python 스크립트 (4개) - 순서대로 실행

#### 1️⃣ **camera_debug_standalone_v2.py** ⭐ (가장 중요)
```bash
python3 camera_debug_standalone_v2.py
```
- **목표**: 카메라 시야각 + 색상 인식 실시간 확인
- **화면**: 4분할 (RGB + Red/Blue/Yellow 마스크)
- **키**: A(APPROACH), G(GRASP), L(LIFT), R(HOME), Q(종료)
- **용도**: 카메라 위치 확인, 기초 색상 테스트

#### 2️⃣ **hsv_tuner_standalone.py** (정밀 조정)
```bash
python3 hsv_tuner_standalone.py
```
- **목표**: HSV 범위값 실시간 정밀 조정
- **화면**: 원본 영상 + 마스크 + 트랙바
- **키**: 1(RED), 2(BLUE), 3(YELLOW), P(출력), Q(종료)
- **용도**: 색상 인식 정확도 최적화

#### 3️⃣ **camera_debug_standalone.py** (기본 버전)
```bash
python3 camera_debug_standalone.py
```
- v2가 무거우면 사용
- 더 간단한 카메라 + 마스크 표시

#### 4️⃣ **ur10_pick_place.py** (메인 프로그램)
```bash
python3 ur10_pick_place.py
```
- **목표**: 색상별 큐브 자동 분류 및 배치
- **동작**: 3개 큐브 → 색상 인식 → KLT 배치
- **출력**: 상세한 디버깅 로그

---

## 🚀 추천 실행 순서 (30분)

### 1단계: 카메라 확인 (5분)
```bash
python3 camera_debug_standalone_v2.py
# [A] [G] [L] [R] 키로 포즈 변경 후 카메라 확인
# 카메라가 큐브를 제대로 보는가?
```

### 2단계: 색상 범위 튜닝 (15분)
```bash
python3 hsv_tuner_standalone.py
# [1] RED 튜닝 → 트랙바 조정 → [P] 값 출력
# [2] BLUE 튜닝 → 트랙바 조정 → [P] 값 출력
# [3] YELLOW 튜닝 → 트랙바 조정 → [P] 값 출력
# [Q] 종료
```

### 3단계: 최종 검증 (5분)
```bash
python3 camera_debug_standalone_v2.py
# 모든 색상이 명확하게 인식되는지 확인
```

### 4단계: 메인 프로그램 실행
```bash
python3 ur10_pick_place.py
# 3개의 큐브가 정확하게 분류되고 배치되는지 확인
```

---

## 📋 각 스크립트의 특징

### camera_debug_standalone_v2.py

**화면 구성**:
```
┌─────────────────────┬─────────────────────┐
│   RGB 카메라 영상   │   Red 마스크        │
│   Frame: 120        │   567 pixels        │
├─────────────────────┼─────────────────────┤
│   Blue 마스크       │   Yellow 마스크     │
│   234 pixels        │   89 pixels         │
└─────────────────────┴─────────────────────┘
```

**주요 기능**:
- ✅ 실시간 RGB 영상
- ✅ 3가지 색상 마스크 동시 표시
- ✅ 각 마스크의 픽셀 수 표시
- ✅ 4개 로봇 포즈 테스트 (A/G/L/R)
- ✅ 3개 테스트 큐브 자동 생성

---

### hsv_tuner_standalone.py

**화면 구성**:
```
좌측: 원본 RGB 영상        우측: 추출된 마스크
[카메라 실시간]           [현재 범위로 추출]
                          
[트랙바]
Lower_H  ▓▓▓▓░░░░ 0~179
Lower_S  ▓▓▓░░░░░ 0~255
Lower_V  ▓▓▓░░░░░ 0~255
Upper_H  ▓▓▓▓▓▓░░ 0~179
Upper_S  ▓▓▓▓▓▓▓░ 0~255
Upper_V  ▓▓▓▓▓▓▓░ 0~255
```

**주요 기능**:
- ✅ 3가지 색상별 독립적 튜닝 (1/2/3 키)
- ✅ 트랙바로 H/S/V 값 직관적 조정
- ✅ 실시간 마스크 미리보기
- ✅ 최적값 콘솔 출력 (P 키)
- ✅ 픽셀 수 임계값(500px) 시각화

---

## 💡 핵심 로그 메시지

### color_debug_standalone_v2.py
```
[FRAME 0030] RED:   567, BLUE:   123, YELLOW:    45 ✅ Best: RED
↑           ↑                                              ↑
프레임       각 색상의 감지 픽셀 수                      인식 성공 여부
```

### ur10_pick_place.py
```
🎯 [ARRIVAL] /Root/red_book_0 도착!
👁️‍🗨️ [VISION] 인식된 책 색상: RED
[COLOR_DEBUG] 색상 감지 픽셀 수 - RED:   567, BLUE:   123, YELLOW:    45
✅ [COLOR_DETECTION] 인식된 색상: RED (픽셀 수: 567)
📦 [PLACE_DEBUG] 색상: RED
📦 [PLACE_DEBUG] 목표 KLT 상자: small_KLT
📦 [PLACE_DEBUG] 목표 관절각도: J1=-3.400, J2=-1.050, J3=1.500, J4=-2.000, J5=-1.570, J6=-0.280
✨ [PLACE_COMPLETE] RED 상자에 성공적으로 배치했습니다!
```

---

## 🔧 만약 문제가 생기면?

### ❌ 색상이 인식되지 않음
1. `camera_debug_standalone_v2.py` 실행
2. 마스크 윈도우 확인
   - 완전히 검은색 → HSV 범위 너무 좁음
   - 너무 많은 흰색 → HSV 범위 너무 넓음
3. `hsv_tuner_standalone.py` 실행 후 범위 조정

### ❌ 카메라가 큐브를 못 봄
1. `environment.usd`에서 카메라 경로 확인
2. 카메라 위치/각도 조정
3. `camera_debug_standalone_v2.py` 재실행

### ❌ 픽셀 수가 500 미만
1. 큐브가 카메라 중앙에 있는지 확인
2. 큐브 크기 확인 (`BOOK_SCALE`)
3. 시뮬레이션 조명 상태 확인

---

## 📊 HSV 범위값 기본값

```python
# RED (빨강: 0-10 또는 160-179)
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])

# BLUE (파랑: 100-140)
lower_blue = np.array([100, 100, 100])
upper_blue = np.array([140, 255, 255])

# YELLOW (노랑: 20-40)
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])
```

**H (Hue, 색상)**: 0~179
**S (Saturation, 채도)**: 0~255 (높을수록 진함)
**V (Value, 밝기)**: 0~255 (높을수록 밝음)

---

## 🎯 색상별 배치 위치

| 색상 | KLT 상자 | 관절각도 J1 | 최종 포즈 |
|------|---------|----------|---------|
| 🔴 RED | small_KLT | -3.40 | PLACE_RED |
| 🔵 BLUE | small_KLT_01 | -3.20 | PLACE_BLUE |
| 🟡 YELLOW | small_KLT_02 | -3.00 | PLACE_YELLOW |

---

## ⏱️ 각 작업별 소요 시간

| 단계 | 시간 | 설명 |
|------|------|------|
| 문서 읽기 | 10분 | README → QUICK_START |
| 카메라 확인 | 5분 | camera_debug_v2 |
| 색상 튜닝 | 15분 | hsv_tuner |
| 최종 검증 | 5분 | camera_debug_v2 재실행 |
| 메인 프로그램 | 10분 | ur10_pick_place |
| **합계** | **45분** | 처음부터 완성까지 |

---

## ✨ 주요 개선 사항

### camera_debug_standalone_v2.py vs v1
- ✅ 4분할 화면 (더 명확한 시각화)
- ✅ 3개 테스트 큐브 자동 생성
- ✅ 실시간 프레임 카운트
- ✅ 포즈별 색상 인식 비교
- ✅ 임계값 시각화 (회색 선)

### hsv_tuner_standalone.py
- ✅ 트랙바로 직관적 조정
- ✅ Red는 2개 범위 지원 (180도 넘음)
- ✅ 실시간 마스크 미리보기
- ✅ 파이썬 코드 형식으로 출력
- ✅ 3개 색상 독립적 튜닝

### ur10_pick_place.py (개선)
- ✅ 상세한 디버깅 로그
- ✅ KLT 상자 이름 매핑
- ✅ 목표 좌표 실시간 출력
- ✅ 색상 인식 픽셀 수 표시
- ✅ 배치 완료 확인 메시지

---

## 📝 다음 단계

1. ✅ **README.md** 읽기
2. ✅ **QUICK_START.md** 읽기
3. ✅ `camera_debug_standalone_v2.py` 실행
4. ✅ `hsv_tuner_standalone.py` 실행
5. ✅ `camera_debug_standalone_v2.py` 재실행
6. ✅ `ur10_pick_place.py` 실행
7. ✅ 결과 확인 및 로그 검토

---

## 🎓 학습 팁

1. **HSV 색상 공간 이해**
   - H는 색상, S는 채도, V는 밝기
   - 색상이 안 잡히면 S 범위 조정
   - 밝기가 변하면 V 범위 조정

2. **카메라 위치 중요성**
   - APPROACH: 위에서 45도 각도 보기
   - GRASP: 아래에서 정면 보기
   - environment.usd에서 세밀 조정 필요

3. **디버깅 순서**
   - 카메라 시야각 확인 (v2)
   - HSV 범위 조정 (tuner)
   - 메인 프로그램 테스트 (main)
   - 문제 발생 시 반복

---

## 💻 시스템 요구사항

- **OS**: Linux (Ubuntu 20.04+)
- **Python**: 3.10+
- **Isaac Sim**: 2024.x
- **메모리**: 16GB+
- **GPU**: NVIDIA GPU (권장)

---

## 🎉 완성!

이제 모든 준비가 완료되었습니다!

### 실행 명령어 (한눈에)
```bash
# 단계 1: 카메라 확인
python3 camera_debug_standalone_v2.py

# 단계 2: 색상 튜닝
python3 hsv_tuner_standalone.py

# 단계 3: 메인 프로그램
python3 ur10_pick_place.py
```

**즐거운 개발 되세요! 🚀**

---

**Version**: 2.0  
**Last Updated**: 2026-02-25  
**Status**: ✅ Ready for Production
