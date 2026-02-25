# 📷 카메라 디버깅 Standalone 스크립트 사용 가이드

## 개요

Isaac Sim에서 UR10 로봇의 카메라 시야각을 실시간으로 확인하고, OpenCV를 이용한 색상 인식 알고리즘을 테스트할 수 있는 standalone 스크립트입니다.

---

## 파일 설명

### 1. **camera_debug_standalone_v2.py** ⭐ (추천)
가장 최신 버전으로, 다음 기능을 포함합니다:
- ✅ 카메라 실시간 RGB 영상 표시
- ✅ 3가지 색상별 마스크 (Red, Blue, Yellow) 동시 표시
- ✅ 각 마스크의 픽셀 수 실시간 출력
- ✅ 키보드로 로봇 포즈 제어 (A/G/L/R 키)
- ✅ 3개의 테스트 큐브 자동 생성
- ✅ 색상 인식 이력 저장

### 2. **camera_debug_standalone.py** (기본 버전)
더 간단한 버전입니다. v2가 무거우면 이것을 사용하세요.

---

## 실행 방법

### 기본 실행
```bash
python3 /mnt/user-data/outputs/camera_debug_standalone_v2.py
```

### 백그라운드에서 실행
```bash
nohup python3 /mnt/user-data/outputs/camera_debug_standalone_v2.py > camera_debug.log 2>&1 &
```

---

## 화면 구성

프로그램 실행 후 2개의 윈도우가 열립니다:

### 1️⃣ Isaac Sim 메인 윈도우
- 시뮬레이션 환경 렌더링
- 3개의 컬러 큐브 표시 (Red, Blue, Yellow)

### 2️⃣ OpenCV 디버그 윈도우 (1280x960)
```
┌─────────────────────────────────────┬──────────────────────────────────────┐
│     RGB Camera Feed                 │     Red Mask (빨강만)               │
│     Frame: 120                      │     Red Mask: 567 px                │
│                                     │                                      │
│     [카메라 실시간 영상]            │     [빨강 색상만 추출된 마스크]    │
│                                     │                                      │
├─────────────────────────────────────┼──────────────────────────────────────┤
│     Blue Mask (파랑만)              │     Yellow Mask (노랑만)            │
│     Blue Mask: 234 px               │     Yellow Mask: 89 px              │
│                                     │                                      │
│     [파랑 색상만 추출된 마스크]    │     [노랑 색상만 추출된 마스크]    │
│                                     │                                      │
└─────────────────────────────────────┴──────────────────────────────────────┘
```

각 마스크 창:
- **흰색**: 색상이 감지된 부분
- **검은색**: 감지되지 않은 부분
- **숫자**: 감지된 픽셀의 총 개수
- **회색 선**: 임계값(500px) 기준선

---

## 키보드 컨트롤

| 키 | 동작 | 설명 |
|---|---|---|
| **A** | APPROACH 포즈 | 카메라가 큐브 위에서 45도 각도에 위치 |
| **G** | GRASP 포즈 | 카메라가 큐브 정면(아래에서 본 각도) |
| **L** | LIFT 포즈 | 큐브를 들어올린 상태 |
| **R** | HOME 포즈 | 로봇 초기 위치로 복귀 |
| **Q** | 종료 | 프로그램 종료 |

### 포즈별 카메라 위치

```
[APPROACH 포즈]
        카메라
       /
      /
    큐브

[GRASP 포즈]
    카메라
      |
      |
    큐브

[LIFT 포즈]
   카메라
     |
     |
   [들어올려진 큐브]
```

---

## 색상 인식 튜닝

### HSV 색상 범위 변경 위치

스크립트의 `detect_book_color()` 메서드 내:

```python
# 색상 범위 설정 (OpenCV HSV: H(0~179), S(0~255), V(0~255))
lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
lower_red2, upper_red2 = np.array([160, 100, 100]), np.array([179, 255, 255])
lower_blue, upper_blue = np.array([100, 100, 100]), np.array([140, 255, 255])
lower_yellow, upper_yellow = np.array([20, 100, 100]), np.array([40, 255, 255])
```

**HSV 값 의미:**
- **H (Hue)**: 색상 (0~179, 빨강은 0-10 또는 160-179)
- **S (Saturation)**: 채도 (0~255, 높을수록 진함)
- **V (Value)**: 밝기 (0~255, 높을수록 밝음)

### 튜닝 팁

1. **빨강이 잘 안 잡힐 때**: 
   - S(채도) 범위를 더 낮게 (예: 50~100)
   - V(밝기) 범위 조정

2. **파랑이 잘 안 잡힐 때**:
   - `lower_blue = np.array([100, 80, 80])`로 변경
   - `upper_blue = np.array([140, 255, 255])`로 변경

3. **노랑이 잘 안 잡힐 때**:
   - `lower_yellow = np.array([15, 100, 100])`로 변경
   - `upper_yellow = np.array([45, 255, 255])`로 변경

---

## 콘솔 출력 예시

```
============================================================
📷 카메라 디버깅 모드 시작
============================================================
키 입력:
  [A] - APPROACH 포즈 (카메라가 큐브 위에서 45도)
  [G] - GRASP 포즈 (카메라가 큐브 정면)
  [L] - LIFT 포즈 (들어올린 상태)
  [R] - HOME 포즈 (원위치)
  [Q] - 종료
============================================================

[SETUP] 스테이지 로딩 중... (/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd)
[SETUP] ✅ 카메라 준비 완료!
✨ [TEST] 테스트 큐브 생성: /Root/red_test_book_0 (색상: RED)
✨ [TEST] 테스트 큐브 생성: /Root/blue_test_book_1 (색상: BLUE)
✨ [TEST] 테스트 큐브 생성: /Root/yellow_test_book_2 (색상: YELLOW)

🤖 [MOTION] GRASP (내려가기 포즈) 포즈로 이동 중...
[FRAME 0030] RED:   567, BLUE:   123, YELLOW:    45 ✅ Best: RED
[FRAME 0060] RED:   570, BLUE:   120, YELLOW:    48 ✅ Best: RED
[FRAME 0090] RED:   575, BLUE:   115, YELLOW:    50 ✅ Best: RED
```

---

## 문제 해결

### 1. "카메라 이미지를 가져올 수 없습니다" 에러
**원인**: 카메라 경로가 잘못됨
**해결책**:
```python
# environment.usd에서 실제 카메라 경로 확인
CAMERA_PRIM_PATH = "/Root/robot/run_robot/ur10/ee_link/short_gripper/Camera"
```

### 2. 색상이 전혀 인식되지 않음
**원인**: HSV 범위 설정이 잘못됨
**해결책**:
- 마스크 윈도우를 확인 (완전히 검은색이면 HSV 범위 확대)
- 조명 상태 확인
- environment.usd의 큐브 색상 확인

### 3. 모든 색상이 같이 인식됨
**원인**: HSV 범위가 너무 넓음
**해결책**:
- S(채도) 범위 조정
- H(색상) 범위 축소

### 4. OpenCV 윈도우가 나타나지 않음
**원인**: X11 포워딩 문제 (원격 서버)
**해결책**:
```bash
# 이미지 저장 방식으로 변경
cv2.imwrite(f"debug_frame_{frame_count}.png", combined)
```

---

## 성능 팁

1. **프레임율이 낮을 때**:
   - OpenCV 윈도우를 줄이거나 숨김
   - 카메라 해상도를 낮춤 (예: 320x240)

2. **메모리 사용량이 높을 때**:
   - 색상 이력(`history_length`) 감소
   - matplotlib 그래프 비활성화

3. **반응이 둔할 때**:
   - `cv2.waitKey(1)` 대신 `cv2.waitKey(5)` 사용

---

## 다음 단계

1. ✅ 카메라 시야각 확인
2. ✅ HSV 범위 튜닝
3. ✅ environment.usd에서 카메라 위치/각도 조정
4. ✅ 본 프로그램 (ur10_pick_place.py) 실행

---

## 버전 정보

- **Script Version**: 2.0
- **Isaac Sim**: 2024.x
- **Python**: 3.10+
- **Dependencies**: OpenCV, NumPy, Isaac Sim

---

## 지원

문제가 있으면:
1. 콘솔 로그 확인
2. 카메라 경로 검증
3. HSV 범위 조정
4. environment.usd 확인
