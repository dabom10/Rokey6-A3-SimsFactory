# 🚀 카메라 디버깅 - 빠른 시작 가이드

## 3가지 Standalone 스크립트

### 1️⃣ **camera_debug_standalone_v2.py** ⭐ (가장 추천)
- **목적**: 카메라 시야각 + 색상 인식 실시간 확인
- **화면**: 
  - 좌상단: RGB 카메라 영상
  - 우상단: Red 마스크
  - 좌하단: Blue 마스크
  - 우하단: Yellow 마스크
- **키 입력**: A(APPROACH), G(GRASP), L(LIFT), R(HOME), Q(종료)
- **용도**: 카메라 위치 확인, 색상 인식 테스트

```bash
python3 camera_debug_standalone_v2.py
```

---

### 2️⃣ **hsv_tuner_standalone.py** (색상 튜닝 전문)
- **목적**: HSV 범위값 정밀 조정
- **화면**:
  - 좌측: 원본 RGB 영상
  - 우측: 현재 범위로 추출된 마스크
  - 트랙바: HSV 범위값 조정
- **키 입력**: 1(RED), 2(BLUE), 3(YELLOW), P(출력), Q(종료)
- **용도**: 색상 인식 정확도 최적화

```bash
python3 hsv_tuner_standalone.py

# 사용 순서:
# 1. [1] 키 → RED 튜닝 (트랙바로 범위 조정)
# 2. [P] 키 → 최적값을 콘솔에 출력
# 3. [2] 키 → BLUE 튜닝
# 4. [3] 키 → YELLOW 튜닝
# 5. [Q] 키 → 종료
```

---

### 3️⃣ **camera_debug_standalone.py** (기본 버전)
- v2가 무거우면 이것 사용
- 기능이 조금 더 간단함

```bash
python3 camera_debug_standalone.py
```

---

## 🎯 추천 작업 순서

### Phase 1️⃣: 카메라 위치 확인
```bash
$ python3 camera_debug_standalone_v2.py
1. [A] 키로 APPROACH 포즈 → 카메라가 큐브를 어떻게 보는지 확인
2. [G] 키로 GRASP 포즈 → 최적의 각도인지 확인
3. 만약 카메라가 큐브를 제대로 못 보면 → environment.usd에서 카메라 위치/각도 조정
```

### Phase 2️⃣: 색상 인식 정밀 튜닝
```bash
$ python3 hsv_tuner_standalone.py
1. [1] 키 → RED 튜닝
   - 트랙바로 Lower_H, Lower_S, Lower_V, Upper_H, Upper_S, Upper_V 조정
   - 우측 마스크에서 빨강만 흰색으로 표시되는지 확인
   - [P] 키 → 최적값 출력
2. [2] 키 → BLUE 튜닝
3. [3] 키 → YELLOW 튜닝
4. [Q] 키 → 종료
```

### Phase 3️⃣: 최종 테스트
```bash
$ python3 camera_debug_standalone_v2.py
- 모든 색상이 올바르게 인식되는지 최종 확인
```

---

## 📊 HSV 범위 튜닝 팁

### OpenCV HSV 값 범위
```
H (Hue, 색상):      0 ~ 179 (빨강: 0-10 또는 160-179)
S (Saturation, 채도): 0 ~ 255 (높을수록 진함)
V (Value, 밝기):    0 ~ 255 (높을수록 밝음)
```

### 각 색상의 대략적인 범위
```
🔴 RED:
   - 범위 1: H(0-10),   S(100-255), V(100-255)
   - 범위 2: H(160-179), S(100-255), V(100-255)

🔵 BLUE:
   - H(100-140), S(100-255), V(100-255)

🟡 YELLOW:
   - H(20-40), S(100-255), V(100-255)
```

### 튜닝 전략
1. **채도(S) 낮음** → 채도 범위 넓히기
2. **밝기가 변함** → V 범위 넓히기
3. **이웃 색상 섞임** → H 범위 좁히기
4. **일부만 감지** → H, S, V 범위 조정

---

## 🔍 문제 해결

### ❌ "카메라 이미지를 가져올 수 없습니다"
```python
# 스크립트에서 CAMERA_PRIM_PATH 확인
CAMERA_PRIM_PATH = "/Root/robot/run_robot/ur10/ee_link/short_gripper/Camera"

# environment.usd에서 실제 경로 확인 후 수정
```

### ❌ 색상이 전혀 감지되지 않음
1. 마스크 윈도우가 완전히 검은색인지 확인
2. HSV 범위를 훨씬 넓게 설정해보기
   - 예: `lower = np.array([0, 0, 0])`, `upper = np.array([255, 255, 255])`
3. environment.usd의 큐브 색상이 올바른지 확인

### ❌ 여러 색상이 동시에 감지됨
1. HSV 범위가 너무 넓음
2. S(채도) 범위를 좁혀보기
3. H(색상) 범위를 더 좁혀보기

### ❌ 인식은 되지만 픽셀 수가 500 미만
1. 카메라가 큐브를 정면으로 보고 있는지 확인
2. environment.usd에서 큐브 크기 확인
3. 시뮬레이션의 조명 상태 확인

---

## 📝 최종 값 적용

HSV 튜닝 완료 후, `ur10_pick_place.py`의 `detect_book_color()` 메서드를 업데이트:

```python
def detect_book_color(self) -> str:
    # ... 기존 코드 ...
    
    # 튜닝된 값으로 변경
    lower_red1 = np.array([0, 100, 100])        # 튜닝 결과
    upper_red1 = np.array([10, 255, 255])       # 튜닝 결과
    lower_red2 = np.array([160, 100, 100])      # 튜닝 결과
    upper_red2 = np.array([179, 255, 255])      # 튜닝 결과
    
    lower_blue = np.array([100, 100, 100])      # 튜닝 결과
    upper_blue = np.array([140, 255, 255])      # 튜닝 결과
    
    lower_yellow = np.array([20, 100, 100])     # 튜닝 결과
    upper_yellow = np.array([40, 255, 255])     # 튜닝 결과
    
    # ... 나머지 코드 ...
```

---

## 💡 팁

### 화면이 너무 작으면
```python
# 스크립트에서 리사이징 크기 변경
combined = cv2.resize(combined, (1600, 1200))  # 더 크게
```

### 프레임율이 낮으면
```python
# cv2.waitKey(1) → cv2.waitKey(5)로 변경
# 또는 카메라 해상도 감소
# resolution=(320, 240)
```

### 값을 저장하고 싶으면
```bash
# hsv_tuner 실행 중 콘솔 로그 저장
python3 hsv_tuner_standalone.py | tee tuning_log.txt

# [P] 키를 눌러 값 출력, 로그에 저장됨
```

---

## 🎓 다음 단계

1. ✅ 카메라 시야각 확인 → camera_debug_standalone_v2.py
2. ✅ HSV 범위 최적화 → hsv_tuner_standalone.py
3. ✅ 최종 검증 → camera_debug_standalone_v2.py
4. ✅ 메인 프로그램 실행 → ur10_pick_place.py

---

## 🆘 추가 도움

- **CAMERA_DEBUG_GUIDE.md** - 상세 가이드
- **ur10_pick_place.py** - 메인 프로그램
- **DEBUG_CHANGES.md** - 디버깅 로그 설명
