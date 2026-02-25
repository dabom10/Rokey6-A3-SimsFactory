# UR10 Pick & Place 색상 인식 디버깅 수정 사항

## 주요 변경 사항

### 1. **KLT 상자 매핑 추가** (36~41줄)
```python
# KLT 상자 이름 매핑
KLT_NAMES = {
    "red": "small_KLT",
    "blue": "small_KLT_01",
    "yellow": "small_KLT_02"
}
```
- 각 색상별로 어느 KLT 상자에 배치되는지 명확히 함

---

### 2. **색상 인식 함수 개선** (`detect_book_color()` 메서드)

#### 개선 전:
```python
carb.log_warn(f"👁️‍🗨️ [VISION] 인식된 책 색상: {detected_color.upper()}")
```

#### 개선 후:
```python
# 실시간 픽셀 수 출력
carb.log_warn(f"[COLOR_DEBUG] 색상 감지 픽셀 수 - RED: {counts['red']:5d}, BLUE: {counts['blue']:5d}, YELLOW: {counts['yellow']:5d}")

# 최종 인식 결과
carb.log_warn(f"✅ [COLOR_DETECTION] 인식된 색상: {best_color.upper()} (픽셀 수: {counts[best_color]})")
```

**디버깅 출력 예시:**
```
[COLOR_DEBUG] 색상 감지 픽셀 수 - RED:   234, BLUE:   567, YELLOW:   123
✅ [COLOR_DETECTION] 인식된 색상: BLUE (픽셀 수: 567)
```

---

### 3. **PLACE(배치) 동작 시 상세 디버깅 추가** (232~239줄)

```python
# 디버깅 출력: 목표 좌표와 KLT 상자 이름
carb.log_warn(f"📦 [PLACE_DEBUG] 색상: {detected_color.upper()}")
carb.log_warn(f"📦 [PLACE_DEBUG] 목표 KLT 상자: {klt_name}")
carb.log_warn(f"📦 [PLACE_DEBUG] 목표 관절각도: J1={target_place_pose[0]:.3f}, J2={target_place_pose[1]:.3f}, J3={target_place_pose[2]:.3f}, J4={target_place_pose[3]:.3f}, J5={target_place_pose[4]:.3f}, J6={target_place_pose[5]:.3f}")
carb.log_warn(f"📦 [PLACE_START] {detected_color.upper()} 상자({klt_name})로 이동합니다.")
```

**배치 시 출력 예시:**
```
📦 [PLACE_DEBUG] 색상: RED
📦 [PLACE_DEBUG] 목표 KLT 상자: small_KLT
📦 [PLACE_DEBUG] 목표 관절각도: J1=-3.400, J2=-1.050, J3=1.500, J4=-2.000, J5=-1.570, J6=-0.280
📦 [PLACE_START] RED 상자(small_KLT)로 이동합니다.
```

---

### 4. **배치 완료 확인 메시지 추가** (245줄)

```python
carb.log_warn(f"✨ [PLACE_COMPLETE] {detected_color.upper()} 상자에 성공적으로 배치했습니다!")
```

---

## 로그 출력 순서

전체 작업 흐름의 로그 출력 순서:

```
🎯 [ARRIVAL] 큐브 도착!
👁️‍🗨️ [VISION] 인식된 책 색상: RED
[COLOR_DEBUG] 색상 감지 픽셀 수 - RED:   567, BLUE:   123, YELLOW:   45
✅ [COLOR_DETECTION] 인식된 색상: RED (픽셀 수: 567)
📦 [PLACE_DEBUG] 색상: RED
📦 [PLACE_DEBUG] 목표 KLT 상자: small_KLT
📦 [PLACE_DEBUG] 목표 관절각도: J1=-3.400, J2=-1.050, J3=1.500, J4=-2.000, J5=-1.570, J6=-0.280
📦 [PLACE_START] RED 상자(small_KLT)로 이동합니다.
✨ [PLACE_COMPLETE] RED 상자에 성공적으로 배치했습니다!
```

---

## 컨솔 색상 코드 사용

실행 중에 다음 이모지를 통해 각 단계를 쉽게 구분할 수 있습니다:

- 🎯 [ARRIVAL] - 큐브 도착
- 👁️‍🗨️ [VISION] - 색상 인식 단계
- [COLOR_DEBUG] - RGB 픽셀 수
- ✅ [COLOR_DETECTION] - 최종 인식된 색상
- 📦 [PLACE_DEBUG] - 배치 목표 정보
- 📦 [PLACE_START] - 배치 시작
- ✨ [PLACE_COMPLETE] - 배치 완료
- ❌ [CAMERA_ERROR] - 카메라 에러
- ⚠️ [COLOR_WARNING] - 색상 인식 불확실

---

## 디버깅 팁

1. **색상 인식이 잘 안될 때**: `[COLOR_DEBUG]` 메시지에서 각 색상의 픽셀 수를 확인
   - 원하는 색상의 픽셀 수가 500 이상이어야 유효 판정
   - 500 미만이면 기본값(red)으로 설정

2. **배치 위치 확인**: `[PLACE_DEBUG]` 메시지에서 관절각도를 확인
   - RED: J1=-3.400 (small_KLT)
   - BLUE: J1=-3.200 (small_KLT_01)
   - YELLOW: J1=-3.000 (small_KLT_02)

3. **카메라 문제**: `[CAMERA_ERROR]` 메시지가 나타나면 카메라 경로(`CAMERA_PRIM_PATH`) 확인
