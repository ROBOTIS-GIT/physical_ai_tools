# Feature: ROSbag to LeRobot Dataset Converter

> **브랜치**: `feature/rosbag-to-lerobot-converter`  
> **Base**: `feature-1.0.0`  
> **상태**: 🚧 개발 중  
> **담당**: AI Assistant (Sisyphus)  
> **최종 수정**: 2026-01-12  
> **⚠️ 이 문서는 feature-1.0.0 머지 시 삭제됩니다**

---

## 📋 목차

1. [개요](#1-개요)
2. [목표](#2-목표)
3. [기술적 배경](#3-기술적-배경)
4. [구현 상세](#4-구현-상세)
5. [데이터 품질 기준](#5-데이터-품질-기준)
6. [구현 진행 상황](#6-구현-진행-상황)
7. [파일 구조](#7-파일-구조)
8. [사용 방법](#8-사용-방법)
9. [검증 방법](#9-검증-방법)
10. [향후 계획](#10-향후-계획)

---

## 1. 개요

### 1.1 문제 정의

Physical AI Tools에서 수집한 ROSbag 데이터를 LeRobot 학습 프레임워크에서 사용하려면 데이터 포맷 변환이 필요합니다. 이 과정에서 다음 문제들을 해결해야 합니다:

1. **멀티모달 데이터 동기화**: Video, JointState, Action이 각각 다른 주기로 수집됨
2. **인과율(Causality) 보장**: 학습 데이터에 미래 정보가 포함되면 안 됨
3. **데이터 품질 검증**: 센서 데이터 누락/지연 감지 및 리포트
4. **확장성**: LeRobot v3.0, RLDS 등 다른 포맷으로의 확장 고려

### 1.2 해결 방안

- **Causal Sync Algorithm**: Video timestamp 기준, 이전 시점의 가장 최근 데이터만 사용
- **Quality Check Pipeline**: 데이터 gap/drop 분석 및 상세 리포트 생성
- **Plugin Architecture**: 다양한 출력 포맷을 지원하는 확장 가능한 구조

---

## 2. 목표

### 2.1 핵심 목표

| 목표 | 설명 | 상태 |
|------|------|------|
| ROSbag → LeRobot v2.1 변환 | MCAP + MP4 → Parquet + Video | ✅ 기본 구현 완료 |
| Causal Sync | 미래 데이터 사용 방지 | 🚧 구현 예정 |
| Quality Report | 데이터 품질 분석 리포트 | 🚧 구현 예정 |
| Timeline Visualizer | 동기화 상태 시각화 (React) | 📋 계획됨 |

### 2.2 비목표 (Scope 외)

- ROSbag Recorder 성능 개선 (별도 작업으로 분리)
- LeRobot v3.0 지원 (향후 확장)
- HuggingFace Hub 자동 업로드 (향후 확장)

---

## 3. 기술적 배경

### 3.1 데이터 동기화 문제

```
ROSbag 원본 데이터 (비동기, 불규칙한 타임스탬프)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Video (30Hz):       ▮     ▮     ▮     ▮     ▮     ▮
                    0ms   33ms  66ms  100ms 133ms 166ms

JointState (100Hz): ████████████████████████████████████
                    0  10  20  30  40  50  60  70  80  90 ...

Action (100Hz):     ████████████████████████████████████
                    0  10  20  30  40  50  60  70  80  90 ...
```

### 3.2 Causal Sync vs Nearest Neighbor

| 방식 | 설명 | 문제점 |
|------|------|--------|
| **Nearest Neighbor** | 가장 가까운 timestamp 선택 | 미래 데이터 사용 가능 (인과율 위반) |
| **Causal (Previous)** | 현재 시점 이전의 가장 최근 데이터 | 약간의 지연 발생 (허용 가능) |

**Causal Sync가 필요한 이유:**
- 로봇 Inference 시에는 과거 센서 데이터만 사용 가능
- 미래 데이터로 학습하면 train-inference gap 발생
- 실제 로봇 환경과 동일한 조건으로 학습해야 함

### 3.3 Staleness (데이터 지연)

```
Target time: 100ms (Video frame timestamp)
Available JointState: [80ms, 90ms, 110ms, 120ms]
                              ↑
                        Causal 선택: 90ms

Staleness = 100ms - 90ms = 10ms (정상)
```

**Staleness가 크면?**
- 센서 데이터 누락 또는 하드웨어 문제 의심
- Warning/Error로 분류하여 리포트에 기록

---

## 4. 구현 상세

### 4.1 Clock Domain 선택

ROSbag에는 두 가지 timestamp가 존재:

| Timestamp | 설명 | 사용 여부 |
|-----------|------|----------|
| `bag_recv_time` | ROSbag이 메시지를 수신한 시간 | ✅ 사용 |
| `header.stamp` | 센서가 발행한 시간 | ❌ 클럭 동기화 문제 |

**발견된 문제**: 테스트 데이터에서 `header.stamp`와 `bag_recv_time`이 약 142일 차이남 (시스템 클럭 미동기화)

**결정**: `bag_recv_time`을 기준으로 동기화 수행

### 4.2 Video Timestamp 추출

ImageMetadata 메시지에서 각 video frame의 timestamp 추출:

```
ROSbag 토픽 구조:
├── /camera_left/.../metadata (rosbag_recorder/msg/ImageMetadata)
│   ├── header.stamp → 원본 이미지 timestamp
│   ├── frame_index → MP4 내 프레임 번호
│   └── video_file_path → MP4 파일 경로
└── videos/
    └── camera_left_*.mp4
```

### 4.3 Causal Sync Algorithm

```python
def _find_previous_value(messages, target_time, feature_name, frame_index):
    """target_time 이전의 가장 최근 값을 찾음 (Causal)"""
    previous_value = None
    previous_time = None
    
    for msg_time, value in messages:
        if msg_time <= target_time:
            previous_value = value
            previous_time = msg_time
        else:
            break  # target_time을 지나면 중단
    
    # Staleness 체크
    if previous_time is not None:
        delay_ms = (target_time - previous_time) * 1000
        if delay_ms > error_threshold:
            record_error(frame_index, feature_name, delay_ms)
        elif delay_ms > warning_threshold:
            record_warning(frame_index, feature_name, delay_ms)
    
    return previous_value
```

### 4.4 Multi-Topic Action 병합

여러 action topic을 하나의 action vector로 병합:

```
/leader/left_arm   → [j1, j2, j3, j4, j5, j6, j7, gripper]  (8 joints)
/leader/right_arm  → [j1, j2, j3, j4, j5, j6, j7, gripper]  (8 joints)
/leader/head       → [j1, j2, lift]                          (3 joints)
                     ─────────────────────────────────────
병합 결과:           [left..., right..., head...]            (19 joints)
```

### 4.5 FPS 다운샘플링

사용자가 원본보다 낮은 FPS로 변환 가능:

```
원본 데이터:
  camera_left:  30Hz
  camera_right: 30Hz  
  zed:          15Hz  ← 최저 Hz
  
허용 target_fps: 15Hz 이하 (1, 3, 5, 10, 15Hz 등)
불가 target_fps: 16Hz 이상 (원본 최저 Hz 초과)
```

**FPS에 따른 MP4 처리 방식:**

| 조건 | 처리 방식 | 속도 |
|------|----------|------|
| `target_fps == source_fps` | MP4 직접 복사 | ⚡ 빠름 |
| `target_fps < source_fps` | ffmpeg 재인코딩 | 🐢 느림 |

**CLI 출력 예시:**
```bash
$ python convert_rosbag_to_lerobot.py --input ... --fps 10

[INFO] Source video FPS: 30Hz
[INFO] Target FPS: 10Hz
[WARN] FPS 변환이 필요합니다. 비디오 재인코딩으로 인해 변환 시간이 증가합니다.
[INFO] Encoding camera_left... (30fps → 10fps)
[INFO] Encoding camera_right... (30fps → 10fps)
```

### 4.6 LeRobot 의존성 분리

**목표:** physical_ai_tools에서 LeRobot 의존성 제거

```
현재 구조:
  physical_ai_server/
  ├── rosbag_to_lerobot_converter.py  ← LeRobot 의존성 없음 ✅
  ├── lerobot_dataset_wrapper.py      ← LeRobot 의존성 있음 (삭제 예정)
  └── data_manager.py                 ← LeRobot import 있음 (제거 예정)

목표 구조:
  physical_ai_server/
  └── data_processing/
      ├── rosbag_to_lerobot_converter.py  ← pyarrow, numpy, cv2만 사용
      └── quality_analyzer.py              ← 표준 라이브러리만 사용
      
  (LeRobot Docker - 분리됨)
  └── Training, Inference 전용
```

**변환에 필요한 최소 의존성:**
- `pyarrow` (Parquet 쓰기)
- `numpy` (수치 연산)
- `opencv-python` (video dimensions, 재인코딩)
- `rosbags` (ROSbag 읽기)

---

## 5. 데이터 품질 기준

### 5.1 임계값 정의

| 센서 | 주기(Hz) | 간격(ms) | Warning (×2) | Error (×4) |
|------|----------|----------|--------------|------------|
| Video (30fps) | 30 | 33.3ms | >66ms | >133ms |
| JointState | 100 | 10ms | >20ms | >40ms |
| Action | 100 | 10ms | >20ms | >40ms |

**공식:**
- Warning 임계값 = `expected_interval × 2`
- Error 임계값 = `expected_interval × 4`

### 5.2 품질 정책

**기본 정책: `permissive`**
- Warning/Error 발생 시 로그 출력
- 변환은 계속 진행
- 상세 리포트 생성

```python
@dataclass
class QualityPolicy:
    warning_threshold_multiplier: float = 2.0
    error_threshold_multiplier: float = 4.0
    on_warning: str = "log"      # 로그만 기록
    on_error: str = "log"        # 로그만 기록 (변환 계속)
```

### 5.3 품질 리포트 구조

```json
{
  "source_bag": "/path/to/rosbag",
  "output_dataset": "/path/to/lerobot_dataset",
  "sync_strategy": "causal",
  
  "quality_summary": {
    "total_frames": 443,
    "warning_frames": 12,
    "error_frames": 3,
    "overall_quality": "WARNING"
  },
  
  "staleness_report": {
    "observation.state": {
      "max_delay_ms": 45,
      "mean_delay_ms": 8.2,
      "warnings": 5,
      "errors": 1
    },
    "action": {
      "max_delay_ms": 38,
      "mean_delay_ms": 7.8,
      "warnings": 4,
      "errors": 0
    }
  },
  
  "gap_analysis": {
    "/camera_left/metadata": {
      "expected_interval_ms": 33.3,
      "actual_mean_interval_ms": 33.5,
      "max_gap_ms": 68,
      "gaps_detected": 2
    }
  },
  
  "detailed_issues": [
    {"frame": 45, "feature": "observation.state", "delay_ms": 45, "severity": "warning"},
    {"frame": 89, "feature": "action", "delay_ms": 142, "severity": "error"}
  ]
}
```

---

## 6. 구현 진행 상황

### 6.1 완료된 작업

| 커밋 | 내용 |
|------|------|
| `e6d8589` | 기본 converter 구현 (ROSbag → LeRobot v2.1) |
| `1927081` | Multi-topic action 병합, video dimensions 수정 |
| `9c1a617` | videos/ 서브디렉토리 검색 추가 |
| `a23e423` | LeRobot v2.1 stats 포맷 수정 |

### 6.2 진행 예정 작업

| Phase | 내용 | 예상 기간 | 상태 |
|-------|------|----------|------|
| Phase 1 | Quality Check (Gap/Drop 분석 + Report) | 1일 | 📋 예정 |
| Phase 2 | Causal Sync + Staleness Detection | 1일 | 📋 예정 |
| Phase 3 | 실제 데이터 분석 (현재 품질 상태 확인) | 0.5일 | 📋 예정 |
| Phase 4 | Timeline Visualizer (React) | 2일 | 📋 예정 |
| Phase 5 | Plugin Architecture Refactoring | 2일 | 📋 예정 |

---

## 7. 파일 구조

```
physical_ai_tools/
├── physical_ai_server/
│   ├── physical_ai_server/
│   │   └── data_processing/
│   │       ├── rosbag_to_lerobot_converter.py   # 메인 converter
│   │       ├── causal_sync_engine.py            # (예정) Sync 엔진
│   │       ├── quality_analyzer.py              # (예정) 품질 분석
│   │       └── quality_report.py                # (예정) 리포트 생성
│   ├── scripts/
│   │   └── convert_rosbag_to_lerobot.py         # CLI 스크립트
│   └── tests/
│       └── data_processing/
│           └── test_rosbag_to_lerobot_converter.py
│
├── physical_ai_manager/
│   └── src/
│       └── components/
│           └── sync/
│               └── TimelineSyncVisualizer.js    # (예정) React 시각화
│
└── FEATURE_ROSBAG_TO_LEROBOT.md                 # 이 문서
```

---

## 8. 사용 방법

### 8.1 기본 변환

```bash
# Docker 컨테이너 내에서 실행
docker exec -it physical_ai_server bash

# ROS2 환경 설정
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

# 변환 실행
cd /root/ros2_ws/src/physical_ai_tools/physical_ai_server/scripts
python convert_rosbag_to_lerobot.py \
    --input /workspace/rosbag2/episode_001 \
    --output /workspace/lerobot_datasets/my_dataset \
    --repo-id my_org/my_dataset \
    --fps 30
```

### 8.2 여러 ROSbag 변환

```bash
python convert_rosbag_to_lerobot.py \
    --input-dir /workspace/rosbag2/session_001 \
    --output /workspace/lerobot_datasets/session_001 \
    --repo-id my_org/session_001 \
    --fps 30 \
    --robot-type ai_worker
```

### 8.3 품질 리포트 포함 (예정)

```bash
python convert_rosbag_to_lerobot.py \
    --input /workspace/rosbag2/episode_001 \
    --output /workspace/lerobot_datasets/my_dataset \
    --quality-report \
    --sync-strategy causal
```

---

## 9. 검증 방법

### 9.1 LeRobot 로딩 테스트

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(
    repo_id='test/my_dataset',
    root='/workspace/lerobot_datasets/my_dataset'
)

print(f"Episodes: {dataset.num_episodes}")
print(f"Frames: {dataset.num_frames}")
print(f"Features: {list(dataset.features.keys())}")

# 샘플 데이터 확인
sample = dataset[0]
print(f"observation.state shape: {sample['observation.state'].shape}")
print(f"action shape: {sample['action'].shape}")
```

### 9.2 품질 리포트 검증 (예정)

```bash
# 변환 후 리포트 확인
cat /workspace/lerobot_datasets/my_dataset/conversion_report.json | jq .
```

### 9.3 Timeline Visualization (예정)

physical_ai_manager UI에서 변환 결과 시각화 확인

---

## 10. 향후 계획

### 10.1 단기 (이번 Feature)

- [ ] Causal Sync 구현
- [ ] Quality Report 생성
- [ ] FPS 다운샘플링 + ffmpeg 재인코딩
- [ ] Timeline Visualizer (React)
- [ ] LeRobot 의존성 제거 (lerobot_dataset_wrapper.py 삭제)

### 10.2 중기 (v1.0.0 이후)

- [ ] LeRobot v3.0 지원
- [ ] RLDS 포맷 지원
- [ ] HuggingFace Hub 자동 업로드

### 10.3 장기

- [ ] ROSbag Recorder 성능 개선 (별도 Feature)
- [ ] 실시간 품질 모니터링
- [ ] 데이터 증강 파이프라인 연동

---

## 📝 변경 이력

| 날짜 | 내용 |
|------|------|
| 2026-01-12 | 문서 초기 작성 |
| 2026-01-12 | 품질 기준 확정 (Warning: ×2, Error: ×4) |
| 2026-01-12 | 기본 정책 확정 (permissive) |
| 2026-01-12 | FPS 다운샘플링 정책 추가 (동일 FPS=복사, 다른 FPS=재인코딩) |
| 2026-01-12 | LeRobot 의존성 분리 계획 추가 |

---

> **Note**: 이 문서는 `feature/rosbag-to-lerobot-converter` 브랜치가 `feature-1.0.0`에 머지될 때 삭제됩니다.
