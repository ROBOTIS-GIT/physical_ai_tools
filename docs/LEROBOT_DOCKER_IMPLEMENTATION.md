# LeRobot Docker 격리 및 Zenoh 통신 구현 문서

> **작성일**: 2025-01-16  
> **브랜치**: `feature-lerobot-docker-isolation`  
> **상태**: Docker 빌드 성공, 기본 통신 레이어 구현 완료

---

## 목차

1. [개요](#개요)
2. [구현된 아키텍처](#구현된-아키텍처)
3. [파일 구조](#파일-구조)
4. [Phase별 구현 내용](#phase별-구현-내용)
5. [사용 방법](#사용-방법)
6. [향후 작업](#향후-작업)

---

## 개요

### 목적
- `physical_ai_tools`에서 LeRobot 의존성을 제거하고 Docker 환경으로 격리
- Zenoh 통신을 통해 ROS2 `physical_ai_server`와 LeRobot Docker 컨테이너 간 학습/추론 명령 전달
- Shared Memory 지원으로 고성능 데이터 전송

### 핵심 변경사항
| 항목 | 이전 | 이후 |
|-----|------|------|
| LeRobot 위치 | `lerobot/` (루트) | `third_party/lerobot/` (submodule) |
| LeRobot 브랜치 | `feature-robotis` | `main` |
| 통신 방식 | 직접 Python import | Zenoh RPC/Pub-Sub |
| 의존성 | `physical_ai_server`에 LeRobot 직접 포함 | Docker 컨테이너 격리 |

---

## 구현된 아키텍처

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     physical_ai_server (ROS2 + Zenoh)                    │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ ZenohLeRobotClient                                                │   │
│  │  - connect() / disconnect()                                       │   │
│  │  - start_training() / stop_training() / get_training_status()    │   │
│  │  - start_inference() / stop_inference()                          │   │
│  │  - subscribe_status() / subscribe_actions()                      │   │
│  └───────────────────────────────┬──────────────────────────────────┘   │
│                                  │                                       │
│                                  │ Zenoh Transport:                     │
│                                  │  - 동일 PC: Shared Memory (ipc:host) │
│                                  │  - 원격 PC: TCP                      │
│                                  │                                       │
│                                  │ Key Expressions:                     │
│                                  │  - lerobot/command (Query/Reply)     │
│                                  │  - lerobot/status (Pub/Sub)          │
│                                  │  - lerobot/action (Pub/Sub)          │
└──────────────────────────────────┼──────────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                     LeRobot Docker Container                             │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ LeRobotZenohServer (소스: docker/lerobot/lerobot_zenoh_server.py)│   │
│  │  - Queryable: lerobot/command (RPC 수신)                         │   │
│  │  - Publisher: lerobot/status (상태 발행)                         │   │
│  │  - Publisher: lerobot/action (추론 결과 발행)                    │   │
│  └───────────────────────────────┬──────────────────────────────────┘   │
│                                  │                                       │
│  ┌───────────────────────────────▼──────────────────────────────────┐   │
│  │ 명령 처리                                                         │   │
│  │  - train_start/stop/resume/status                                │   │
│  │  - infer_start/stop/status                                       │   │
│  │  - model_load/unload/list                                        │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  볼륨 매핑:                                                              │
│  - /dev/shm:/dev/shm (Shared Memory)                                    │
│  - ./workspace/lerobot:/root/.cache/huggingface/lerobot                 │
│  - ./huggingface:/root/.cache/huggingface                               │
│  - ../../zenoh_ros2_sdk:/zenoh_sdk:ro                                   │
└─────────────────────────────────────────────────────────────────────────┘
```

> **Note**: `LeRobotZenohServer`는 `physical_ai_tools/docker/lerobot/` 내에 위치하며,
> LeRobot submodule(`third_party/lerobot/`)과는 완전히 분리됩니다.
> Docker 빌드 시 컨테이너로 복사되므로 LeRobot에 커밋할 필요가 없습니다.

---

## 파일 구조

```
physical_ai_tools/
├── .gitmodules                          # submodule 설정
├── third_party/
│   └── lerobot/                         # LeRobot submodule (main 브랜치)
│
├── docker/
│   ├── docker-compose.yml               # lerobot 서비스 추가됨
│   ├── lerobot/
│   │   ├── Dockerfile                   # LeRobot + Zenoh Docker 이미지
│   │   ├── entrypoint.sh                # 컨테이너 엔트리포인트
│   │   └── lerobot_zenoh_server.py      # Zenoh 서버 (컨테이너 내 실행)
│   └── workspace/
│       └── lerobot/                     # 볼륨 마운트 (datasets, models)
│
├── physical_ai_server/
│   └── physical_ai_server/
│       ├── communication/
│       │   └── zenoh_lerobot_client.py  # Zenoh 클라이언트 (ROS2 측)
│       ├── training/
│       │   └── zenoh_training_manager.py # Training 관리 (Zenoh 통신)
│       └── inference/
│           └── zenoh_inference_manager.py # Inference 관리 (Zenoh 통신)
│
├── scripts/
│   └── update_lerobot.sh                # LeRobot submodule 업데이트 스크립트
│
└── docs/
    ├── LEROBOT_DOCKER_PLAN.md           # 초기 계획 문서
    └── LEROBOT_DOCKER_IMPLEMENTATION.md # 본 문서
```

---

## Phase별 구현 내용

### Phase 1: LeRobot Submodule 재구성 ✅

**작업 내용:**
- `lerobot/` → `third_party/lerobot/`로 이동
- `feature-robotis` → `main` 브랜치로 변경
- `.gitmodules` 업데이트

**결과 `.gitmodules`:**
```ini
[submodule "third_party/lerobot"]
    path = third_party/lerobot
    url = https://github.com/huggingface/lerobot.git
    branch = main
```

---

### Phase 2: LeRobot Docker 컨테이너 구축 ✅

**Dockerfile 주요 내용** (`docker/lerobot/Dockerfile`):

```dockerfile
FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

# 시스템 의존성 (cmake, ninja-build, pkg-config for egl-probe)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git curl wget ca-certificates \
    cmake pkg-config ninja-build \
    libglib2.0-0 libegl1-mesa libegl1-mesa-dev \
    libgl1-mesa-glx libgles2-mesa ffmpeg \
    libusb-1.0-0-dev speech-dispatcher libgeos-dev portaudio19-dev \
    python3.11 python3.11-venv python3.11-dev python3-pip

# uv (빠른 Python 패키지 매니저)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# LeRobot 설치 (setup.py, MANIFEST.in 포함 - 동적 readme 해결)
COPY third_party/lerobot/setup.py third_party/lerobot/pyproject.toml \
     third_party/lerobot/README.md third_party/lerobot/MANIFEST.in ./
COPY third_party/lerobot/src/ ./src/
RUN uv pip install --no-cache ".[all]"

# Zenoh 의존성
RUN pip install eclipse-zenoh>=1.0.0 rosbags>=0.11.0 GitPython>=3.1.18
```

**docker-compose.yml lerobot 서비스:**

```yaml
lerobot:
  container_name: lerobot_server
  image: robotis/lerobot-zenoh:latest
  build:
    context: ..
    dockerfile: docker/lerobot/Dockerfile
  
  network_mode: host      # Zenoh 통신
  ipc: host               # Shared Memory
  runtime: nvidia         # GPU 지원
  
  volumes:
    - /dev/shm:/dev/shm
    - ./workspace/lerobot:/root/.cache/huggingface/lerobot
    - ./huggingface:/root/.cache/huggingface
    - ../../zenoh_ros2_sdk:/zenoh_sdk:ro
  
  environment:
    - ZENOH_ROUTER_IP=127.0.0.1
    - ZENOH_ROUTER_PORT=7447
    - ZENOH_SHM_ENABLED=true
    - ZENOH_SDK_PATH=/zenoh_sdk
```

---

### Phase 3: Zenoh 통신 레이어 구현 ✅

**통신 프로토콜:**

| Key Expression | 방식 | 용도 |
|----------------|------|------|
| `lerobot/command` | Query/Reply (RPC) | 명령 전송 및 응답 |
| `lerobot/status` | Pub/Sub | 상태 브로드캐스트 |
| `lerobot/action` | Pub/Sub | 추론 결과 (Mode 1) |

**명령 타입:**

```python
class CommandType(Enum):
    # Training
    TRAIN_START = "train_start"
    TRAIN_STOP = "train_stop"
    TRAIN_RESUME = "train_resume"
    TRAIN_STATUS = "train_status"
    
    # Inference
    INFER_START = "infer_start"
    INFER_STOP = "infer_stop"
    INFER_STATUS = "infer_status"
    
    # Model Management
    MODEL_LOAD = "model_load"
    MODEL_UNLOAD = "model_unload"
    MODEL_LIST = "model_list"
```

**요청/응답 포맷 (JSON):**

```json
// Request
{
  "command": "train_start",
  "params": {
    "policy_type": "act",
    "dataset_path": "lerobot/aloha_static_coffee",
    "output_dir": "/outputs/train/act_coffee",
    "num_epochs": 100,
    "batch_size": 8
  },
  "request_id": "uuid-xxx"
}

// Response
{
  "success": true,
  "message": "Training started",
  "data": {"pid": 12345},
  "request_id": "uuid-xxx"
}
```

---

### Phase 4: 의존성 분리 파일 생성 ✅

**ZenohLeRobotClient** (`physical_ai_server/communication/zenoh_lerobot_client.py`):
- `physical_ai_server`에서 사용하는 클라이언트
- Raw Zenoh 세션 사용 (ROS2와 독립적)
- Query/Reply로 명령 전송, Subscriber로 상태 수신

**ZenohTrainingManager** (`physical_ai_server/training/zenoh_training_manager.py`):
- LeRobot Docker에 학습 명령 전송
- 학습 상태 모니터링

**ZenohInferenceManager** (`physical_ai_server/inference/zenoh_inference_manager.py`):
- LeRobot Docker에 추론 명령 전송
- 추론 결과 수신

---

### Phase 5: 업데이트 스크립트 ✅

**`scripts/update_lerobot.sh`:**

```bash
#!/bin/bash
# LeRobot submodule을 최신 main으로 업데이트

cd third_party/lerobot
git fetch origin main
git checkout main
git pull origin main

echo "LeRobot updated to: $(git rev-parse HEAD)"
```

---

## 사용 방법

### Docker 이미지 빌드

```bash
cd physical_ai_tools
docker compose -f docker/docker-compose.yml build lerobot
```

### 컨테이너 실행

```bash
# 서버 모드 (기본)
docker compose -f docker/docker-compose.yml up lerobot

# 대화형 셸
docker compose -f docker/docker-compose.yml run lerobot bash
```

### Python 클라이언트 사용 예시

```python
from physical_ai_server.communication.zenoh_lerobot_client import ZenohLeRobotClient

# 연결
client = ZenohLeRobotClient()
client.connect()

# 학습 시작
response = client.start_training(
    policy_type="act",
    dataset_path="lerobot/aloha_static_coffee",
    output_dir="/outputs/train/my_model",
    num_epochs=100
)
print(f"Training started: {response.success}")

# 상태 구독
def on_status(status):
    print(f"Status: {status}")

client.subscribe_status(on_status)

# 학습 중지
client.stop_training()

# 연결 종료
client.disconnect()
```

---

## zenoh_ros2_sdk 업데이트 (v0.1.2)

### 주요 변경사항

1. **`resolve_domain_id()` 함수 추가**
   - `domain_id=None`일 때 `ROS_DOMAIN_ID` 환경변수 사용
   - 기본값: 0

2. **생성자 시그니처 변경**
   ```python
   # 이전
   ROS2Publisher(topic, msg_type, domain_id=0, ...)
   
   # 이후
   ROS2Publisher(topic, msg_type, domain_id=None, ...)  # 환경변수 우선
   ```

3. **음수 `domain_id` 검증 추가**
   ```python
   if domain_id < 0:
       raise ValueError(f"domain_id must be non-negative, got {domain_id}")
   ```

4. **Python 3.8+ 호환성**
   - `slotted_dataclass` 유틸리티 추가

### physical_ai_tools 영향

현재 `ZenohLeRobotClient`와 `LeRobotZenohServer`는 raw Zenoh를 사용하므로 SDK 업데이트의 직접적인 영향 없음.

---

## 향후 작업

### 필수 (P0)
- [ ] `physical_ai_server` 기존 TrainingManager/InferenceManager에서 Zenoh 클라이언트 통합
- [ ] ROS2 서비스 인터페이스 정의 (`physical_ai_interfaces`)
- [ ] E2E 테스트: 학습 명령 → Docker 실행 → 상태 반환

### 권장 (P1)
- [ ] GR00T, OpenVLA 등 추가 모델 지원을 위한 확장 구조
- [ ] 모델 레지스트리 구현 (사용 가능한 모델 목록 관리)
- [ ] 학습 진행률 콜백 (LeRobot train 함수 래핑)

### 선택 (P2)
- [ ] 실시간 Inference 모드 (카메라 토픽 직접 구독)
- [ ] Mode 1: Action 직접 발행 (로봇에 바로 전달)
- [ ] Web UI에서 학습 상태 실시간 모니터링

---

## Git 커밋 이력

| 커밋 | 내용 |
|-----|------|
| `20cbac1` | LeRobot submodule 재구성, Docker 컨테이너 구축, Zenoh 통신 레이어 |
| `16d0060` | Docker 빌드 수정 (setup.py, MANIFEST.in, cmake 의존성 추가) |

---

## 참고 자료

- [LeRobot 공식 문서](https://huggingface.co/docs/lerobot)
- [Zenoh 공식 문서](https://zenoh.io/docs)
- [zenoh_ros2_sdk](https://github.com/ROBOTIS-GIT/zenoh_ros2_sdk)
- [기존 계획 문서](./LEROBOT_DOCKER_PLAN.md)
