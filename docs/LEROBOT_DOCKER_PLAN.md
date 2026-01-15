# LeRobot Docker 격리 및 Zenoh 통신 구현 계획

> **작성일**: 2025-01-14  
> **목적**: physical_ai_tools에서 LeRobot 의존성을 제거하고, Docker 환경으로 격리하여 Zenoh 통신으로 학습/추론 명령을 주고받는 아키텍처 구현

---

## 📋 목차

1. [현재 상태 분석](#현재-상태-분석)
2. [목표 아키텍처](#목표-아키텍처)
3. [Phase 1: LeRobot Submodule 재구성](#phase-1-lerobot-submodule-재구성)
4. [Phase 2: LeRobot Docker 컨테이너 구축](#phase-2-lerobot-docker-컨테이너-구축)
5. [Phase 3: Zenoh 통신 레이어 구현](#phase-3-zenoh-통신-레이어-구현)
6. [Phase 4: physical_ai_server 의존성 분리](#phase-4-physical_ai_server-의존성-분리)
7. [Phase 5: 테스트 및 자동 업데이트](#phase-5-테스트-및-자동-업데이트)
8. [확인이 필요한 사항](#확인이-필요한-사항)
9. [코멘트 섹션](#코멘트-섹션)

---

## 현재 상태 분석

### physical_ai_tools 구조

```
physical_ai_tools/
├── lerobot/                    # 현재: 직접 포함된 폴더 (submodule 아님) --> 
│   └── (feature-robotis 브랜치, 커밋: 989f3d05)
├── physical_ai_server/
│   ├── training/
│   │   ├── training_manager.py      # LeRobot에 강하게 의존
│   │   └── trainers/lerobot/
│   ├── inference/
│   │   └── inference_manager.py     # 7개 정책 클래스 직접 import
│   └── data_processing/
│       └── lerobot_dataset_wrapper.py
├── physical_ai_manager/
├── Isaac-GR00T/
└── docker/
    └── docker-compose.yml
```

--> 코멘트:  현재 이렇게 submodule로 설정 되어 있음. 
[submodule "lerobot"]
	path = lerobot
	url = https://github.com/huggingface/lerobot.git
  branch = feature-robotis
 

### 주요 LeRobot 의존성 (제거 대상)

| 파일 | 의존성 내용 |
|-----|------------|
| `training_manager.py` | `import lerobot`, `from lerobot.configs.train import TrainPipelineConfig` |
| `inference_manager.py` | 7개 정책 클래스 직접 import (ACT, Diffusion, PI0, PI0FAST, TDMPC, VQBeT, SmolVLA) |
| `lerobot_trainer.py` | LeRobot의 모든 학습 유틸리티 사용 |
| `lerobot_dataset_wrapper.py` | `LeRobotDataset` 확장 |
| `data_manager.py` | `from lerobot.datasets.utils import DEFAULT_FEATURES` |
| `evaluation_manager.py` | `from lerobot.configs.default import DatasetConfig` |

--> 코멘트: training manager의 경우, 앞으로 zenoh로 traning 관련 명령을 보내는 용도로 하면 좋을 것 같아. 그리고 앞으로 다양하게 opensource model들이 추가가 될 것이거든.
그래서 우리가 어떤 모델이 추가되었는지, 사용자들은 어떤 모델을 지금 사용할 수 있는지에 대한 정보들이 정리되어 있으면 좋을 것 같아.


### zenoh_ros2_sdk 현황

- **위치**: `/home/dongyun/main_ws/zenoh_ros2_sdk`
- **기능**: ROS2 없이 Zenoh를 통해 ROS2 토픽 pub/sub 가능
- **지원**: Publisher, Subscriber, ServiceClient, ServiceServer
- **제한**: Shared Memory 설정 미구현 (추가 필요)

--> 코멘트: 현재 아마 service 부분은 구현아 안되어 있을거야. 한번 확인해줘. 추가로 명령에 필요한 부분이 있으면 ROS2 데이터 타입으로해서 구현해야할 것 같아.
현재 아마 기본 data type을 예시로 만들어둔게 있을거야. 그거를 참고해서 어떻게 추가적인 data type과 custom data를 추가할지에 대해서 알아보면 좋겠어.

### LeRobot 브랜치 상태

| 항목 | 현재 | 목표 |
|-----|------|------|
| 원격 | `https://github.com/huggingface/lerobot.git` | 동일 |
| 브랜치 | `feature-robotis` | `main` |
| 커밋 | 989f3d05 | 15724826 (최신 main) |

---

## 목표 아키텍처

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        physical_ai_server (ROS2)                         │
│  ┌──────────────────┐           ┌─────────────────────────────────────┐ │
│  │ TrainingManager  │──────────▶│ ZenohTrainingClient                 │ │
│  │ InferenceManager │           │ (zenoh_ros2_sdk) --> 코멘트:  physical_ai_server는 무조건 ROS2에 zenoh를 써야해. zenoh sdk는 쓰지 않을거야. 그리고 따로 Client가 필요 없고, Client는 physical_ai_server 내에 구현해줘. 굳이 다른 프로세스를 또 돌릴 필요 없을 듯해.                   │ │
│  └──────────────────┘           └─────────────────┬───────────────────┘ │
│        ▲                                          │                      │
│        │ ROS2 Services/Topics                     │ Zenoh (TCP/SHM)      │
│        ▼                                          ▼                      │
│  ┌──────────────────┐           ┌─────────────────────────────────────┐ │
│  │ Robot Controller │◀──────────│ Action Output (Mode 2)              │ │
│  └──────────────────┘           └─────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
                                        │
                                        │ Zenoh (TCP / Shared Memory)  
                                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      LeRobot Docker Container                            │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │ Zenoh Subscriber/ServiceServer (zenoh_ros2_sdk - ROS2 없이)     │    │
│  └───────────────────────────────┬─────────────────────────────────┘    │
│                                  │                                       │
│  ┌───────────────────────────────▼─────────────────────────────────┐    │
│  │                    LeRobotZenohServer                            │    │
│  │  ├─ Training: LerobotTrainer 실행, 상태/결과 publish            │    │
│  │  └─ Inference: Policy 로드, 예측 수행                           │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                  │                                       │
│  ┌───────────────────────────────▼─────────────────────────────────┐    │
│  │ Inference 입력: Zenoh로 직접 ROS2 토픽 구독                      │    │
│  │  └─ 이미지, JointState 토픽 → Inference 입력                    │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                  │                                       │
│  ┌───────────────────────────────▼─────────────────────────────────┐    │
│  │ Inference 출력 (두 가지 모드)                                    │    │
│  │  ├─ Mode 1: 직접 Action Topic 발행 → Robot                      │    │
│  │  └─ Mode 2: physical_ai_server로 전달 → Server가 Action 발행    │    │
│  └─────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
```

<!-- 
💬 코멘트: 아키텍처에 대한 의견이나 수정사항

-->

---

## Phase 1: LeRobot Submodule 재구성

**예상 소요 시간**: 1시간

### 작업 목록

| 단계 | 작업 | 명령어/설명 |
|-----|------|------------|
| 1.1 | `third_party/` 폴더 생성 | `mkdir -p third_party` |
| 1.2 | 기존 `lerobot/` 폴더 제거 | `git rm -rf lerobot` |
| 1.3 | LeRobot을 submodule로 추가 | `git submodule add https://github.com/huggingface/lerobot.git third_party/lerobot` |
| 1.4 | main 브랜치로 체크아웃 | `cd third_party/lerobot && git checkout main` |
| 1.5 | 최신 커밋으로 pull | `git pull origin main` |
| 1.6 | `.gitmodules` 업데이트 확인 | 경로: `third_party/lerobot`, 브랜치: `main` |
| 1.7 | submodule 커밋 고정 | 현재 main HEAD로 고정 |

### 결과 폴더 구조

```
physical_ai_tools/
├── third_party/
│   └── lerobot/  (git submodule)
│       ├── src/lerobot/
│       ├── docker/
│       └── ...
├── physical_ai_server/
├── physical_ai_manager/
└── ...
```

### `.gitmodules` 예상 내용

```ini
[submodule "third_party/lerobot"]
    path = third_party/lerobot
    url = https://github.com/huggingface/lerobot.git
    branch = main
```

<!-- 
💬 코멘트: Phase 1에 대한 의견

-->

---

## Phase 2: LeRobot Docker 컨테이너 구축

**예상 소요 시간**: 3-4시간

### 2.1 Dockerfile 작성

**파일 위치**: `third_party/lerobot/docker/Dockerfile.zenoh`

```dockerfile
# LeRobot + Zenoh SDK Docker 이미지
# 기반: 기존 Dockerfile.user

ARG PYTHON_VERSION=3.10
FROM python:${PYTHON_VERSION}-slim

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive \
    MUJOCO_GL=egl \
    PATH=/lerobot/.venv/bin:$PATH \
    ZENOH_SHM_ENABLED=true

# 시스템 의존성 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git curl libglib2.0-0 libegl1-mesa ffmpeg \
    libusb-1.0-0-dev speech-dispatcher libgeos-dev portaudio19-dev \
    && curl -LsSf https://astral.sh/uv/install.sh | sh \
    && mv /root/.local/bin/uv /usr/local/bin/uv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 작업 디렉토리 설정
WORKDIR /lerobot

# LeRobot 설치
COPY third_party/lerobot/pyproject.toml third_party/lerobot/README.md third_party/lerobot/MANIFEST.in ./
COPY third_party/lerobot/src/ src/
RUN uv venv && uv pip install --no-cache ".[all]"

# LeRobot 전체 복사
COPY third_party/lerobot/ .

# zenoh_ros2_sdk 설치
WORKDIR /zenoh_sdk
COPY zenoh_ros2_sdk/ .
RUN pip install -e .

# Zenoh 추가 의존성
RUN pip install eclipse-zenoh>=0.10.0 rosbags>=0.11.0 GitPython>=3.1.18

# 통신 서버 스크립트 복사
WORKDIR /app
COPY scripts/lerobot_zenoh_server.py .

# 환경 변수
ENV HF_HOME=/home/.cache/huggingface \
    HF_LEROBOT_HOME=/home/.cache/huggingface/lerobot \
    TORCH_HOME=/home/.cache/torch

# 기본 명령
CMD ["python", "/app/lerobot_zenoh_server.py"]
```
--> 코멘트: Container는 Shared Memory를 지원하도록 해야해. 그리고 /home/dongyun/main_ws/physical_ai_tools/docker/workspace/lerobot를 physical_ai_tools와 공간을 공유해야해. 즉 볼륨 매핑해야한다는거야. 
컨테이너 안에서 볼륨 매핑할 주소는 ~/.cache/huggingface/lerobot  과 매핑해줘. 만약 폴더가 둘다 없으면 둘다 생성해야해.


### 2.2 docker-compose.yml 업데이트

**파일 위치**: `docker/docker-compose.yml`

```yaml
services:
  # 기존 서비스들...
  physical_ai_manager:
    # ... (기존 설정 유지)
    
  physical_ai_server:
    # ... (기존 설정 유지)
  
  # 신규: LeRobot 서비스
  lerobot:
    container_name: lerobot_server
    image: robotis/lerobot-zenoh:latest
    build:
      context: ..
      dockerfile: third_party/lerobot/docker/Dockerfile.zenoh
    
    # Zenoh 통신을 위한 네트워크 설정
    network_mode: host
    
    # Shared Memory 지원
    ipc: host
    
    # GPU 지원
    runtime: nvidia
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              capabilities: [gpu]
    
    # 볼륨 마운트
    volumes:
      # Shared Memory
      - /dev/shm:/dev/shm
      # 데이터셋 공유
      - ./datasets:/datasets
      # 모델 공유
      - ./models:/models
      # 출력 디렉토리
      - ./outputs:/outputs
      # HuggingFace 캐시
      - ./huggingface:/home/.cache/huggingface
      # 타임존
      - /etc/timezone:/etc/timezone:ro
      - /etc/localtime:/etc/localtime:ro
    
    # 환경 변수
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - ZENOH_DOMAIN_ID=30
      - ZENOH_ROUTER_IP=127.0.0.1
      - ZENOH_ROUTER_PORT=7447
    
    # 자동 재시작
    restart: unless-stopped
```
--> 코멘트: Container는 Shared Memory를 지원하도록 해야해. 그리고 /home/dongyun/main_ws/physical_ai_tools/docker/workspace/lerobot를 physical_ai_tools와 공간을 공유해야해. 즉 볼륨 매핑해야한다는거야. 
컨테이너 안에서 볼륨 매핑할 주소는 ~/.cache/huggingface/lerobot  과 매핑해줘. 만약 폴더가 둘다 없으면 둘다 생성해야해.

### 2.3 Shared Memory 설정

**zenoh_ros2_sdk 수정 필요 사항** (`session.py`):

```python
# 현재 설정 (TCP만 지원)
self.conf = zenoh.Config()
self.conf.insert_json5("connect/endpoints", f'["tcp/{router_ip}:{router_port}"]')

# 추가할 Shared Memory 설정
if os.environ.get("ZENOH_SHM_ENABLED", "false").lower() == "true":
    self.conf.insert_json5("transport/shared_memory/enabled", "true")
```

현재 topic sub, pub만 구현되어 있을 텐데 service가 구현되어 있지 않으면 구현해야해.
그리고 training, inference 명령을 받을 msgtype도 추가해야해.

<!-- 
💬 코멘트: Shared Memory 설정에 대한 의견

-->

---

## Phase 3: Zenoh 통신 레이어 구현

**예상 소요 시간**: 4-6시간

### 3.1 메시지/서비스 타입 정의

**파일 위치**: `physical_ai_interfaces/msg/` 및 `physical_ai_interfaces/srv/`

#### Training 관련 메시지

```
# physical_ai_interfaces/msg/TrainCommand.msg
string command_type        # "start", "stop", "resume", "status"
string policy_type         # "act", "diffusion", "pi0fast", "pi0", "tdmpc", "vqbet", "smolvla"
string dataset_path        # 데이터셋 경로 (로컬 또는 HuggingFace repo_id)
string output_dir          # 출력 디렉토리
string policy_device       # "cuda", "cpu"
int32 batch_size           # 배치 크기 (default: 8)
int32 steps                # 총 학습 스텝 (default: 100000)
int32 save_freq            # 체크포인트 저장 주기 (default: 1000)
int32 eval_freq            # 평가 주기 (default: 20000)
int32 log_freq             # 로그 주기 (default: 200)
int32 seed                 # 랜덤 시드 (default: 1000)
int32 num_workers          # 데이터로더 워커 수 (default: 4)
string extra_config        # JSON string for additional config
```

```
# physical_ai_interfaces/msg/TrainStatus.msg
string status              # "idle", "initializing", "training", "evaluating", "saving", "completed", "error", "stopped"
int32 current_step         # 현재 스텝
int32 total_steps          # 총 스텝
float32 current_loss       # 현재 손실값
float32 learning_rate      # 현재 학습률
string checkpoint_path     # 최근 저장된 체크포인트 경로
string error_message       # 에러 발생 시 메시지
float32 progress_percent   # 진행률 (0-100)
```

#### Inference 관련 메시지

```
# physical_ai_interfaces/msg/InferenceCommand.msg
string command_type        # "load", "unload", "configure"
string model_path          # 모델 경로
string output_mode         # "direct" (직접 발행), "server" (서버로 전달)
string[] camera_topics     # 구독할 카메라 토픽 목록
string joint_state_topic   # 구독할 JointState 토픽
string action_topic        # Action 발행 토픽 (direct 모드)
```

```
# physical_ai_interfaces/msg/InferenceInput.msg
sensor_msgs/Image[] images           # 카메라 이미지들
string[] camera_names                # 카메라 이름들 (images와 1:1 대응)
float32[] joint_state                # 관절 상태
string task_instruction              # 작업 지시 (VLA 모델용)
builtin_interfaces/Time timestamp    # 타임스탬프
```

```
# physical_ai_interfaces/msg/InferenceOutput.msg
float32[] action                     # 예측된 액션
bool success                         # 성공 여부
string error_message                 # 에러 메시지
builtin_interfaces/Time timestamp    # 타임스탬프
float32 inference_time_ms            # 추론 소요 시간 (밀리초)
```

#### Inference 서비스

```
# physical_ai_interfaces/srv/Inference.srv
# Request
sensor_msgs/Image[] images
string[] camera_names
float32[] joint_state
string task_instruction
---
# Response
float32[] action
bool success
string error_message
float32 inference_time_ms
```

--> 코멘트: 일단 구현해보고 부족한게 있으면 추가해보자.

### 3.2 LeRobot Docker 내 Zenoh 서버 구현

**파일 위치**: `scripts/lerobot_zenoh_server.py`

```python
#!/usr/bin/env python3
"""
LeRobot Zenoh Server
- Training/Inference 명령 수신 및 실행
- physical_ai_server와 Zenoh로 통신
- ROS2 환경 없이 동작
"""

import os
import json
import threading
import time
from dataclasses import dataclass
from typing import Optional, Dict, List
import numpy as np

from zenoh_ros2_sdk import (
    ROS2Subscriber, ROS2Publisher,
    ROS2ServiceServer, get_message_class
)

# LeRobot imports
from lerobot.configs.train import TrainPipelineConfig
from lerobot.policies.pretrained import PreTrainedPolicy
import torch


@dataclass
class ServerConfig:
    domain_id: int = 30
    router_ip: str = "127.0.0.1"
    router_port: int = 7447


class LeRobotZenohServer:
    """
    LeRobot Docker 컨테이너 내에서 실행되는 Zenoh 서버
    Training과 Inference 요청을 처리
    """
    
    def __init__(self, config: ServerConfig = None):
        self.config = config or ServerConfig()
        self.domain_id = self.config.domain_id
        
        # Training 상태
        self.trainer = None
        self.trainer_thread: Optional[threading.Thread] = None
        self.training_stop_event = threading.Event()
        self.training_status = {
            "status": "idle",
            "current_step": 0,
            "total_steps": 0,
            "current_loss": float('nan'),
            "checkpoint_path": ""
        }
        
        # Inference 상태
        self.inference_policy: Optional[PreTrainedPolicy] = None
        self.inference_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.output_mode = "server"  # "direct" or "server"
        
        # 실시간 토픽 구독 (Inference용)
        self.image_subscribers: Dict[str, ROS2Subscriber] = {}
        self.joint_state_sub: Optional[ROS2Subscriber] = None
        self.latest_images: Dict[str, np.ndarray] = {}
        self.latest_joint_state: Optional[np.ndarray] = None
        
        self._setup_communication()
    
    def _setup_communication(self):
        """Zenoh 통신 설정"""
        
        # === Training 통신 ===
        # Training 명령 수신
        self.train_cmd_sub = ROS2Subscriber(
            topic="/lerobot/train_command",
            msg_type="physical_ai_interfaces/msg/TrainCommand",
            callback=self._on_train_command,
            domain_id=self.domain_id
        )
        
        # Training 상태 발행
        self.train_status_pub = ROS2Publisher(
            topic="/lerobot/train_status",
            msg_type="physical_ai_interfaces/msg/TrainStatus",
            domain_id=self.domain_id
        )
        
        # === Inference 통신 ===
        # Inference 명령 수신
        self.inference_cmd_sub = ROS2Subscriber(
            topic="/lerobot/inference_command",
            msg_type="physical_ai_interfaces/msg/InferenceCommand",
            callback=self._on_inference_command,
            domain_id=self.domain_id
        )
        
        # Inference 서비스 (동기 호출용)
        self.inference_service = ROS2ServiceServer(
            service_name="/lerobot/inference",
            srv_type="physical_ai_interfaces/srv/Inference",
            callback=self._handle_inference_service,
            domain_id=self.domain_id
        )
        
        # Inference 출력 (Mode 2: 서버로 전달)
        self.inference_output_pub = ROS2Publisher(
            topic="/lerobot/inference_output",
            msg_type="physical_ai_interfaces/msg/InferenceOutput",
            domain_id=self.domain_id
        )
        
        # Action 직접 발행 (Mode 1: 로봇에 직접)
        self.action_pub: Optional[ROS2Publisher] = None
        
        print(f"[LeRobotZenohServer] 초기화 완료 (domain_id={self.domain_id})")
    
    # ==================== Training ====================
    
    def _on_train_command(self, msg):
        """Training 명령 처리"""
        cmd_type = msg.command_type
        print(f"[Training] 명령 수신: {cmd_type}")
        
        if cmd_type == "start":
            self._start_training(msg)
        elif cmd_type == "stop":
            self._stop_training()
        elif cmd_type == "resume":
            self._resume_training(msg)
        elif cmd_type == "status":
            self._publish_training_status()
    
    def _start_training(self, config):
        """새 학습 시작"""
        if self.trainer_thread and self.trainer_thread.is_alive():
            print("[Training] 이미 학습 중입니다")
            return
        
        self.training_stop_event.clear()
        self.training_status["status"] = "initializing"
        self.training_status["total_steps"] = config.steps
        self._publish_training_status()
        
        self.trainer_thread = threading.Thread(
            target=self._training_loop,
            args=(config, False)
        )
        self.trainer_thread.start()
    
    def _resume_training(self, config):
        """학습 재개"""
        if self.trainer_thread and self.trainer_thread.is_alive():
            print("[Training] 이미 학습 중입니다")
            return
        
        self.training_stop_event.clear()
        self.training_status["status"] = "initializing"
        self._publish_training_status()
        
        self.trainer_thread = threading.Thread(
            target=self._training_loop,
            args=(config, True)
        )
        self.trainer_thread.start()
    
    def _stop_training(self):
        """학습 중지"""
        if self.trainer_thread and self.trainer_thread.is_alive():
            print("[Training] 학습 중지 요청")
            self.training_stop_event.set()
            self.training_status["status"] = "stopped"
            self._publish_training_status()
    
    def _training_loop(self, config, resume: bool):
        """실제 학습 루프 (별도 스레드)"""
        try:
            from lerobot.scripts.train import train as lerobot_train
            import draccus
            
            # TrainPipelineConfig 생성
            args = [
                f'--policy.type={config.policy_type}',
                f'--policy.device={config.policy_device or "cuda"}',
                f'--dataset.repo_id={config.dataset_path}',
                f'--output_dir={config.output_dir}',
                f'--batch_size={config.batch_size or 8}',
                f'--steps={config.steps or 100000}',
                f'--save_freq={config.save_freq or 1000}',
                f'--eval_freq={config.eval_freq or 20000}',
                f'--log_freq={config.log_freq or 200}',
                f'--seed={config.seed or 1000}',
                f'--num_workers={config.num_workers or 4}',
            ]
            
            if resume:
                args.append('--resume=true')
            
            cfg = draccus.parse(TrainPipelineConfig, None, args=args)
            
            self.training_status["status"] = "training"
            self._publish_training_status()
            
            # 학습 실행 (커스텀 콜백으로 상태 업데이트)
            # TODO: LeRobot train 함수를 수정하거나 래핑하여 콜백 지원
            lerobot_train(cfg)
            
            self.training_status["status"] = "completed"
            
        except Exception as e:
            print(f"[Training] 에러 발생: {e}")
            self.training_status["status"] = "error"
            self.training_status["error_message"] = str(e)
        
        finally:
            self._publish_training_status()
    
    def _publish_training_status(self):
        """Training 상태 발행"""
        # TrainStatus 메시지 생성 및 발행
        self.train_status_pub.publish(
            status=self.training_status["status"],
            current_step=self.training_status["current_step"],
            total_steps=self.training_status["total_steps"],
            current_loss=self.training_status["current_loss"],
            checkpoint_path=self.training_status.get("checkpoint_path", ""),
            error_message=self.training_status.get("error_message", ""),
            progress_percent=(
                self.training_status["current_step"] / 
                max(self.training_status["total_steps"], 1) * 100
            )
        )
    
    # ==================== Inference ====================
    
    def _on_inference_command(self, msg):
        """Inference 명령 처리"""
        cmd_type = msg.command_type
        print(f"[Inference] 명령 수신: {cmd_type}")
        
        if cmd_type == "load":
            self._load_model(msg.model_path)
            self.output_mode = msg.output_mode or "server"
            
            # 실시간 모드 설정
            if msg.camera_topics:
                self._setup_realtime_inference(
                    msg.camera_topics,
                    msg.joint_state_topic,
                    msg.action_topic
                )
                
        elif cmd_type == "unload":
            self._unload_model()
        elif cmd_type == "configure":
            self.output_mode = msg.output_mode or "server"
    
    def _load_model(self, model_path: str) -> bool:
        """모델 로드"""
        try:
            print(f"[Inference] 모델 로드: {model_path}")
            self.inference_policy = PreTrainedPolicy.from_pretrained(model_path)
            self.inference_policy.to(self.inference_device)
            self.inference_policy.eval()
            print(f"[Inference] 모델 로드 완료")
            return True
        except Exception as e:
            print(f"[Inference] 모델 로드 실패: {e}")
            return False
    
    def _unload_model(self):
        """모델 언로드"""
        if self.inference_policy:
            del self.inference_policy
            self.inference_policy = None
            torch.cuda.empty_cache()
            print("[Inference] 모델 언로드 완료")
    
    def _setup_realtime_inference(
        self,
        camera_topics: List[str],
        joint_state_topic: str,
        action_topic: str = None
    ):
        """실시간 Inference를 위한 토픽 구독 설정"""
        
        # 기존 구독 정리
        for sub in self.image_subscribers.values():
            sub.close()
        self.image_subscribers.clear()
        
        if self.joint_state_sub:
            self.joint_state_sub.close()
        
        # 카메라 토픽 구독
        for i, topic in enumerate(camera_topics):
            camera_name = f"camera_{i}"
            self.image_subscribers[camera_name] = ROS2Subscriber(
                topic=topic,
                msg_type="sensor_msgs/msg/Image",
                callback=lambda msg, name=camera_name: self._on_image(msg, name),
                domain_id=self.domain_id
            )
        
        # JointState 구독
        if joint_state_topic:
            self.joint_state_sub = ROS2Subscriber(
                topic=joint_state_topic,
                msg_type="sensor_msgs/msg/JointState",
                callback=self._on_joint_state,
                domain_id=self.domain_id
            )
        
        # Direct 모드: Action 발행자 설정
        if self.output_mode == "direct" and action_topic:
            self.action_pub = ROS2Publisher(
                topic=action_topic,
                msg_type="sensor_msgs/msg/JointState",  # 또는 적절한 Action 메시지 타입
                domain_id=self.domain_id
            )
        
        print(f"[Inference] 실시간 모드 설정 완료 (카메라 {len(camera_topics)}개)")
    
    def _on_image(self, msg, camera_name: str):
        """이미지 메시지 콜백"""
        # sensor_msgs/Image → numpy array 변환
        # TODO: 실제 변환 로직 구현
        self.latest_images[camera_name] = self._image_msg_to_numpy(msg)
    
    def _on_joint_state(self, msg):
        """JointState 메시지 콜백"""
        self.latest_joint_state = np.array(msg.position, dtype=np.float32)
    
    def _image_msg_to_numpy(self, msg) -> np.ndarray:
        """sensor_msgs/Image → numpy 변환"""
        # TODO: 인코딩에 따른 처리 (rgb8, bgr8, mono8 등)
        import numpy as np
        height, width = msg.height, msg.width
        if msg.encoding == "rgb8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif msg.encoding == "bgr8":
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            return img[:, :, ::-1]  # BGR → RGB
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")
    
    def _handle_inference_service(self, request):
        """Inference 서비스 핸들러"""
        if self.inference_policy is None:
            Response = get_message_class("physical_ai_interfaces/srv/Inference_Response")
            return Response(
                action=[],
                success=False,
                error_message="모델이 로드되지 않았습니다"
            )
        
        start_time = time.time()
        
        try:
            # 이미지 변환
            images = {}
            for i, (img_msg, name) in enumerate(zip(request.images, request.camera_names)):
                images[name] = self._image_msg_to_numpy(img_msg)
            
            # 예측
            action = self._predict(
                images=images,
                state=np.array(request.joint_state, dtype=np.float32),
                task_instruction=request.task_instruction
            )
            
            inference_time = (time.time() - start_time) * 1000
            
            Response = get_message_class("physical_ai_interfaces/srv/Inference_Response")
            return Response(
                action=action.tolist(),
                success=True,
                error_message="",
                inference_time_ms=inference_time
            )
            
        except Exception as e:
            Response = get_message_class("physical_ai_interfaces/srv/Inference_Response")
            return Response(
                action=[],
                success=False,
                error_message=str(e)
            )
    
    def _predict(
        self,
        images: Dict[str, np.ndarray],
        state: np.ndarray,
        task_instruction: str = None
    ) -> np.ndarray:
        """Inference 수행"""
        # 전처리
        observation = {}
        
        # 이미지 처리
        for name, img in images.items():
            tensor = torch.from_numpy(img).float() / 255.0
            tensor = tensor.permute(2, 0, 1).unsqueeze(0)  # HWC → BCHW
            tensor = tensor.to(self.inference_device)
            observation[f"observation.images.{name}"] = tensor
        
        # 상태 처리
        state_tensor = torch.from_numpy(state).float().unsqueeze(0)
        state_tensor = state_tensor.to(self.inference_device)
        observation["observation.state"] = state_tensor
        
        # Task instruction (VLA 모델용)
        if task_instruction:
            observation["task"] = [task_instruction]
        
        # 추론
        with torch.inference_mode():
            action = self.inference_policy.select_action(observation)
            action = action.squeeze(0).cpu().numpy()
        
        return action
    
    def run_realtime_inference_loop(self, rate_hz: float = 30.0):
        """실시간 Inference 루프 (별도 스레드로 실행)"""
        period = 1.0 / rate_hz
        
        while True:
            start_time = time.time()
            
            if self.inference_policy and self.latest_images and self.latest_joint_state is not None:
                try:
                    action = self._predict(
                        images=self.latest_images.copy(),
                        state=self.latest_joint_state.copy()
                    )
                    
                    if self.output_mode == "direct" and self.action_pub:
                        # Mode 1: 직접 발행
                        self.action_pub.publish(position=action.tolist())
                    else:
                        # Mode 2: 서버로 전달
                        self.inference_output_pub.publish(
                            action=action.tolist(),
                            success=True
                        )
                        
                except Exception as e:
                    print(f"[Inference] 실시간 추론 에러: {e}")
            
            # Rate 유지
            elapsed = time.time() - start_time
            if elapsed < period:
                time.sleep(period - elapsed)
    
    # ==================== Lifecycle ====================
    
    def run(self):
        """메인 루프"""
        print("[LeRobotZenohServer] 서버 시작...")
        
        # 상태 발행 루프
        try:
            while True:
                # 주기적으로 Training 상태 발행
                if self.training_status["status"] not in ["idle", "completed", "error"]:
                    self._publish_training_status()
                
                time.sleep(1.0)
                
        except KeyboardInterrupt:
            print("[LeRobotZenohServer] 종료 요청")
            self.shutdown()
    
    def shutdown(self):
        """리소스 정리"""
        print("[LeRobotZenohServer] 종료 중...")
        
        # Training 중지
        if self.trainer_thread and self.trainer_thread.is_alive():
            self.training_stop_event.set()
            self.trainer_thread.join(timeout=10)
        
        # 구독자 정리
        self.train_cmd_sub.close()
        self.inference_cmd_sub.close()
        for sub in self.image_subscribers.values():
            sub.close()
        if self.joint_state_sub:
            self.joint_state_sub.close()
        
        # 발행자 정리
        self.train_status_pub.close()
        self.inference_output_pub.close()
        if self.action_pub:
            self.action_pub.close()
        
        # 서비스 정리
        self.inference_service.close()
        
        # 모델 언로드
        self._unload_model()
        
        print("[LeRobotZenohServer] 종료 완료")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain-id", type=int, default=30)
    parser.add_argument("--router-ip", type=str, default="127.0.0.1")
    parser.add_argument("--router-port", type=int, default=7447)
    args = parser.parse_args()
    
    config = ServerConfig(
        domain_id=args.domain_id,
        router_ip=args.router_ip,
        router_port=args.router_port
    )
    
    server = LeRobotZenohServer(config)
    server.run()
```

--> 앞으로 Lerobot 외에도 Gr00t, OpenVLA 등도 통합할거거든. 확장성 있게 작성되면 좋겠어.

### 3.3 physical_ai_server 측 클라이언트

**파일 위치**: `physical_ai_server/physical_ai_server/training/zenoh_training_client.py`

```python
#!/usr/bin/env python3
"""
Zenoh Training Client
physical_ai_server에서 LeRobot Docker와 통신
"""

from zenoh_ros2_sdk import ROS2Publisher, ROS2Subscriber, ROS2ServiceClient


class ZenohTrainingClient:
    """LeRobot Docker의 Training 기능과 통신하는 클라이언트"""
    
    def __init__(self, domain_id: int = 30):
        self.domain_id = domain_id
        self.latest_status = None
        
        # Training 명령 발행
        self.train_cmd_pub = ROS2Publisher(
            topic="/lerobot/train_command",
            msg_type="physical_ai_interfaces/msg/TrainCommand",
            domain_id=domain_id
        )
        
        # Training 상태 구독
        self.train_status_sub = ROS2Subscriber(
            topic="/lerobot/train_status",
            msg_type="physical_ai_interfaces/msg/TrainStatus",
            callback=self._on_status,
            domain_id=domain_id
        )
    
    def _on_status(self, msg):
        """상태 콜백"""
        self.latest_status = msg
    
    def send_train_command(
        self,
        command_type: str,
        policy_type: str = None,
        dataset_path: str = None,
        output_dir: str = None,
        **kwargs
    ):
        """Training 명령 전송"""
        self.train_cmd_pub.publish(
            command_type=command_type,
            policy_type=policy_type or "",
            dataset_path=dataset_path or "",
            output_dir=output_dir or "",
            batch_size=kwargs.get("batch_size", 8),
            steps=kwargs.get("steps", 100000),
            save_freq=kwargs.get("save_freq", 1000),
            eval_freq=kwargs.get("eval_freq", 20000),
            log_freq=kwargs.get("log_freq", 200),
            seed=kwargs.get("seed", 1000),
            num_workers=kwargs.get("num_workers", 4),
            extra_config=kwargs.get("extra_config", "")
        )
    
    def start_training(self, **kwargs):
        """학습 시작"""
        self.send_train_command(command_type="start", **kwargs)
    
    def stop_training(self):
        """학습 중지"""
        self.send_train_command(command_type="stop")
    
    def resume_training(self, **kwargs):
        """학습 재개"""
        self.send_train_command(command_type="resume", **kwargs)
    
    def get_status(self):
        """최근 상태 반환"""
        return self.latest_status
    
    def close(self):
        """리소스 정리"""
        self.train_cmd_pub.close()
        self.train_status_sub.close()
```

**파일 위치**: `physical_ai_server/physical_ai_server/inference/zenoh_inference_client.py`

```python
#!/usr/bin/env python3
"""
Zenoh Inference Client
physical_ai_server에서 LeRobot Docker와 통신
"""

import numpy as np
from zenoh_ros2_sdk import ROS2Publisher, ROS2Subscriber, ROS2ServiceClient


class ZenohInferenceClient:
    """LeRobot Docker의 Inference 기능과 통신하는 클라이언트"""
    
    def __init__(self, domain_id: int = 30):
        self.domain_id = domain_id
        
        # Inference 명령 발행
        self.inference_cmd_pub = ROS2Publisher(
            topic="/lerobot/inference_command",
            msg_type="physical_ai_interfaces/msg/InferenceCommand",
            domain_id=domain_id
        )
        
        # Inference 서비스 클라이언트
        self.inference_client = ROS2ServiceClient(
            service_name="/lerobot/inference",
            srv_type="physical_ai_interfaces/srv/Inference",
            domain_id=domain_id
        )
        
        # Inference 결과 구독 (비동기 모드용)
        self.latest_output = None
        self.inference_output_sub = ROS2Subscriber(
            topic="/lerobot/inference_output",
            msg_type="physical_ai_interfaces/msg/InferenceOutput",
            callback=self._on_output,
            domain_id=domain_id
        )
    
    def _on_output(self, msg):
        """Inference 결과 콜백"""
        self.latest_output = msg
    
    def load_model(self, model_path: str, output_mode: str = "server"):
        """모델 로드 명령"""
        self.inference_cmd_pub.publish(
            command_type="load",
            model_path=model_path,
            output_mode=output_mode
        )
    
    def unload_model(self):
        """모델 언로드 명령"""
        self.inference_cmd_pub.publish(command_type="unload")
    
    def configure_realtime(
        self,
        camera_topics: list,
        joint_state_topic: str,
        action_topic: str = None,
        output_mode: str = "direct"
    ):
        """실시간 모드 설정"""
        self.inference_cmd_pub.publish(
            command_type="load",
            output_mode=output_mode,
            camera_topics=camera_topics,
            joint_state_topic=joint_state_topic,
            action_topic=action_topic or ""
        )
    
    def predict(
        self,
        images: dict,
        state: np.ndarray,
        task_instruction: str = None,
        timeout: float = 10.0
    ) -> np.ndarray:
        """동기 Inference 호출"""
        # 이미지를 sensor_msgs/Image 메시지로 변환
        image_msgs = []
        camera_names = []
        
        for name, img in images.items():
            # numpy → Image 메시지 변환 (간략화)
            # TODO: 실제 변환 로직
            image_msgs.append(self._numpy_to_image_msg(img))
            camera_names.append(name)
        
        # 서비스 호출
        response = self.inference_client.call(
            images=image_msgs,
            camera_names=camera_names,
            joint_state=state.tolist(),
            task_instruction=task_instruction or "",
            timeout=timeout
        )
        
        if response and response.success:
            return np.array(response.action, dtype=np.float32)
        else:
            raise RuntimeError(f"Inference failed: {response.error_message if response else 'No response'}")
    
    def _numpy_to_image_msg(self, img: np.ndarray):
        """numpy → sensor_msgs/Image 변환"""
        from zenoh_ros2_sdk import get_message_class
        Image = get_message_class("sensor_msgs/msg/Image")
        
        height, width = img.shape[:2]
        channels = img.shape[2] if len(img.shape) > 2 else 1
        
        msg = Image(
            height=height,
            width=width,
            encoding="rgb8" if channels == 3 else "mono8",
            step=width * channels,
            data=img.tobytes()
        )
        return msg
    
    def get_latest_output(self):
        """최근 비동기 결과 반환"""
        return self.latest_output
    
    def close(self):
        """리소스 정리"""
        self.inference_cmd_pub.close()
        self.inference_client.close()
        self.inference_output_sub.close()
```

--> 코멘트: 따로 프로세스가 도는건 아니지? 

---

## Phase 4: physical_ai_server 의존성 분리

**예상 소요 시간**: 4-6시간

### 4.1 수정할 파일 목록

| 파일 | 현재 상태 | 수정 내용 |
|-----|----------|----------|
| `training_manager.py` | LeRobot 직접 import | ZenohTrainingClient 사용 |
| `inference_manager.py` | 7개 정책 클래스 import | ZenohInferenceClient 사용 |
| `lerobot_trainer.py` | LeRobot 전체 사용 | 제거 또는 Docker 내부로 이동 |
| `lerobot_dataset_wrapper.py` | LeRobotDataset 확장 | 데이터 포맷만 사용 (선택적 유지) |
| `data_manager.py` | LeRobot import | 최소화 또는 제거 |
| `evaluation_manager.py` | LeRobot import | ZenohInferenceClient로 대체 |

### 4.2 TrainingManager 수정

**파일**: `physical_ai_server/physical_ai_server/training/training_manager.py`

```python
#!/usr/bin/env python3
"""
Training Manager (Modified)
LeRobot Docker와 Zenoh 통신으로 학습 관리
LeRobot 직접 의존성 제거
"""

from pathlib import Path
import threading

from physical_ai_interfaces.msg import TrainingInfo, TrainingStatus
from physical_ai_server.training.zenoh_training_client import ZenohTrainingClient


class TrainingManager:
    """
    LeRobot Docker 컨테이너와 Zenoh로 통신하여 학습 관리
    """

    # 지원 정책 목록 (Docker에서 실제 처리)
    SUPPORTED_POLICIES = [
        'pi0fast', 'pi0', 'diffusion', 'act', 'tdmpc', 'vqbet', 'smolvla'
    ]

    def __init__(self, domain_id: int = 30):
        self.training_info = TrainingInfo()
        self.zenoh_client = ZenohTrainingClient(domain_id=domain_id)
        self.stop_event = threading.Event()

        # Resume 설정
        self.resume = False
        self.resume_model_path = None

    def train(self):
        """학습 시작 (LeRobot Docker로 명령 전송)"""
        if self.resume and self.resume_model_path:
            self.zenoh_client.resume_training(
                policy_type=self.training_info.policy_type,
                dataset_path=self.training_info.dataset,
                output_dir=self.resume_model_path,
                batch_size=self.training_info.batch_size,
                steps=self.training_info.steps,
                save_freq=self.training_info.save_freq,
                eval_freq=self.training_info.eval_freq,
                log_freq=self.training_info.log_freq,
                seed=self.training_info.seed,
                num_workers=self.training_info.num_workers,
            )
        else:
            self.zenoh_client.start_training(
                policy_type=self.training_info.policy_type,
                dataset_path=self.training_info.dataset,
                output_dir=self.training_info.output_folder_name,
                policy_device=self.training_info.policy_device,
                batch_size=self.training_info.batch_size,
                steps=self.training_info.steps,
                save_freq=self.training_info.save_freq,
                eval_freq=self.training_info.eval_freq,
                log_freq=self.training_info.log_freq,
                seed=self.training_info.seed,
                num_workers=self.training_info.num_workers,
            )

    def stop(self):
        """학습 중지"""
        self.zenoh_client.stop_training()
        self.stop_event.set()

    def get_current_training_status(self) -> TrainingStatus:
        """현재 학습 상태 조회"""
        status = self.zenoh_client.get_status()
        
        training_status = TrainingStatus()
        training_status.training_info = self.training_info
        
        if status:
            training_status.current_step = status.current_step
            training_status.current_loss = status.current_loss
            # 추가 필드 매핑...
        
        return training_status

    @staticmethod
    def get_available_list():
        """지원 정책 및 디바이스 목록"""
        policy_list = TrainingManager.SUPPORTED_POLICIES
        device_list = ['cuda', 'cpu']
        return policy_list, device_list

    @staticmethod
    def get_weight_save_root_path():
        """학습 결과 저장 경로 (Docker 볼륨과 일치)"""
        return Path('/outputs/train')
```

### 4.3 InferenceManager 수정

**파일**: `physical_ai_server/physical_ai_server/inference/inference_manager.py`

```python
#!/usr/bin/env python3
"""
Inference Manager (Modified)
LeRobot Docker와 Zenoh 통신으로 추론 관리
LeRobot 직접 의존성 제거
"""

import numpy as np
from physical_ai_server.inference.zenoh_inference_client import ZenohInferenceClient


class InferenceManager:
    """
    LeRobot Docker 컨테이너와 Zenoh로 통신하여 추론 관리
    """

    SUPPORTED_POLICIES = [
        'tdmpc', 'diffusion', 'act', 'vqbet', 'pi0', 'pi0fast', 'smolvla'
    ]

    def __init__(self, device: str = 'cuda', domain_id: int = 30):
        self.device = device
        self.zenoh_client = ZenohInferenceClient(domain_id=domain_id)
        self.policy_type = None
        self.policy_path = None

    def validate_policy(self, policy_path: str) -> tuple:
        """정책 유효성 검사 (로컬에서 config.json만 확인)"""
        import os
        import json
        
        if not os.path.exists(policy_path):
            return False, f'Policy path {policy_path} does not exist.'

        config_path = os.path.join(policy_path, 'config.json')
        if not os.path.exists(config_path):
            return False, f'config.json not found in {policy_path}.'

        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
            policy_type = config.get('type') or config.get('model_type')
            
            if policy_type not in self.SUPPORTED_POLICIES:
                return False, f'Policy type {policy_type} is not supported.'
            
            self.policy_path = policy_path
            self.policy_type = policy_type
            return True, f'Policy {policy_type} is valid.'
            
        except Exception as e:
            return False, f'Error reading config: {e}'

    def load_policy(self, output_mode: str = "server") -> bool:
        """정책 로드 (Docker에 명령)"""
        if not self.policy_path:
            print('No policy path set. Call validate_policy first.')
            return False
        
        self.zenoh_client.load_model(self.policy_path, output_mode)
        return True

    def clear_policy(self):
        """정책 언로드"""
        self.zenoh_client.unload_model()
        self.policy_type = None
        self.policy_path = None

    def predict(
        self,
        images: dict,
        state: list,
        task_instruction: str = None
    ) -> np.ndarray:
        """추론 수행 (Docker에 요청)"""
        state_array = np.array(state, dtype=np.float32)
        
        # 이미지 dict 변환
        image_arrays = {}
        for key, value in images.items():
            if isinstance(value, np.ndarray):
                image_arrays[key] = value
            else:
                image_arrays[key] = np.array(value)
        
        return self.zenoh_client.predict(
            images=image_arrays,
            state=state_array,
            task_instruction=task_instruction
        )

    def configure_realtime_mode(
        self,
        camera_topics: list,
        joint_state_topic: str,
        action_topic: str = None,
        output_mode: str = "direct"
    ):
        """실시간 모드 설정 (Docker가 직접 토픽 구독/발행)"""
        self.zenoh_client.configure_realtime(
            camera_topics=camera_topics,
            joint_state_topic=joint_state_topic,
            action_topic=action_topic,
            output_mode=output_mode
        )

    @staticmethod
    def get_available_policies() -> list:
        return InferenceManager.SUPPORTED_POLICIES

    @staticmethod
    def get_saved_policies():
        """저장된 정책 목록 (로컬 캐시 스캔)"""
        import os
        import json

        home_dir = os.path.expanduser('~')
        hub_dir = os.path.join(home_dir, '.cache/huggingface/hub')
        
        if not os.path.exists(hub_dir):
            return [], []
        
        models_folders = [d for d in os.listdir(hub_dir) if d.startswith('models--')]
        
        saved_policy_path = []
        saved_policy_type = []

        for model_folder in models_folders:
            model_path = os.path.join(hub_dir, model_folder, 'snapshots')
            if not os.path.exists(model_path):
                continue
                
            for snapshot in os.listdir(model_path):
                snapshot_path = os.path.join(model_path, snapshot)
                pretrained_path = os.path.join(snapshot_path, 'pretrained_model')
                config_path = os.path.join(pretrained_path, 'config.json')
                
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            config = json.load(f)
                        policy_type = config.get('type') or config.get('model_type')
                        if policy_type:
                            saved_policy_path.append(pretrained_path)
                            saved_policy_type.append(policy_type)
                    except:
                        pass

        return saved_policy_path, saved_policy_type
```



## Phase 5: 테스트 및 자동 업데이트

**예상 소요 시간**: 3-4시간

### 5.1 테스트 구조

```
tests/
├── unit/
│   ├── test_zenoh_training_client.py
│   ├── test_zenoh_inference_client.py
│   └── test_training_manager.py
├── integration/
│   ├── test_zenoh_communication.py      # Zenoh 통신 테스트
│   ├── test_docker_lerobot.py           # Docker 컨테이너 테스트
│   └── test_training_integration.py     # Training 통합 테스트
├── e2e/
│   ├── test_training_e2e.py             # 실제 데이터로 학습 E2E
│   └── test_inference_e2e.py            # 실제 모델로 추론 E2E
└── fixtures/
    ├── test_dataset/                    # 테스트 데이터셋
    └── test_model/                      # 테스트 모델
```

### 5.2 LeRobot 업데이트 스크립트

**파일 위치**: `scripts/update_lerobot.sh`

```bash
#!/bin/bash
# LeRobot Submodule 업데이트 스크립트
# 사용법: ./scripts/update_lerobot.sh

set -e

echo "=========================================="
echo "LeRobot Submodule 업데이트"
echo "=========================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
LEROBOT_DIR="$ROOT_DIR/third_party/lerobot"

# 1. 현재 상태 확인
echo ""
echo "[1/6] 현재 상태 확인..."
cd "$LEROBOT_DIR"
PREV_COMMIT=$(git rev-parse HEAD)
echo "현재 커밋: $PREV_COMMIT"

# 2. main 브랜치 최신으로 업데이트
echo ""
echo "[2/6] main 브랜치 업데이트..."
git fetch origin main
git checkout main
git pull origin main

CURR_COMMIT=$(git rev-parse HEAD)
echo "업데이트 후 커밋: $CURR_COMMIT"

if [ "$PREV_COMMIT" == "$CURR_COMMIT" ]; then
    echo "✅ 이미 최신 버전입니다."
    exit 0
fi

# 3. 변경사항 확인
echo ""
echo "[3/6] 변경사항 확인..."
echo "변경된 파일:"
git diff --name-only "$PREV_COMMIT" "$CURR_COMMIT" | head -20

echo ""
echo "주요 API 변경사항:"
git diff "$PREV_COMMIT" "$CURR_COMMIT" --name-only | grep -E "(policies|datasets|configs|scripts)" | head -10

# 4. Docker 이미지 재빌드
echo ""
echo "[4/6] Docker 이미지 재빌드..."
cd "$ROOT_DIR/docker"
docker-compose build lerobot

# 5. 테스트 실행
echo ""
echo "[5/6] 테스트 실행..."
cd "$ROOT_DIR"

# Unit 테스트
echo "Unit 테스트..."
pytest tests/unit/ -v --tb=short || echo "⚠️ Unit 테스트 일부 실패"

# Integration 테스트
echo "Integration 테스트..."
pytest tests/integration/ -v --tb=short || echo "⚠️ Integration 테스트 일부 실패"

# 6. 결과 보고
echo ""
echo "[6/6] 업데이트 완료"
echo "=========================================="
echo "이전 커밋: $PREV_COMMIT"
echo "현재 커밋: $CURR_COMMIT"
echo ""
echo "변경 요약:"
git log --oneline "$PREV_COMMIT".."$CURR_COMMIT" | head -10
echo "=========================================="

# 7. Submodule 커밋 업데이트 안내
echo ""
echo "📌 submodule 변경사항을 커밋하려면:"
echo "   cd $ROOT_DIR"
echo "   git add third_party/lerobot"
echo "   git commit -m 'chore: update lerobot submodule to $CURR_COMMIT'"
```

### 5.3 Training E2E 테스트

**파일 위치**: `tests/e2e/test_training_e2e.py`

```python
#!/usr/bin/env python3
"""
Training E2E 테스트
실제 테스트 데이터로 LeRobot Docker에서 학습이 정상 동작하는지 확인
"""

import os
import time
import pytest
import docker

from physical_ai_server.training.zenoh_training_client import ZenohTrainingClient


@pytest.fixture(scope="module")
def docker_client():
    """Docker 클라이언트"""
    return docker.from_env()


@pytest.fixture(scope="module")
def lerobot_container(docker_client):
    """LeRobot 컨테이너 시작"""
    # 컨테이너가 이미 실행 중인지 확인
    try:
        container = docker_client.containers.get("lerobot_server")
        if container.status != "running":
            container.start()
    except docker.errors.NotFound:
        # 컨테이너 실행
        container = docker_client.containers.run(
            "robotis/lerobot-zenoh:latest",
            name="lerobot_server",
            detach=True,
            network_mode="host",
            volumes={
                "/dev/shm": {"bind": "/dev/shm", "mode": "rw"},
                os.path.expanduser("~/datasets"): {"bind": "/datasets", "mode": "rw"},
                os.path.expanduser("~/outputs"): {"bind": "/outputs", "mode": "rw"},
            },
            runtime="nvidia",
        )
    
    # 컨테이너 준비 대기
    time.sleep(5)
    
    yield container
    
    # 테스트 후 정리 (선택적)
    # container.stop()


@pytest.fixture
def zenoh_client():
    """Zenoh Training 클라이언트"""
    client = ZenohTrainingClient(domain_id=30)
    yield client
    client.close()


class TestTrainingE2E:
    """Training E2E 테스트 클래스"""
    
    def test_training_basic(self, lerobot_container, zenoh_client):
        """기본 학습 테스트 (짧은 스텝)"""
        
        # 학습 시작
        zenoh_client.start_training(
            policy_type="act",
            dataset_path="lerobot/pusht",  # 공개 데이터셋
            output_dir="/outputs/test_basic",
            batch_size=4,
            steps=10,  # 테스트용 짧은 스텝
            save_freq=5,
            eval_freq=0,  # 평가 비활성화
            log_freq=1,
        )
        
        # 학습 완료 대기 (최대 120초)
        for i in range(120):
            status = zenoh_client.get_status()
            if status:
                print(f"Step {i}: {status.status}, step={status.current_step}")
                if status.status == "completed":
                    break
                elif status.status == "error":
                    pytest.fail(f"Training error: {status.error_message}")
            time.sleep(1)
        
        # 결과 검증
        assert status is not None
        assert status.status == "completed"
        assert status.current_step >= 10
    
    def test_training_with_checkpoint(self, lerobot_container, zenoh_client):
        """체크포인트 저장 테스트"""
        
        output_dir = "/outputs/test_checkpoint"
        
        zenoh_client.start_training(
            policy_type="diffusion",
            dataset_path="lerobot/pusht",
            output_dir=output_dir,
            batch_size=4,
            steps=20,
            save_freq=10,
            eval_freq=0,
            log_freq=5,
        )
        
        # 완료 대기
        for i in range(180):
            status = zenoh_client.get_status()
            if status and status.status in ["completed", "error"]:
                break
            time.sleep(1)
        
        assert status.status == "completed"
        
        # 체크포인트 확인
        checkpoint_path = os.path.expanduser(f"~/outputs/test_checkpoint")
        assert os.path.exists(checkpoint_path)
        
        # pretrained_model 폴더 확인
        checkpoints = [d for d in os.listdir(checkpoint_path) if d.startswith("checkpoint")]
        assert len(checkpoints) >= 1
    
    def test_training_stop(self, lerobot_container, zenoh_client):
        """학습 중지 테스트"""
        
        zenoh_client.start_training(
            policy_type="act",
            dataset_path="lerobot/pusht",
            output_dir="/outputs/test_stop",
            batch_size=4,
            steps=1000,  # 긴 학습
            save_freq=100,
            eval_freq=0,
        )
        
        # 잠시 대기 후 중지
        time.sleep(10)
        zenoh_client.stop_training()
        
        # 상태 확인
        time.sleep(2)
        status = zenoh_client.get_status()
        
        assert status is not None
        assert status.status in ["stopped", "completed"]
        assert status.current_step < 1000
```

### 5.4 Inference E2E 테스트

**파일 위치**: `tests/e2e/test_inference_e2e.py`

```python
#!/usr/bin/env python3
"""
Inference E2E 테스트
rosbag2 테스트 데이터와 테스트 모델로 추론 결과 검증
"""

import os
import time
import numpy as np
import pytest

from physical_ai_server.inference.zenoh_inference_client import ZenohInferenceClient


@pytest.fixture
def zenoh_client():
    """Zenoh Inference 클라이언트"""
    client = ZenohInferenceClient(domain_id=30)
    yield client
    client.close()


@pytest.fixture
def test_model_path():
    """테스트 모델 경로"""
    # 환경변수 또는 기본 경로
    return os.environ.get(
        "TEST_MODEL_PATH",
        os.path.expanduser("~/models/test_policy")
    )


@pytest.fixture
def test_rosbag_path():
    """테스트 rosbag 경로"""
    return os.environ.get(
        "TEST_ROSBAG_PATH",
        os.path.expanduser("~/datasets/test_rosbag")
    )


class TestInferenceE2E:
    """Inference E2E 테스트 클래스"""
    
    def test_model_load_unload(self, zenoh_client, test_model_path):
        """모델 로드/언로드 테스트"""
        
        # 모델 로드
        zenoh_client.load_model(test_model_path, output_mode="server")
        time.sleep(2)  # 로드 대기
        
        # 언로드
        zenoh_client.unload_model()
        time.sleep(1)
    
    def test_single_inference(self, zenoh_client, test_model_path):
        """단일 추론 테스트"""
        
        # 모델 로드
        zenoh_client.load_model(test_model_path, output_mode="server")
        time.sleep(2)
        
        # 테스트 입력 생성
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        test_state = np.random.randn(6).astype(np.float32)
        
        # 추론
        action = zenoh_client.predict(
            images={"camera_0": test_image},
            state=test_state,
            task_instruction="pick up the cup"
        )
        
        # 결과 검증
        assert action is not None
        assert isinstance(action, np.ndarray)
        assert len(action.shape) == 1
        assert not np.isnan(action).any()
        
        # 정리
        zenoh_client.unload_model()
    
    def test_inference_with_rosbag(self, zenoh_client, test_model_path, test_rosbag_path):
        """rosbag 데이터로 추론 테스트"""
        
        if not os.path.exists(test_rosbag_path):
            pytest.skip(f"Test rosbag not found: {test_rosbag_path}")
        
        # 모델 로드
        zenoh_client.load_model(test_model_path, output_mode="server")
        time.sleep(2)
        
        # rosbag 데이터 로드 (간략화)
        # TODO: 실제 rosbag 로딩 로직
        test_frames = self._load_rosbag_frames(test_rosbag_path)
        
        results = []
        for frame in test_frames[:10]:  # 처음 10 프레임만
            action = zenoh_client.predict(
                images=frame["images"],
                state=frame["joint_state"],
                task_instruction=frame.get("task", "")
            )
            
            results.append({
                "action": action,
                "timestamp": frame["timestamp"]
            })
        
        # 결과 검증
        assert len(results) == 10
        for result in results:
            assert result["action"] is not None
            assert not np.isnan(result["action"]).any()
        
        # 정리
        zenoh_client.unload_model()
    
    def test_inference_output_modes(self, zenoh_client, test_model_path):
        """출력 모드 테스트 (direct vs server)"""
        
        # Mode 1: Server 모드
        zenoh_client.load_model(test_model_path, output_mode="server")
        time.sleep(2)
        
        # 서비스 호출로 결과 받기
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        test_state = np.random.randn(6).astype(np.float32)
        
        action_server = zenoh_client.predict(
            images={"camera_0": test_image},
            state=test_state
        )
        assert action_server is not None
        
        zenoh_client.unload_model()
        
        # Mode 2: Direct 모드 (실시간 구독)
        # TODO: 실제 ROS2 토픽이 필요하므로 모의 테스트
    
    def _load_rosbag_frames(self, rosbag_path):
        """rosbag에서 프레임 로드 (간략화)"""
        # TODO: 실제 rosbag2 로딩 구현
        # 여기서는 더미 데이터 반환
        frames = []
        for i in range(20):
            frames.append({
                "images": {"camera_0": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)},
                "joint_state": np.random.randn(6).astype(np.float32),
                "timestamp": i * 0.033,  # 30Hz
                "task": "test task"
            })
        return frames
```

<!-- 
💬 코멘트: 테스트 및 업데이트 스크립트에 대한 의견

-->

---

## 작업 요약

| Phase | 작업 | 예상 소요 | 우선순위 | 상태 |
|-------|------|----------|---------|------|
| 1 | LeRobot Submodule 재구성 | 1시간 | ⭐⭐⭐ | 🔲 대기 |
| 2 | LeRobot Docker 컨테이너 구축 | 3-4시간 | ⭐⭐⭐ | 🔲 대기 |
| 3 | Zenoh 통신 레이어 구현 | 4-6시간 | ⭐⭐⭐ | 🔲 대기 |
| 4 | physical_ai_server 의존성 분리 | 4-6시간 | ⭐⭐ | 🔲 대기 |
| 5 | 테스트 및 자동 업데이트 | 3-4시간 | ⭐⭐ | 🔲 대기 |

**총 예상 소요 시간**: 15-21시간

---

## 확인이 필요한 사항

아래 질문들에 대한 답변을 코멘트로 남겨주세요:

### Q1: zenoh_ros2_sdk Shared Memory 설정

현재 SDK에는 SHM 설정이 없습니다. Docker에서 SHM을 사용하려면 SDK를 수정해야 합니다.

- [O] SDK 수정하여 SHM 지원 추가
- [ ] 일단 TCP로 진행하고 나중에 SHM 추가
- [ ] 기타: _______________

<!-- 
💬 코멘트: 

-->

### Q2: 테스트 데이터 및 모델

E2E 테스트를 위한 테스트 데이터셋과 모델이 필요합니다.

- [ ] 이미 준비된 테스트 데이터/모델이 있음 (경로: _______________)
- [O] 공개 데이터셋(lerobot/pusht 등) 사용 --> 코멘트: v3.0을 기준으로된 테스트 데이터가 필요해
- [ ] 새로 만들어야 함
- [ ] 기타: _______________

<!-- 
💬 코멘트: 

-->

### Q3: Inference 출력 모드 우선순위

두 가지 모드 중 먼저 구현할 것:

- [O] Mode 1: Docker가 직접 Action 발행 (로봇에 직접)
- [ ] Mode 2: physical_ai_server로 전달 (Server가 Action 발행)
- [ ] 둘 다 동시에

<!-- 
💬 코멘트:  먼저 구현하고 꼭 나중에 둘다 구현해야해.
-->

### Q4: feature-robotis 브랜치 커스텀 내용

현재 `feature-robotis` 브랜치를 사용 중입니다. main으로 변경 시:

- [ ] 커스텀 수정사항 없음, main으로 변경해도 됨
- [ ] 커스텀 수정사항 있음, 확인 필요 (내용: _______________)
- [ ] feature-robotis 유지하고 싶음

<!-- 
💬 코멘트: 
  feautre-1.0.0에서 분기를 따서 작업해줘. 그리고 최종 머지할 때도 feature-1.0.0으로 머지해야해.
  아니면 
-->

### Q5: Docker 실행 환경

LeRobot Docker 실행 위치:

- [O] 같은 PC에서만 실행
- [O] 원격 서버에서도 실행 가능해야 함
- [O] 클라우드 환경 지원 필요

<!-- 
💬 코멘트: 
우선 같은 PC를 기준으로 작업하자.
-->

### Q6: 기타 요구사항

<!-- 
💬 코멘트: 추가 요구사항이나 수정사항
 Lerobot을 최신으로 pull하고 작업해야해.
-->

---

## 코멘트 섹션

아래에 전체적인 피드백이나 추가 의견을 남겨주세요:

<!-- 
💬 전체 코멘트:
전체적으로 좋아. 대신 순차적으로 잘 구현해보자. 너무 큰게크게 바꾸지말고 유닛테스트를 철저히 하면서 코드를 작성해줘.



-->

---

**문서 버전**: 1.0  
**마지막 업데이트**: 2025-01-14
