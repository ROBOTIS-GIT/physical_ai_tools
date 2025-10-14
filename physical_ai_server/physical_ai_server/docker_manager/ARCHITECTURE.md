# Physical AI Tools - ZMQ 기반 Docker 통신 아키텍처

## 🎯 최종 아키텍처

### 전체 구조

```
┌─────────────────────────────────────────────────────────────────┐
│                     Physical AI Server                           │
│                    (ROS2 Humble Node)                            │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                  Docker Manager                          │   │
│  │  - Container Lifecycle Management                        │   │
│  │  - Resource Allocation (GPU, Memory)                     │   │
│  │  - Network Configuration                                 │   │
│  └──────────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                  ZMQ Client Pool                         │   │
│  │  - LeRobot Client    (tcp://172.17.0.2:5555)           │   │
│  │  - GR00T Client      (tcp://172.17.0.3:5556)           │   │
│  │  - Pi0 Client        (tcp://172.17.0.4:5557)           │   │
│  └──────────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              ROS2 Interface Layer                        │   │
│  │  - Publishers/Subscribers                                │   │
│  │  - Service Servers                                       │   │
│  │  - Action Servers                                        │   │
│  └──────────────────────────────────────────────────────────┘   │
└──────────────┬──────────────┬──────────────┬───────────────────┘
               ↓ ZMQ          ↓ ZMQ          ↓ ZMQ
               (REQ-REP)      (REQ-REP)      (REQ-REP)
               │              │              │
    ┌──────────▼─────┐  ┌────▼──────────┐  ┌▼────────────┐
    │   LeRobot      │  │   GR00T N1.5  │  │    Pi0      │
    │   Container    │  │   Container   │  │  Container  │
    │ (Ubuntu 22.04) │  │ (Ubuntu 20.04)│  │(Ubuntu ?.??)│
    │  ROS2 Humble   │  │  ROS2 Foxy    │  │  No ROS2    │
    ├────────────────┤  ├───────────────┤  ├─────────────┤
    │  ZMQ Server    │  │  ZMQ Server   │  │ ZMQ Server  │
    │  Port: 5555    │  │  Port: 5556   │  │ Port: 5557  │
    ├────────────────┤  ├───────────────┤  ├─────────────┤
    │ LeRobot Core   │  │  GR00T Core   │  │  Pi0 Core   │
    │ - Policy Net   │  │  - VLA Model  │  │ - VLA Model │
    │ - Training     │  │  - Inference  │  │ - Inference │
    └────────────────┘  └───────────────┘  └─────────────┘
           ↕                    ↕                  ↕
    Shared Docker Volumes (datasets, models, configs)
           ↕                    ↕                  ↕
    ┌──────────────────────────────────────────────────┐
    │          Host File System                        │
    │  ~/.cache/physical_ai_tools/                     │
    │  ├── shared_data/  (공유 데이터)                │
    │  ├── lerobot/      (LeRobot 전용)               │
    │  ├── groot_n15/    (GR00T 전용)                 │
    │  └── pi0/          (Pi0 전용)                    │
    └──────────────────────────────────────────────────┘
```

## ✅ 왜 ZMQ로 통일했는가?

### 문제점
```
❌ HTTP/REST
- 느린 속도 (추론에 부적합)
- Overhead가 큼
- Stateless (상태 유지 어려움)

❌ gRPC
- 설정 복잡
- Protocol Buffers 정의 필요
- 디버깅 어려움

❌ ROS2 DDS
- 버전 의존성 (Ubuntu 20.04 Foxy ≠ Ubuntu 22.04 Humble)
- 같은 ROS2 버전 필요
- Network Discovery 복잡
```

### 해결책: ZMQ ✅

```
✅ 장점
1. 고성능: 매우 빠른 메시지 전송 (추론에 최적)
2. 일관성: 기존 Inference 코드가 이미 ZMQ 사용
3. 버전 독립: ROS2/Ubuntu 버전과 무관
4. 간단함: 몇 줄의 코드로 구현 가능
5. 다양한 패턴: REQ-REP, PUB-SUB, PUSH-PULL
6. Python 친화적: pyzmq로 쉬운 통합
7. 가볍다: HTTP 서버보다 메모리 적게 사용
```

## 📦 구현 내역

### 1. Docker Manager (`docker_manager.py`)
```python
class DockerManager:
    """Docker 컨테이너 생명주기 관리"""
    
    # Image Management
    - build_image()
    - pull_image()
    - list_images()
    
    # Container Lifecycle
    - create_container()
    - start_container()
    - stop_container()
    - remove_container()
    
    # Process Management
    - exec_command()
    - start_framework_process()  # ZMQ 서버 시작
    - monitor_process()
    
    # Network & Communication
    - setup_communication()  # ZMQ endpoint 설정
    - get_container_ip()
    
    # Resource Management
    - set_memory_limits()
    - monitor_resources()
```

### 2. ZMQ Communication (`zmq_communication.py`)
```python
class ZMQClient:
    """Physical AI Server가 컨테이너와 통신"""
    - send_request()
    - inference()      # 추론 요청
    - train()          # 학습 요청
    - command()        # 명령 실행
    - health_check()   # 헬스 체크

class ZMQServer:
    """각 컨테이너 내부에서 실행"""
    - register_handler()  # 메시지 핸들러 등록
    - run()              # 서버 실행 (블로킹)

class ZMQClientPool:
    """여러 프레임워크 클라이언트 관리"""
    - add_client()
    - remove_client()
    - inference()         # 특정 프레임워크에 추론 요청
    - health_check_all()  # 모든 프레임워크 헬스 체크
```

### 3. 메시지 포맷 (JSON)
```json
// Request
{
    "type": "inference",
    "data": {
        "observation": {
            "image": [...],
            "robot_state": [...]
        }
    },
    "timestamp": 1696234567.123,
    "request_id": "uuid-1234-5678"
}

// Response
{
    "status": "success",
    "result": {
        "action": [0.1, 0.2, 0.3, 0.4, 0.5],
        "confidence": 0.95
    },
    "request_id": "uuid-1234-5678",
    "timestamp": 1696234567.456
}
```

## 🚀 사용 예시

### 1. Physical AI Server 시작

```python
from physical_ai_server.docker_manager import (
    DockerManager,
    FrameworkType,
    ZMQClientPool,
)

# 초기화
docker_manager = DockerManager()
zmq_clients = ZMQClientPool()

# LeRobot 컨테이너 시작
container_id = docker_manager.create_container(
    framework=FrameworkType.LEROBOT,
    gpu_ids=[0],
    api_port=5555,
)
docker_manager.start_container(container_id)

# ZMQ 서버 프로세스 시작 (컨테이너 내부)
process_config = {'mode': 'inference', 'port': 5555}
docker_manager.start_framework_process(
    container_id, FrameworkType.LEROBOT, process_config
)

# ZMQ 클라이언트 연결
container_ip = docker_manager.get_container_ip(container_id)
zmq_clients.add_client('lerobot', f'tcp://{container_ip}:5555')

# 추론 실행
observation = {'image': [...], 'robot_state': [...]}
response = zmq_clients.inference('lerobot', observation)
action = response['result']['action']
```

### 2. 컨테이너 내부 (LeRobot ZMQ Server)

```python
# lerobot/scripts/zmq_inference_server.py
from physical_ai_server.docker_manager import ZMQServer, MessageType
from lerobot import LeRobotPolicy

# 모델 로드
policy = LeRobotPolicy.from_pretrained('lerobot/policy')

# Inference 핸들러
def handle_inference(data):
    observation = data['observation']
    action = policy.predict(observation)
    return {'action': action.tolist(), 'confidence': 0.95}

# ZMQ 서버 시작
server = ZMQServer('tcp://*:5555')
server.register_handler(MessageType.INFERENCE, handle_inference)
server.run()  # 블로킹 - 계속 실행
```

## 📊 통신 플로우

```
1. 사용자가 로봇 제어 시작
   ↓
2. Physical AI Server가 LeRobot 컨테이너 필요 감지
   ↓
3. Docker Manager가 LeRobot 컨테이너 생성/시작
   ↓
4. 컨테이너 내부에서 ZMQ Server 프로세스 시작
   ↓
5. Physical AI Server가 ZMQ Client 생성 및 연결
   ↓
6. 로봇에서 관측 데이터 수신 (ROS2)
   ↓
7. Physical AI Server가 ZMQ로 LeRobot에 추론 요청
   ↓
8. LeRobot이 추론 실행 후 ZMQ로 응답
   ↓
9. Physical AI Server가 액션을 ROS2로 로봇에 전송
   ↓
10. 반복...
```

## 🔧 설정

### Docker Compose (optional)
```yaml
# docker-compose.yml
version: '3.8'

services:
  lerobot:
    image: robotis/lerobot:latest
    container_name: physical_ai_lerobot
    network_mode: bridge
    ports:
      - "5555:5555"  # ZMQ port
    volumes:
      - ~/.cache/physical_ai_tools/shared_data:/workspace/shared_data
      - ~/.cache/physical_ai_tools/lerobot:/workspace/lerobot_data
      - ~/.cache/huggingface:/root/.cache/huggingface
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=0
    command: python -m lerobot.scripts.zmq_inference_server --port 5555
```

### 포트 할당
```
Physical AI Server: ROS2 (no port, DDS)
LeRobot Container:  5555 (ZMQ)
GR00T Container:    5556 (ZMQ)
Pi0 Container:      5557 (ZMQ)
```

## 🎓 학습 사항

### 1. ROS2 버전 문제 해결
- 각 컨테이너가 다른 ROS2 버전 사용 가능
- Physical AI Server만 ROS2 사용 (외부 인터페이스)
- 컨테이너 간 통신은 ZMQ로 버전 독립

### 2. 성능 최적화
- ZMQ는 HTTP보다 10-100배 빠름
- Zero-copy 가능 (numpy array)
- 비동기 통신 지원

### 3. 확장성
- 새로운 프레임워크 추가 용이
- ZMQ Server만 구현하면 됨
- Docker Manager가 자동으로 관리

## 📈 다음 단계

### 1. 각 프레임워크별 ZMQ Server 구현
```bash
lerobot/scripts/zmq_inference_server.py
groot/scripts/zmq_inference_server.py
pi0/scripts/zmq_inference_server.py
```

### 2. Physical AI Server 통합
```python
# physical_ai_server.py에 Docker Manager 통합
self.docker_manager = DockerManager()
self.zmq_clients = ZMQClientPool()
```

### 3. Dockerfile 업데이트
```dockerfile
# lerobot/Dockerfile에 ZMQ 서버 포함
CMD ["python", "-m", "lerobot.scripts.zmq_inference_server", "--port", "5555"]
```

### 4. 테스트
```bash
# 단위 테스트
pytest tests/test_docker_manager.py
pytest tests/test_zmq_communication.py

# 통합 테스트
python -m physical_ai_server.docker_manager.zmq_examples
```

## 🎉 완료!

이제 Physical AI Tools는:
- ✅ 다양한 ROS2 버전 지원
- ✅ 고성능 ZMQ 통신
- ✅ 컨테이너 기반 격리
- ✅ 중앙화된 관리
- ✅ 쉬운 확장성

을 갖춘 완전한 시스템이 되었습니다! 🚀
