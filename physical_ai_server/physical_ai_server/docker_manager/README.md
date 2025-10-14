# Docker Manager for Physical AI Tools

Physical AI Tools의 Docker 컨테이너 생명주기를 관리하는 모듈입니다. LeRobot, GR00T N1.5, Pi0 등 다양한 Physical AI 프레임워크를 Docker 컨테이너로 실행하고 관리합니다.

## 📋 주요 기능

### 1. Image Management
- Docker 이미지 빌드
- 이미지 Pull/Push
- 이미지 목록 조회 및 삭제

### 2. Container Lifecycle
- 컨테이너 생성/시작/중지/재시작/삭제
- 컨테이너 상태 모니터링
- 여러 프레임워크 동시 실행

### 3. Process Management
- 컨테이너 내부 명령 실행
- 프레임워크별 프로세스 시작/관리
- 프로세스 모니터링

### 4. Network & Communication
- 컨테이너 간 네트워크 구성
- HTTP/gRPC/ZMQ 통신 설정
- IP 주소 및 포트 관리

### 5. Resource Management
- GPU 할당 및 관리
- 메모리 제한 설정
- 리소스 사용량 모니터링

## 🏗️ 아키텍처

### 전체 구조
```
┌─────────────────────────────────────────────────────────┐
│                  Physical AI Server                      │
│              (ROS2 Node - Orchestrator)                  │
│  ┌────────────────────────────────────────────────────┐ │
│  │            Docker Manager                          │ │
│  │  - Image Management                                │ │
│  │  - Container Lifecycle                             │ │
│  │  - Resource Allocation                             │ │
│  │  - Communication Setup                             │ │
│  └────────────────────────────────────────────────────┘ │
└──────────────┬───────────────┬────────────────┬─────────┘
               │               │                │
         ┌─────▼──────┐  ┌────▼─────┐   ┌─────▼──────┐
         │  LeRobot   │  │  GR00T   │   │    Pi0     │
         │ Container  │  │Container │   │ Container  │
         │            │  │          │   │            │
         │ API Server │  │API Server│   │ API Server │
         │  Port:8000 │  │Port:8001 │   │ Port:8002  │
         └────────────┘  └──────────┘   └────────────┘
               │               │                │
               └───────────────┴────────────────┘
                    Docker Network Bridge
                  (physical_ai_network)
```

### 통신 전략

#### ❌ 문제: ROS2 DDS 버전 불일치
```
LeRobot Container (Ubuntu 22.04 + ROS2 Humble)
     ↕️ DDS 통신 불가능 (버전 불일치)
GR00T Container (Ubuntu 20.04 + ROS2 Foxy)
```

#### ✅ 해결: ZMQ 기반 통합 통신
```
[Robot/External Device]
         ↕ ROS2 DDS (Same version)
[Physical AI Server]
         ↕ ZMQ (Version independent, High performance)
[Framework Containers]
  - LeRobot Container (ZMQ Server)
  - GR00T Container (ZMQ Server)
  - Pi0 Container (ZMQ Server)
         ↕ Shared Docker Volumes (For large data: datasets, models)
[Host File System]
```

**ZMQ 통신 아키텍처:**
```
Physical AI Server
├── ZMQ Client Manager
│   ├── LeRobot Client (tcp://lerobot_container:5555)
│   ├── GR00T Client (tcp://groot_container:5556)
│   └── Pi0 Client (tcp://pi0_container:5557)
└── ROS2 Interface (외부 로봇과 통신)

각 Framework Container
├── ZMQ Server (REQ-REP or PUB-SUB pattern)
├── Framework Core (LeRobot/GR00T/Pi0)
└── Shared Volume Access
```

### 왜 ZMQ를 선택했는가?

| 이유 | 설명 |
|------|------|
| **🚀 고성능** | 매우 빠른 메시지 전송 (추론에 중요) |
| **🔄 일관성** | 이미 Inference에서 ZMQ 사용 중 |
| **🌐 버전 독립** | ROS2 버전과 무관하게 동작 |
| **🎯 간단함** | HTTP보다 가볍고, gRPC보다 설정 간단 |
| **📦 다양한 패턴** | REQ-REP, PUB-SUB, PUSH-PULL 등 |
| **🐍 Python 친화적** | pyzmq로 쉬운 통합 |

### ZMQ vs 다른 프로토콜

| 프로토콜 | 사용처 | 장점 | 단점 |
|---------|------|------|------|
| **ZMQ** | **모든 컨테이너 통신** ✅ | 매우 빠름, 비동기, 다양한 패턴 | 메시지 형식 직접 정의 |
| **Shared Volume** | 대용량 데이터 (데이터셋, 모델) | 빠른 I/O, 파일 공유 | 동기화 필요 |
| **ROS2 DDS** | Physical AI Server ↔ 로봇 | Native ROS2, 표준 | 버전 의존성 |
| ~~HTTP REST~~ | ❌ 사용 안함 | 간단, 디버깅 용이 | 느림 |
| ~~gRPC~~ | ❌ 사용 안함 | 타입 안정성 | 복잡한 설정 |

## 🚀 빠른 시작

### 설치

```bash
# Docker SDK와 ZMQ 설치
pip install docker>=7.0.0 pyzmq>=25.0.0

# 또는 ROS2 패키지 빌드 시 자동 설치
cd ~/ros2_ws
colcon build --packages-select physical_ai_server
```

### 기본 사용법

#### Physical AI Server 측 (ZMQ Client)

```python
from physical_ai_server.docker_manager import (
    DockerManager,
    FrameworkType,
    ZMQClientPool,
)

# Docker Manager 초기화
manager = DockerManager()

# LeRobot 컨테이너 생성 및 시작
container_id = manager.create_container(
    framework=FrameworkType.LEROBOT,
    gpu_ids=[0],
    memory_limit='8g',
    api_port=5555,  # ZMQ 포트
)
manager.start_container(container_id)

# ZMQ 서버 시작 (컨테이너 내부)
process_config = {
    'mode': 'inference',
    'port': 5555,
}
manager.start_framework_process(container_id, FrameworkType.LEROBOT, process_config)

# ZMQ 클라이언트 풀 생성
client_pool = ZMQClientPool()

# 컨테이너 IP 가져오기 및 클라이언트 추가
container_ip = manager.get_container_ip(container_id)
client_pool.add_client('lerobot', f'tcp://{container_ip}:5555')

# Inference 요청
observation = {
    'image': [...],  # 이미지 데이터
    'robot_state': [...],  # 로봇 상태
}
response = client_pool.inference('lerobot', observation)
print(f"Action: {response['result']['action']}")
```

#### Framework Container 측 (ZMQ Server)

```python
# LeRobot 컨테이너 내부에서 실행
from physical_ai_server.docker_manager import ZMQServer, MessageType

# Inference 핸들러
def handle_inference(data):
    observation = data['observation']
    # LeRobot으로 추론 실행
    action = model.predict(observation)
    return {'action': action}

# ZMQ 서버 시작
server = ZMQServer('tcp://*:5555')
server.register_handler(MessageType.INFERENCE, handle_inference)
server.run()  # 블로킹 - 계속 실행됨
```

## 📚 상세 사용 예시

### 1. 여러 프레임워크 동시 실행

```python
from physical_ai_server.docker_manager import (
    DockerManager,
    FrameworkType,
    CommunicationProtocol,
)

manager = DockerManager()

# 여러 프레임워크 컨테이너 생성
frameworks = [
    (FrameworkType.LEROBOT, 8000),
    (FrameworkType.GROOT_N15, 8001),
    (FrameworkType.PI0, 8002),
]

containers = {}
for framework, port in frameworks:
    container_id = manager.create_container(
        framework=framework,
        api_port=port,
        gpu_ids=None,  # 모든 GPU 공유
    )
    manager.start_container(container_id)
    containers[framework] = container_id

# ZMQ 통신 설정 (모든 컨테이너는 ZMQ Server로 동작)
comm_config = manager.setup_communication(
    source_framework=FrameworkType.LEROBOT,
    target_framework=FrameworkType.GROOT_N15,
    protocol=CommunicationProtocol.ZMQ,
)
print(f"ZMQ Endpoint: {comm_config['endpoint']}")  # tcp://groot_ip:8001
```

### 2. 프로세스 관리

```python
# LeRobot 추론 서버 시작
process_config = {
    'mode': 'inference',
    'model_path': '/workspace/shared_data/models/lerobot_policy',
    'port': 8000,
}

process_info = manager.start_framework_process(
    container_id=container_id,
    framework=FrameworkType.LEROBOT,
    config=process_config,
)

print(f"Process PID: {process_info['pid']}")
print(f"Log file: {process_info['log_file']}")

# 프로세스 모니터링
status = manager.monitor_process(container_id, process_info['pid'])
print(f"CPU: {status['cpu_percent']}%")
print(f"Elapsed: {status['elapsed_time']}")
```

### 3. 컨테이너 내부 명령 실행

```python
# 단일 명령 실행
output = manager.exec_command(
    container_id,
    "python -c 'import torch; print(torch.cuda.is_available())'",
)
print(f"CUDA Available: {output}")

# 스트리밍 출력
for line in manager.exec_command(
    container_id,
    "python train.py",
    stream=True,
):
    print(line.decode('utf-8'), end='')
```

### 4. 리소스 관리

```python
# 메모리 제한 업데이트
manager.set_memory_limits(
    container_id,
    memory_limit='16g',  # 16GB로 증가
)

# 실시간 리소스 모니터링
import time
for i in range(10):
    resources = manager.monitor_resources(container_id)
    print(f"[{i}] CPU: {resources['cpu_percent']:5.1f}% | "
          f"Memory: {resources['memory_percent']:5.1f}% | "
          f"Network RX: {resources['network_rx_bytes']/(1024**2):.2f}MB")
    time.sleep(1)
```

## 🔧 Configuration

### 환경 변수

```bash
# Docker 데몬 접속 (기본: unix:///var/run/docker.sock)
export DOCKER_HOST=unix:///var/run/docker.sock

# GPU 사용 설정
export NVIDIA_VISIBLE_DEVICES=0,1  # GPU 0, 1 사용
```

### Docker 네트워크 설정

```python
manager = DockerManager(
    network_name="custom_network",
    shared_volume_path="/custom/path",
    base_image_registry="myregistry",
)
```

### 볼륨 매핑

Docker Manager는 자동으로 다음 볼륨을 생성합니다:

```
Host                                    → Container
─────────────────────────────────────────────────────────────
~/.cache/physical_ai_tools/shared_data  → /workspace/shared_data
~/.cache/physical_ai_tools/lerobot      → /workspace/lerobot_data
~/.cache/physical_ai_tools/groot_n15    → /workspace/groot_n15_data
~/.cache/huggingface                    → /root/.cache/huggingface
```

## 🐛 문제 해결

### Docker 데몬 연결 실패

```bash
# Docker 서비스 시작
sudo systemctl start docker

# 사용자를 docker 그룹에 추가
sudo usermod -aG docker $USER
newgrp docker
```

### NVIDIA Runtime 오류

```bash
# NVIDIA Container Toolkit 설치
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### 컨테이너 네트워크 문제

```bash
# 네트워크 재생성
docker network rm physical_ai_network
docker network create physical_ai_network

# 네트워크 확인
docker network inspect physical_ai_network
```

### 포트 충돌

```python
# 사용 가능한 포트 자동 할당
import socket

def find_free_port():
    with socket.socket() as s:
        s.bind(('', 0))
        return s.getsockname()[1]

port = find_free_port()
container_id = manager.create_container(
    framework=FrameworkType.LEROBOT,
    api_port=port,
)
```

## 📖 API Reference

### DockerManager 클래스

#### 초기화

```python
DockerManager(
    network_name: str = "physical_ai_network",
    shared_volume_path: Optional[Path] = None,
    base_image_registry: str = "robotis",
    logger: Optional[logging.Logger] = None,
)
```

#### Image Management

- `build_image(framework, dockerfile_path, platform=None, build_args=None, tag=None)` → Image
- `pull_image(framework, tag="latest")` → Image
- `list_images(framework=None)` → List[Dict]
- `remove_image(framework, tag="latest", force=False)` → None

#### Container Lifecycle

- `create_container(framework, config=None, gpu_ids=None, memory_limit=None, api_port=None)` → str
- `start_container(container_id)` → None
- `stop_container(container_id, timeout=10)` → None
- `restart_container(container_id, timeout=10)` → None
- `remove_container(container_id, force=False, volumes=False)` → None
- `get_container_status(container_id)` → ContainerStatus
- `list_containers(all_containers=True)` → List[Dict]

#### Process Management

- `exec_command(container_id, command, workdir=None, environment=None, stream=False)` → str
- `start_framework_process(container_id, framework, config)` → Dict
- `monitor_process(container_id, process_id)` → Dict

#### Network & Communication

- `connect_containers(container_ids, network=None)` → None
- `get_container_ip(container_id)` → Optional[str]
- `setup_communication(source_framework, target_framework, protocol)` → Dict

#### Resource Management

- `allocate_gpu(container_id, gpu_ids)` → None (Not implemented - requires recreation)
- `set_memory_limits(container_id, memory_limit, memory_swap=None)` → None
- `monitor_resources(container_id)` → Dict

### Enums

#### FrameworkType
- `LEROBOT`: LeRobot 프레임워크
- `GROOT_N15`: GR00T N1.5 프레임워크
- `PI0`: Pi0 프레임워크
- `ISAAC_SIM`: Isaac Sim
- `ISAAC_LAB`: Isaac Lab

#### ContainerStatus
- `NOT_EXIST`, `CREATED`, `RUNNING`, `PAUSED`, `RESTARTING`, `REMOVING`, `EXITED`, `DEAD`

#### CommunicationProtocol
- `ZMQ`: ZeroMQ (기본 통신 프로토콜) ✅
- `SHARED_VOLUME`: 공유 볼륨 (대용량 데이터용)

## 🧪 테스트

```bash
# ZMQ 통신 예제 실행
cd ~/ros2_ws/src/physical_ai_tools/physical_ai_server
python -m physical_ai_server.docker_manager.zmq_examples

# Docker Manager 예제 실행
python -m physical_ai_server.docker_manager.examples
```

### ZMQ 통신 테스트

#### 1. 터미널 1: ZMQ 서버 시작
```bash
python -c "
from physical_ai_server.docker_manager import ZMQServer, MessageType
import time

def handle_inference(data):
    print(f'Received: {data}')
    return {'action': [0.1, 0.2, 0.3]}

server = ZMQServer('tcp://*:5555')
server.register_handler(MessageType.INFERENCE, handle_inference)
server.run()
"
```

#### 2. 터미널 2: ZMQ 클라이언트 테스트
```bash
python -c "
from physical_ai_server.docker_manager import ZMQClient

client = ZMQClient('tcp://localhost:5555')
response = client.inference({'image': [1,2,3]})
print(f'Response: {response}')
client.close()
"
```

## 🔄 Physical AI Server 통합

```python
# physical_ai_server.py에서 사용

from physical_ai_server.docker_manager import (
    DockerManager,
    FrameworkType,
    ZMQClientPool,
)

class PhysicalAIServer(Node):
    def __init__(self):
        super().__init__('physical_ai_server')
        
        # Docker Manager 초기화
        self.docker_manager = DockerManager(
            logger=self.get_logger(),
        )
        
        # ZMQ Client Pool 초기화
        self.zmq_clients = ZMQClientPool(
            logger=self.get_logger(),
        )
        
        # 활성 컨테이너 관리
        self.active_containers = {}
    
    def start_framework(self, framework_name: str):
        """사용자 요청 시 프레임워크 컨테이너 시작"""
        framework = FrameworkType(framework_name)
        
        # 1. 컨테이너 생성 및 시작
        container_id = self.docker_manager.create_container(
            framework=framework,
            gpu_ids=[0],
            api_port=5555,  # ZMQ 포트
        )
        self.docker_manager.start_container(container_id)
        
        # 2. ZMQ 서버 프로세스 시작
        config = {
            'mode': 'inference',
            'port': 5555,
        }
        process_info = self.docker_manager.start_framework_process(
            container_id, framework, config
        )
        
        # 3. ZMQ 클라이언트 추가
        container_ip = self.docker_manager.get_container_ip(container_id)
        endpoint = f'tcp://{container_ip}:5555'
        self.zmq_clients.add_client(framework_name, endpoint)
        
        # 4. 상태 저장
        self.active_containers[framework_name] = {
            'container_id': container_id,
            'process_info': process_info,
            'endpoint': endpoint,
        }
        
        self.get_logger().info(f"Started {framework_name}: {endpoint}")
    
    def run_inference(self, framework_name: str, observation: dict):
        """추론 실행"""
        response = self.zmq_clients.inference(framework_name, observation)
        
        if response and response.get('status') == 'success':
            return response['result']['action']
        else:
            self.get_logger().error(f"Inference failed: {response}")
            return None
    
    def stop_framework(self, framework_name: str):
        """프레임워크 컨테이너 중지"""
        if framework_name in self.active_containers:
            container_id = self.active_containers[framework_name]['container_id']
            
            # ZMQ 클라이언트 제거
            self.zmq_clients.remove_client(framework_name)
            
            # 컨테이너 중지 및 제거
            self.docker_manager.stop_container(container_id)
            self.docker_manager.remove_container(container_id)
            
            del self.active_containers[framework_name]
            self.get_logger().info(f"Stopped {framework_name}")
```

## 📝 License

Apache 2.0

## 👥 Authors

- Dongyun Kim (kdy@robotis.com)
- Physical AI Tools Team

## 🤝 Contributing

Issues와 Pull Requests를 환영합니다!

## 📞 Support

문제가 있으시면 GitHub Issues에 등록해주세요.
