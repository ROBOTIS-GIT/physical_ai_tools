# ROSbag to LeRobot Converter Test Guide

ROSbag(MCAP + MP4) 데이터를 LeRobot 데이터셋으로 변환하는 기능을 테스트하기 위한 가이드입니다.

## Prerequisites

- Docker 컨테이너 실행 중: `physical_ai_server`
- HuggingFace 테스트 데이터셋: `Dongkkka/physical_ai_tools_test_rosbag`

## Test Workflow

### 1. 테스트 데이터 다운로드

HuggingFace에서 테스트용 ROSbag 데이터를 다운로드합니다:

```bash
docker exec physical_ai_server python3 -c "
from huggingface_hub import snapshot_download
snapshot_download(
    repo_id='Dongkkka/physical_ai_tools_test_rosbag',
    repo_type='dataset',
    local_dir='/workspace/test_rosbag'
)
"
```

### 2. 로봇 동작 시뮬레이션 (rosbag play)

새 터미널에서 rosbag play를 실행하여 로봇이 동작하는 것처럼 토픽을 발행합니다:

```bash
# Terminal 1: rosbag play (loop mode)
docker exec -it physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
ros2 bag play /workspace/test_rosbag/0 --loop --rate 1.0
'
```

### 3. physical_ai_server 및 rosbag_recorder 실행

```bash
# Terminal 2: physical_ai_server
docker exec -it physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch physical_ai_server physical_ai_server.launch.py
'

# Terminal 3: rosbag_recorder
docker exec -it physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run rosbag_recorder service_bag_recorder
'
```

### 4. 로봇 타입 설정

UI의 "Select Robot Type" 드롭다운에서 로봇을 선택하는 것과 동일합니다:

```bash
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 service call /set_robot_type physical_ai_interfaces/srv/SetRobotType "{robot_type: ffw_bg2_rev4}"
'
```

### 5. rosbag_recorder로 MCAP + MP4 녹화

#### 5.1 PREPARE - 녹화할 토픽 및 로봇 타입 설정

```bash
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand "{
  command: 0,
  topics: [
    \"/camera_left/camera_left/color/image_rect_raw/compressed\",
    \"/camera_right/camera_right/color/image_rect_raw/compressed\",
    \"/zed/zed_node/left/image_rect_color/compressed\",
    \"/joint_states\"
  ],
  robot_type: \"ffw_bg2_rev4\"
}"
'
```

> **Note**: `robot_type`을 설정하면 로봇 설정 파일에서 카메라 매핑 정보를 로드하여 `robot_config.yaml`에 저장합니다.

#### 5.2 START - 녹화 시작

```bash
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand "{
  command: 1,
  uri: \"/workspace/test_recording\"
}"
'
```

#### 5.3 12초 대기 후 STOP

```bash
sleep 12

docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand "{command: 2}"
'
```

### 6. 녹화 결과 확인

```bash
# 디렉토리 구조 확인
docker exec physical_ai_server ls -la /workspace/test_recording/

# robot_config.yaml 내용 확인 (camera_mapping 포함)
docker exec physical_ai_server cat /workspace/test_recording/robot_config.yaml
```

예상 출력:
```yaml
robot_type: ffw_bg2_rev4
camera_mapping:
  /zed/zed_node/left/image_rect_color/compressed: cam_head
  /camera_left/camera_left/color/image_rect_raw/compressed: cam_wrist_left
  /camera_right/camera_right/color/image_rect_raw/compressed: cam_wrist_right
```

### 7. LeRobot 데이터셋으로 변환

```bash
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/ros2_ws/src/physical_ai_tools/physical_ai_server/scripts
python convert_rosbag_to_lerobot.py \
  --input /workspace/test_recording \
  --output /workspace/lerobot_test_output \
  --repo-id test/test_dataset \
  --fps 30 \
  --verbose
'
```

### 8. 변환 결과 검증

```bash
# 디렉토리 구조 확인
docker exec physical_ai_server find /workspace/lerobot_test_output -type f | sort
```

**올바른 결과** (카메라 이름이 매핑된 경우):
```
/workspace/lerobot_test_output/data/chunk-000/episode_000000.parquet
/workspace/lerobot_test_output/meta/info.json
/workspace/lerobot_test_output/meta/episodes.jsonl
/workspace/lerobot_test_output/meta/tasks.jsonl
/workspace/lerobot_test_output/videos/chunk-000/observation.images.cam_head/episode_000000.mp4
/workspace/lerobot_test_output/videos/chunk-000/observation.images.cam_wrist_left/episode_000000.mp4
/workspace/lerobot_test_output/videos/chunk-000/observation.images.cam_wrist_right/episode_000000.mp4
```

**잘못된 결과** (카메라 매핑이 없는 경우):
```
/workspace/lerobot_test_output/videos/chunk-000/observation.images.zed_zed_node_left_image_rect_color/episode_000000.mp4
/workspace/lerobot_test_output/videos/chunk-000/observation.images.left/episode_000000.mp4
/workspace/lerobot_test_output/videos/chunk-000/observation.images.right/episode_000000.mp4
```

## Test Checklist

- [ ] 테스트 데이터 다운로드 완료
- [ ] rosbag play 실행 중 (토픽 발행 확인)
- [ ] physical_ai_server 실행 중
- [ ] rosbag_recorder 실행 중
- [ ] SetRobotType 서비스 호출 성공
- [ ] PREPARE 서비스 호출 성공 (robot_type 포함)
- [ ] START 서비스 호출 성공
- [ ] 12초 녹화 후 STOP 성공
- [ ] robot_config.yaml에 camera_mapping 포함 확인
- [ ] LeRobot 변환 성공
- [ ] 변환된 데이터셋의 카메라 이름이 올바른지 확인:
  - `observation.images.cam_head`
  - `observation.images.cam_wrist_left`
  - `observation.images.cam_wrist_right`

## Troubleshooting

### robot_config.yaml에 camera_mapping이 없는 경우

1. rosbag_recorder가 최신 버전으로 빌드되었는지 확인:
   ```bash
   docker exec physical_ai_server bash -c '
   source /opt/ros/jazzy/setup.bash
   cd /root/ros2_ws
   colcon build --packages-select rosbag_recorder
   '
   ```

2. PREPARE 호출 시 robot_type이 올바르게 전달되었는지 확인

3. 로봇 설정 파일이 존재하는지 확인:
   ```bash
   docker exec physical_ai_server ls /root/ros2_ws/install/physical_ai_server/share/physical_ai_server/config/
   ```

### 변환 후에도 카메라 이름이 잘못된 경우

1. 변환기 로그에서 "Loaded camera mapping" 메시지 확인
2. robot_config.yaml 파일이 rosbag 디렉토리에 있는지 확인
3. camera_mapping의 토픽 이름이 정확한지 확인

## Service Reference

| Service | Type | Description |
|---------|------|-------------|
| `/set_robot_type` | `physical_ai_interfaces/srv/SetRobotType` | 로봇 타입 설정 |
| `/rosbag_recorder/send_command` | `rosbag_recorder/srv/SendCommand` | ROSbag 녹화 제어 |

### SendCommand Commands

| Command | Value | Description |
|---------|-------|-------------|
| PREPARE | 0 | 녹화할 토픽 및 robot_type 설정 |
| START | 1 | 녹화 시작 (uri 필수) |
| STOP | 2 | 녹화 중지 |
| STOP_AND_DELETE | 3 | 녹화 중지 및 삭제 |
| FINISH | 4 | 녹화 종료 및 정리 |
