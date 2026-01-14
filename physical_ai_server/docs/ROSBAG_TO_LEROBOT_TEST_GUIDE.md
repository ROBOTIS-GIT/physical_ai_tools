# ROSbag to LeRobot Converter Test Guide

ROSbag(MCAP + MP4) 데이터를 LeRobot 데이터셋으로 변환하는 기능을 테스트하기 위한 가이드입니다.

## Prerequisites

- Docker 컨테이너 실행 중: `physical_ai_server`
- HuggingFace 테스트 데이터셋: `Dongkkka/physical_ai_tools_test_rosbag`

## Test Workflow

### 0. 테스트 환경 정리 (중요!)

테스트 시작 전 기존에 실행 중인 rosbag play나 physical_ai_server를 종료해야 합니다.
여러 개의 rosbag play가 동시에 실행되면 토픽이 중복 발행되어 데이터가 손상됩니다.

```bash
# 실행 중인 ROS2 노드 확인
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
ros2 node list
'

# rosbag play가 실행 중인 경우 (rosbag2_player 노드가 보이면)
# 해당 터미널에서 Ctrl+C로 종료하거나:
docker exec physical_ai_server pkill -f "ros2 bag play"

# physical_ai_server가 실행 중인 경우
# 해당 터미널에서 Ctrl+C로 종료
```

> **Warning**: rosbag play가 여러 개 실행되면 동일한 토픽에 메시지가 2배로 발행되어
> 녹화 데이터의 주파수가 비정상적으로 높아집니다 (예: 30Hz → 60Hz).

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

새 터미널에서 rosbag play를 실행하여 로봇이 동작하는 것처럼 토픽을 발행합니다.

> **Important**: 반드시 하나의 rosbag play만 실행하세요!

```bash
# Terminal 1: rosbag play (loop mode)
docker exec -it physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
ros2 bag play /workspace/test_rosbag/0 --loop --rate 1.0
'
```

토픽이 정상적으로 발행되는지 확인:
```bash
# 토픽 주파수 확인 (약 30Hz여야 정상)
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
ros2 topic hz /joint_states --window 10
'
# 100Hz 근처면 정상, 200Hz 이상이면 rosbag play가 중복 실행 중
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

### 환경 준비
- [ ] 기존 rosbag play 종료 확인 (중복 실행 방지)
- [ ] 기존 physical_ai_server 종료 확인
- [ ] 테스트 데이터 다운로드 완료

### 데이터 취득
- [ ] rosbag play 실행 중 (단일 인스턴스만!)
- [ ] 토픽 주파수 정상 확인 (/joint_states ~100Hz, 카메라 ~30Hz)
- [ ] physical_ai_server 실행 중
- [ ] rosbag_recorder 실행 중
- [ ] SetRobotType 서비스 호출 성공
- [ ] PREPARE 서비스 호출 성공 (robot_type 포함)
- [ ] START 서비스 호출 성공
- [ ] 12초 녹화 후 STOP 성공

### 녹화 결과 검증
- [ ] robot_config.yaml에 camera_mapping 포함 확인
- [ ] robot_config.yaml에 joint_order가 flat list로 저장됨 확인
- [ ] videos/ 폴더에 3개 MP4 파일 생성 확인
- [ ] MCAP 파일 생성 확인

### LeRobot 변환 검증
- [ ] 변환 성공 (에러 없이 완료)
- [ ] 변환된 데이터셋의 카메라 이름이 올바른지 확인:
  - `observation.images.cam_head`
  - `observation.images.cam_wrist_left`
  - `observation.images.cam_wrist_right`
- [ ] info.json의 observation.state에 19개 joint 이름 포함 확인
- [ ] info.json의 total_videos가 3인지 확인
- [ ] videos/ 폴더에 3개 카메라별 MP4 파일 존재 확인

---

## 추가 기능 테스트

### joint_order 처리 테스트

`robot_config.yaml`의 `joint_order`가 nested dict 형태이거나 `total_joint_order`가 있을 때 변환기가 올바르게 처리하는지 테스트합니다.

#### 테스트 케이스 1: total_joint_order 우선 사용

`robot_config.yaml`에 `total_joint_order`가 있으면 이를 우선 사용해야 합니다.

```bash
# robot_config.yaml에 total_joint_order가 있는 경우
docker exec physical_ai_server cat /workspace/test_recording/robot_config.yaml | grep -A 20 "joint_order"
```

변환 로그에서 확인:
```
[INFO] Loaded total_joint_order with 19 joints
```

#### 테스트 케이스 2: nested dict joint_order 처리

`total_joint_order`가 없고 `joint_order`가 nested dict인 경우 자동으로 flatten해야 합니다.

```yaml
# 이런 형태의 joint_order를
joint_order:
  leader_left:
    - arm_l_joint1
    - arm_l_joint2
  leader_right:
    - arm_r_joint1
    - arm_r_joint2

# 이렇게 처리해야 함
# [arm_l_joint1, arm_l_joint2, arm_r_joint1, arm_r_joint2]
```

변환 로그에서 확인:
```
[INFO] Loaded joint_order (flattened from dict) with N joints
```

### LeRobot 실시간 저장 제거 확인

physical_ai_server에서 LeRobot 실시간 저장이 제거되었는지 확인합니다.

```bash
# LeRobot 관련 코드가 제거되었는지 확인
docker exec physical_ai_server grep -rn "LeRobotDatasetWrapper\|lerobot_dataset\|\.add_frame\|\.save_episode" \
  /root/ros2_ws/src/physical_ai_tools/physical_ai_server/physical_ai_server/data_processing/data_manager.py \
  /root/ros2_ws/src/physical_ai_tools/physical_ai_server/physical_ai_server/physical_ai_server.py \
  2>/dev/null || echo "✅ LeRobot 실시간 저장 코드가 정상적으로 제거됨"

# check_lerobot_dataset 호출이 제거되었는지 확인
docker exec physical_ai_server grep -n "check_lerobot_dataset" \
  /root/ros2_ws/src/physical_ai_tools/physical_ai_server/physical_ai_server/physical_ai_server.py \
  || echo "✅ check_lerobot_dataset 호출이 정상적으로 제거됨"
```

### 변환 품질 검증

변환된 데이터의 품질을 검증합니다.

```bash
# parquet 파일 내용 확인
docker exec physical_ai_server python3 -c "
import pyarrow.parquet as pq
table = pq.read_table('/workspace/lerobot_test_output/data/chunk-000/episode_000000.parquet')
print('Columns:', table.column_names)
print('Rows:', table.num_rows)
print('State shape:', len(table['observation.state'][0].as_py()))
print('Action shape:', len(table['action'][0].as_py()))
"

# 비디오 프레임 수와 parquet 행 수 일치 확인
docker exec physical_ai_server bash -c '
PARQUET_ROWS=$(python3 -c "import pyarrow.parquet as pq; print(pq.read_table(\"/workspace/lerobot_test_output/data/chunk-000/episode_000000.parquet\").num_rows)")
echo "Parquet rows: $PARQUET_ROWS"

for f in /workspace/lerobot_test_output/videos/chunk-000/*/episode_000000.mp4; do
  FRAMES=$(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames -of csv=p=0 "$f")
  echo "$(basename $(dirname $f)): $FRAMES frames"
done
'
```

---

## Troubleshooting

### rosbag play 중복 실행

**증상**: 녹화된 데이터의 주파수가 예상보다 2배 높음 (예: 30Hz → 60Hz)

**원인**: 여러 개의 rosbag play가 동시에 실행됨

**해결**:
```bash
# 모든 rosbag play 종료
docker exec physical_ai_server pkill -f "ros2 bag play"

# 단일 인스턴스만 실행
docker exec -it physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
ros2 bag play /workspace/test_rosbag/0 --loop --rate 1.0
'
```

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

### "No state messages found" 에러

**증상**: 변환 시 "No state messages found" 에러 발생

**원인**: 
- robot_config.yaml의 state_topics 설정이 잘못됨
- joint_order가 nested dict인데 처리되지 않음

**해결**:
```bash
# state_topics 확인
docker exec physical_ai_server cat /workspace/test_recording/robot_config.yaml | grep -A 3 state_topics

# MCAP 파일에 /joint_states 토픽이 있는지 확인
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
ros2 bag info /workspace/test_recording
' | grep joint_states
```

### joint_order 관련 에러

**증상**: "Joint 'xxx' from joint_order not found in message" 경고가 많이 발생

**원인**: robot_config.yaml의 joint_order에 실제 메시지에 없는 joint가 포함됨

**해결**:
- `total_joint_order` 또는 `joint_order`가 실제 /joint_states 메시지의 joint와 일치하는지 확인
- 불필요한 joint 제거 또는 올바른 joint 이름으로 수정

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

---

## Synced Image Bag Recorder 테스트

카메라 간 프레임 동기화가 필요한 경우 `synced_image_bag_recorder`를 사용합니다.

### 1. Synced Recorder 실행

```bash
# Terminal: synced_image_bag_recorder
docker exec -it physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run rosbag_recorder synced_image_bag_recorder
'
```

### 2. 동기화된 녹화 테스트

서비스 인터페이스는 기존과 동일합니다:

```bash
# PREPARE
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand "{
  command: 0,
  topics: [
    \"/camera_left/camera_left/color/image_rect_raw\",
    \"/camera_right/camera_right/color/image_rect_raw\",
    \"/zed/zed_node/left/image_rect_color\",
    \"/joint_states\"
  ],
  robot_type: \"ffw_bg2_rev4\"
}"
'

# START
docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand "{
  command: 1,
  uri: \"/workspace/test_synced_recording\"
}"
'

# 12초 대기 후 STOP
sleep 12

docker exec physical_ai_server bash -c '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand "{command: 2}"
'
```

### 3. 동기화 결과 확인

```bash
# 동기화 통계 확인
docker exec physical_ai_server cat /workspace/test_synced_recording/sync_stats_report.json

# 프레임 수 일치 확인
docker exec physical_ai_server bash -c '
for f in /workspace/test_synced_recording/videos/*.mp4; do
  echo -n "$f: "
  ffprobe -v error -count_frames -select_streams v:0 \
    -show_entries stream=nb_read_frames -of csv=p=0 "$f"
done
'
```

### 4. 기대 결과

- 모든 카메라 MP4의 프레임 수가 동일
- `sync_stats_report.json`에서 `frame_counts`의 모든 값이 동일
- staleness (데이터 지연)가 1 프레임(33ms) 이내

### Synced vs Non-synced 비교

| 항목 | image_bag_recorder | synced_image_bag_recorder |
|------|-------------------|---------------------------|
| 프레임 동기화 | ❌ 독립 녹화 | ✅ Causal Sync |
| 프레임 수 일치 | ❌ 불일치 가능 | ✅ 보장 |
| CPU 부하 (취득 중) | 높음 (실시간 인코딩) | 낮음 (RAM 버퍼) |
| RAM 사용량 | 낮음 | 높음 (30초분 버퍼) |
| 통계 리포트 | ❌ 없음 | ✅ sync_stats_report.json |

---

## 테스트 후 정리

테스트가 완료되면 실행 중인 프로세스와 테스트 데이터를 정리합니다.

```bash
# 1. rosbag play 종료 (해당 터미널에서 Ctrl+C 또는)
docker exec physical_ai_server pkill -f "ros2 bag play"

# 2. physical_ai_server 종료 (해당 터미널에서 Ctrl+C)

# 3. 테스트 데이터 정리 (선택사항)
docker exec physical_ai_server rm -rf /workspace/test_rosbag
docker exec physical_ai_server rm -rf /workspace/test_recording
docker exec physical_ai_server rm -rf /workspace/lerobot_test_output
```

---

## 변경 이력

| 날짜 | 변경 내용 |
|------|----------|
| 2026-01-13 | rosbag play 중복 실행 방지 섹션 추가 |
| 2026-01-13 | joint_order 처리 테스트 섹션 추가 |
| 2026-01-13 | LeRobot 실시간 저장 제거 확인 테스트 추가 |
| 2026-01-13 | 변환 품질 검증 섹션 추가 |
| 2026-01-13 | Test Checklist 상세화 |
