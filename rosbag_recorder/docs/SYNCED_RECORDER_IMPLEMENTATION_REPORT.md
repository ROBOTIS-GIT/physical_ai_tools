# Synced Image Bag Recorder 구현 리포트

## 1. 개요

### 1.1 목표
- 카메라 간 timestamp 차이 최소화
- 모든 카메라 프레임 수 동일 보장
- 취득 중 CPU 부하 최소화
- 프레임 드랍 최소화

### 1.2 구현 범위
- `SyncedImageCompressor`: 동기화된 이미지 압축기
- `SyncedImageBagRecorder`: 동기화된 ROSbag 레코더
- 청크 단위 백그라운드 인코딩
- 동기화 통계 리포트 생성

## 2. 아키텍처

### 2.1 전체 구조

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       SyncedImageBagRecorder Node                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐     ┌──────────────────────────────────────────────────┐  │
│  │   Non-Image  │     │                Image Topics                       │  │
│  │   Topics     │     │  (cam_left, cam_right, cam_zed)                  │  │
│  └──────┬───────┘     └───────────────────┬──────────────────────────────┘  │
│         │                                 │                                 │
│         │                                 ▼                                 │
│         │             ┌───────────────────────────────────────────────────┐ │
│         │             │            SyncedImageCompressor                  │ │
│         │             │  ┌─────────────────────────────────────────────┐  │ │
│         │             │  │  State Machine                               │  │ │
│         │             │  │  IDLE → INITIALIZING → RECORDING → STOPPING │  │ │
│         │             │  └─────────────────────────────────────────────┘  │ │
│         │             │                                                   │ │
│         │             │  ┌─────────────────┐  ┌─────────────────────────┐ │ │
│         │             │  │ Latest Frame    │  │ Frame Buffer (RAM)      │ │ │
│         │             │  │ Buffer          │  │ (per camera, per chunk) │ │ │
│         │             │  │ (per camera)    │  │                         │ │ │
│         │             │  └────────┬────────┘  └────────────┬────────────┘ │ │
│         │             │           │                        │              │ │
│         │             │           ▼                        │              │ │
│         │             │  ┌─────────────────┐              │              │ │
│         │             │  │ Sampling Thread │              │              │ │
│         │             │  │ (30fps fixed)   │──────────────┘              │ │
│         │             │  │ Causal Sync     │                             │ │
│         │             │  └────────┬────────┘                             │ │
│         │             │           │                                       │ │
│         │             │           │ (every 30 sec)                       │ │
│         │             │           ▼                                       │ │
│         │             │  ┌─────────────────┐     ┌─────────────────────┐  │ │
│         │             │  │ Encoding Thread │────▶│  MP4 Chunks         │  │ │
│         │             │  │ (background)    │     │  _chunk0, _chunk1.. │  │ │
│         │             │  └─────────────────┘     └──────────┬──────────┘  │ │
│         │             │                                     │             │ │
│         │             │                          (finalize) │             │ │
│         │             │                                     ▼             │ │
│         │             │                          ┌──────────────────────┐ │ │
│         │             │                          │  Final MP4 (concat)  │ │ │
│         │             │                          │  camera_left.mp4     │ │ │
│         │             │                          │  camera_right.mp4    │ │ │
│         │             │                          │  camera_zed.mp4      │ │ │
│         │             │                          └──────────────────────┘ │ │
│         │             └───────────────────────────────────────────────────┘ │
│         │                                                                   │
│         ▼                                                                   │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                           ROSbag Writer                               │   │
│  │  - Non-image topics: 원본 메시지 저장                                 │   │
│  │  - Image topics: ImageMetadata 메시지만 저장 (synced_frame_callback) │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 상태 머신

```
         start_recording()
              │
              ▼
┌───────┐  ┌─────────────┐  all cameras  ┌───────────┐
│ IDLE  │──│INITIALIZING │─────────────▶│ RECORDING │
└───────┘  └─────────────┘   have data   └─────┬─────┘
    ▲                                          │
    │           stop_recording()               │
    │                                          ▼
    │      ┌──────────────────────────────────────┐
    └──────│              STOPPING                 │
           │  - Stop sampling thread               │
           │  - Finalize remaining chunks          │
           │  - Concat MP4 chunks                  │
           │  - Write stats report                 │
           └──────────────────────────────────────┘
```

## 3. 핵심 알고리즘

### 3.1 초기화 단계 (try_initialize)

```cpp
void SyncedImageCompressor::try_initialize()
{
    // 1. 모든 카메라에 데이터가 있는지 확인
    bool all_have_data = true;
    for (const auto& topic : topics_) {
        if (!topic_data_[topic]->has_data) {
            all_have_data = false;
            break;
        }
    }
    
    // 2. 모든 카메라의 timestamp 수집
    int64_t max_timestamp = 0;
    int64_t min_timestamp = INT64_MAX;
    for (const auto& topic : topics_) {
        max_timestamp = max(max_timestamp, topic_data_[topic]->latest_frame.timestamp_ns);
        min_timestamp = min(min_timestamp, topic_data_[topic]->latest_frame.timestamp_ns);
    }
    
    // 3. timestamp 차이가 init_window 이내인지 확인
    double time_diff_sec = (max_timestamp - min_timestamp) / 1e9;
    if (time_diff_sec > config_.init_window_sec) {
        return;  // 아직 안정화되지 않음
    }
    
    // 4. anchor 설정 (가장 늦은 timestamp)
    anchor_timestamp_ns_ = max_timestamp;
    
    // 5. 샘플링 스레드 시작
    state_ = State::RECORDING;
    sampling_thread_ = thread([this]() { /* 30fps 루프 */ });
}
```

### 3.2 Causal Sync 샘플링 (sample_synced_frame)

```cpp
void SyncedImageCompressor::sample_synced_frame()
{
    int64_t target_time = anchor_timestamp_ns_ + frame_index_ * frame_duration_ns_;
    
    SyncedFrameOutput output;
    output.frame_index = frame_index_;
    
    for (const auto& topic : topics_) {
        auto& data = topic_data_[topic];
        
        // Causal Sync: target_time 이전의 최신 데이터만 사용
        if (data->latest_frame.timestamp_ns <= target_time) {
            int64_t staleness_ns = target_time - data->latest_frame.timestamp_ns;
            
            // 프레임 복사 및 버퍼에 저장
            output.frames[topic] = data->latest_frame;
            output.staleness_ns[topic] = staleness_ns;
            data->frame_buffer.push_back(data->latest_frame);
        }
    }
    
    // 모든 카메라 프레임이 있으면 성공
    if (output.frames.size() == topics_.size()) {
        synced_frame_callback_(output);  // 메타데이터 저장
        frame_index_++;
        
        // 30초마다 청크 인코딩 트리거
        if (frame_index_ % frames_per_chunk_ == 0) {
            trigger_chunk_encoding(frame_index_ / frames_per_chunk_ - 1);
        }
    }
}
```

### 3.3 청크 단위 인코딩

```cpp
void SyncedImageCompressor::trigger_chunk_encoding(uint32_t chunk_index)
{
    for (const auto& topic : topics_) {
        ChunkTask task;
        task.topic = topic;
        task.chunk_index = chunk_index;
        task.output_path = output_dir_ + "/" + sanitize(topic) + "_chunk" + to_string(chunk_index) + ".mp4";
        
        // 버퍼에서 프레임 추출
        for (size_t i = 0; i < frames_per_chunk_; ++i) {
            task.frames.push_back(move(topic_data_[topic]->frame_buffer.front().frame));
            topic_data_[topic]->frame_buffer.pop_front();
        }
        
        // 인코딩 큐에 추가 (백그라운드 스레드가 처리)
        encoding_queue_.push_back(move(task));
        encoding_cv_.notify_one();
    }
}
```

## 4. 설정 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `target_fps` | 30.0 | 목표 프레임 레이트 |
| `init_window_sec` | 0.15 | 초기화 윈도우 (150ms) |
| `chunk_duration_sec` | 30.0 | 청크 길이 (30초) |
| `ram_limit_bytes` | 16GB | RAM 사용량 제한 |
| `ffmpeg_crf` | 23 | 비디오 품질 (0-51) |
| `ffmpeg_preset` | "fast" | 인코딩 속도 프리셋 |

## 5. 리소스 사용량 분석

### 5.1 RAM 사용량

```
카메라: 3개
해상도: 1280 × 720
FPS: 30
청크 길이: 30초

프레임 크기: 1280 × 720 × 3 = 2.76 MB
청크당 프레임: 30 × 30 = 900
청크당 RAM: 3 × 900 × 2.76 MB = 7.45 GB

최대 RAM 사용량: 약 7.5 GB (한 청크 버퍼링 중)
```

### 5.2 CPU 부하 비교

| 단계 | 기존 (실시간 인코딩) | 새 구현 (청크 인코딩) |
|------|---------------------|----------------------|
| 취득 중 | FFmpeg 3개 프로세스 상시 | 프레임 복사만 (낮음) |
| 청크 경계 | - | FFmpeg 3개 프로세스 (30초마다) |
| 종료 후 | - | 최종 청크 인코딩 + concat |

## 6. 출력 파일

### 6.1 디렉토리 구조

```
output_bag/
├── metadata.yaml
├── rosbag_0.db3
├── robot_config.yaml
├── videos/
│   ├── camera_left.mp4
│   ├── camera_right.mp4
│   └── camera_zed.mp4
└── sync_stats_report.json
```

### 6.2 sync_stats_report.json 형식

```json
{
  "total_synced_frames": 572,
  "recording_duration_sec": 19.07,
  "peak_ram_usage_mb": 1234.56,
  "topics": {
    "/camera_left/image": {
      "received_frames": 580,
      "synced_frames": 572,
      "dropped_frames": 8,
      "avg_staleness_ms": 5.2,
      "max_staleness_ms": 15.3,
      "min_staleness_ms": 0.1
    }
  },
  "frame_counts": {
    "/camera_left/image": 572,
    "/camera_right/image": 572,
    "/camera_zed/image": 572
  }
}
```

## 7. 사용 방법

### 7.1 빌드

```bash
cd /root/ros2_ws
colcon build --packages-select rosbag_recorder --symlink-install
source install/setup.bash
```

### 7.2 실행

```bash
ros2 run rosbag_recorder synced_image_bag_recorder
```

### 7.3 서비스 호출

```bash
# PREPARE
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 0, topics: ['/cam_left/image', '/cam_right/image', '/cam_zed/image']}"

# START
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 1, uri: '/workspace/output'}"

# STOP
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 2}"
```

## 8. 검증 방법

### 8.1 프레임 수 일치 확인

```bash
for f in output/videos/*.mp4; do
  echo -n "$f: "
  ffprobe -v error -count_frames -select_streams v:0 \
    -show_entries stream=nb_read_frames -of csv=p=0 "$f"
done
```

### 8.2 Staleness 분석

```bash
cat output/sync_stats_report.json | jq '.topics | to_entries[] | {topic: .key, avg_staleness_ms: .value.avg_staleness_ms}'
```

## 9. 제한사항

1. **RAM 요구사항**: 30초 × 3카메라 × 2.76MB = 약 7.5GB 필요
2. **초기화 지연**: 모든 카메라 데이터 도착까지 최대 150ms 대기
3. **실시간 처리 아님**: 청크 인코딩 중 CPU 부하 발생

## 10. 향후 개선 방향

1. **하드웨어 동기화**: GPIO 트리거로 물리적 동시 촬영
2. **GPU 인코딩**: NVENC 활용으로 인코딩 속도 향상
3. **적응형 청크**: RAM 사용량에 따라 청크 크기 자동 조절
