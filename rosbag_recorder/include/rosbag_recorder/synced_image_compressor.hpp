// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Sisyphus (AI Agent)

#ifndef ROSBAG_RECORDER__SYNCED_IMAGE_COMPRESSOR_HPP_
#define ROSBAG_RECORDER__SYNCED_IMAGE_COMPRESSOR_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rosbag_recorder
{

struct TimestampedFrame
{
  cv::Mat frame;
  int64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
  std::string encoding;
};

struct SyncedFrameOutput
{
  uint32_t frame_index;
  int64_t anchor_timestamp_ns;
  std::unordered_map<std::string, TimestampedFrame> frames;
  std::unordered_map<std::string, int64_t> staleness_ns;
};

struct RecordingStats
{
  struct TopicStats
  {
    uint32_t received_frames{0};
    uint32_t synced_frames{0};
    uint32_t dropped_frames{0};
    int64_t total_staleness_ns{0};
    int64_t max_staleness_ns{0};
    int64_t min_staleness_ns{INT64_MAX};
  };

  std::unordered_map<std::string, TopicStats> topic_stats;
  uint32_t total_synced_frames{0};
  int64_t recording_start_ns{0};
  int64_t recording_end_ns{0};

  size_t peak_ram_usage_bytes{0};
  size_t current_ram_usage_bytes{0};
  double peak_cpu_percent{0.0};
};

struct ChunkResult
{
  std::string topic;
  uint32_t chunk_index;
  uint32_t frame_count;
  std::string output_path;
  bool success;
  std::string error_message;
};

class SyncedImageCompressor
{
public:
  struct Config
  {
    double target_fps{30.0};
    double init_window_sec{0.15};
    double chunk_duration_sec{30.0};
    size_t ram_limit_bytes{16ULL * 1024 * 1024 * 1024};
    int ffmpeg_crf{23};
    std::string ffmpeg_preset{"fast"};
    bool enable_background_encoding{true};
  };

  using SyncedFrameCallback = std::function<void(const SyncedFrameOutput &)>;

  explicit SyncedImageCompressor(
    const std::string & output_dir,
    const std::vector<std::string> & topics);

  explicit SyncedImageCompressor(
    const std::string & output_dir,
    const std::vector<std::string> & topics,
    const Config & config);

  ~SyncedImageCompressor();

  void set_synced_frame_callback(SyncedFrameCallback callback);

  void add_incoming_frame(
    const std::string & topic,
    const sensor_msgs::msg::Image::SharedPtr & image_msg);

  void start_recording();

  void stop_recording();

  bool is_recording() const;

  bool is_initialized() const;

  RecordingStats get_stats() const;

  double get_encoding_time_remaining() const;

  void wait_for_encoding_complete();

private:
  enum class State
  {
    IDLE,
    INITIALIZING,
    RECORDING,
    STOPPING,
    ENCODING_FINAL
  };

  struct TopicData
  {
    std::mutex mutex;
    TimestampedFrame latest_frame;
    bool has_data{false};
    std::deque<TimestampedFrame> frame_buffer;
    uint32_t encoded_frame_count{0};
    uint32_t current_chunk_index{0};
  };

  struct ChunkTask
  {
    std::string topic;
    uint32_t chunk_index;
    std::vector<cv::Mat> frames;
    std::string output_path;
  };

  void try_initialize();
  void sample_synced_frame();
  void trigger_chunk_encoding(uint32_t chunk_index);
  void encoding_thread_func();
  void encode_chunk(const ChunkTask & task);
  void finalize_all_chunks();
  void concat_chunks();
  cv::Mat convert_ros_image_to_bgr(const sensor_msgs::msg::Image::SharedPtr & image_msg);
  std::string build_ffmpeg_command(
    const std::string & output_path,
    uint32_t width,
    uint32_t height,
    double fps);
  std::string sanitize_topic_name(const std::string & topic_name) const;
  void update_ram_usage();
  int64_t get_current_time_ns() const;

  Config config_;
  std::string output_dir_;
  std::vector<std::string> topics_;

  std::atomic<State> state_{State::IDLE};
  std::atomic<bool> initialized_{false};

  int64_t anchor_timestamp_ns_{0};
  int64_t frame_duration_ns_{0};
  uint32_t frame_index_{0};
  uint32_t frames_per_chunk_{0};

  std::unordered_map<std::string, std::unique_ptr<TopicData>> topic_data_;

  std::thread sampling_thread_;
  std::atomic<bool> sampling_active_{false};
  std::mutex sampling_mutex_;
  std::condition_variable sampling_cv_;

  std::thread encoding_thread_;
  std::atomic<bool> encoding_active_{false};
  std::mutex encoding_mutex_;
  std::condition_variable encoding_cv_;
  std::deque<ChunkTask> encoding_queue_;
  std::atomic<uint32_t> pending_chunks_{0};

  SyncedFrameCallback synced_frame_callback_;

  mutable std::mutex stats_mutex_;
  RecordingStats stats_;
};

}  // namespace rosbag_recorder

#endif
