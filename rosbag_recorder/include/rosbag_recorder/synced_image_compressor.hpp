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

enum class CompressorState
{
  IDLE,
  RECORDING,
  ENCODING
};

struct TimestampedFrame
{
  cv::Mat frame;
  int64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
};

struct FrameOutput
{
  std::string topic;
  uint32_t frame_index;
  int64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
};

struct RecordingStats
{
  struct TopicStats
  {
    uint32_t received_frames{0};
  };

  std::unordered_map<std::string, TopicStats> topic_stats;
  uint32_t total_frames{0};
  int64_t recording_start_ns{0};
  int64_t recording_end_ns{0};
  size_t peak_ram_usage_bytes{0};
  size_t current_ram_usage_bytes{0};
};

class ImageCompressorRaw
{
public:
  struct Config
  {
    double target_fps{30.0};
    size_t ram_limit_bytes{16ULL * 1024 * 1024 * 1024};
    int ffmpeg_crf{23};
    std::string ffmpeg_preset{"fast"};
  };

  using FrameCallback = std::function<void(const FrameOutput &)>;
  using EncodingCompleteCallback = std::function<void(bool success, const std::string & message)>;

  explicit ImageCompressorRaw(
    const std::string & output_dir,
    const std::vector<std::string> & topics);

  explicit ImageCompressorRaw(
    const std::string & output_dir,
    const std::vector<std::string> & topics,
    const Config & config);

  ~ImageCompressorRaw();

  void set_frame_callback(FrameCallback callback);
  void set_encoding_complete_callback(EncodingCompleteCallback callback);

  void add_frame(
    const std::string & topic,
    const sensor_msgs::msg::Image::SharedPtr & image_msg);

  void start_recording();
  void stop_recording();

  CompressorState get_state() const;
  bool is_recording() const;
  bool is_encoding() const;

  RecordingStats get_stats() const;

private:
  struct TopicData
  {
    std::mutex mutex;
    std::deque<TimestampedFrame> frame_buffer;
    uint32_t frame_count{0};
  };

  struct EncodeTask
  {
    std::string topic;
    std::vector<cv::Mat> frames;
    std::string output_path;
  };

  void encoding_thread_func();
  void encode_all_videos();
  void encode_video(const EncodeTask & task);
  cv::Mat convert_ros_image_to_bgr(const sensor_msgs::msg::Image::SharedPtr & image_msg);
  std::string build_ffmpeg_command(
    const std::string & output_path,
    uint32_t width,
    uint32_t height);
  std::string sanitize_topic_name(const std::string & topic_name) const;
  void update_ram_usage();
  int64_t get_current_time_ns() const;

  Config config_;
  std::string output_dir_;
  std::vector<std::string> topics_;

  std::atomic<CompressorState> state_{CompressorState::IDLE};

  std::unordered_map<std::string, std::unique_ptr<TopicData>> topic_data_;

  std::thread encoding_thread_;
  std::atomic<bool> encoding_requested_{false};

  FrameCallback frame_callback_;
  EncodingCompleteCallback encoding_complete_callback_;

  mutable std::mutex stats_mutex_;
  RecordingStats stats_;
};

}  // namespace rosbag_recorder

#endif
