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
// Author: Dongyun Kim

#ifndef ROSBAG_RECORDER__IMAGE_COMPRESSOR_HPP_
#define ROSBAG_RECORDER__IMAGE_COMPRESSOR_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <deque>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace rosbag_recorder
{

struct ImageMetadata
{
  uint32_t frame_index;
  int64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
  std::string encoding;
};

// Buffered frame for FPS detection
struct BufferedFrame
{
  cv::Mat frame;
  int64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
  std::string encoding;
};

class ImageCompressor
{
public:
  explicit ImageCompressor(
    const std::string & output_dir,
    size_t fps_detection_frames = 10);
  ~ImageCompressor();

  // Add image frame to MP4 and return metadata
  // FPS is automatically detected from timestamp intervals
  ImageMetadata add_frame(
    const std::string & topic_name,
    const sensor_msgs::msg::Image::SharedPtr & image_msg);

  // Finalize and close all video writers
  void finalize_all();

  // Finalize specific video writer
  void finalize_writer(const std::string & topic_name);

  // Check if writer is active (initialized after FPS detection)
  bool has_active_writer(const std::string & topic_name) const;

  // Check if topic is being tracked (buffering or active)
  bool is_tracking(const std::string & topic_name) const;

  // Get detected FPS for a topic (0 if not yet detected)
  double get_detected_fps(const std::string & topic_name) const;

private:
  struct FFmpegWriterInfo
  {
    FILE * pipe;
    uint32_t frame_count;
    std::string output_path;
    uint32_t width;
    uint32_t height;
    double fps;
    bool is_initialized;
  };

  struct TopicBufferInfo
  {
    std::deque<BufferedFrame> frames;
    std::vector<int64_t> timestamps;
    double detected_fps;
    bool fps_detected;
  };

  std::string output_dir_;
  size_t fps_detection_frames_;
  std::unordered_map<std::string, FFmpegWriterInfo> writers_;
  std::unordered_map<std::string, TopicBufferInfo> topic_buffers_;

  std::string sanitize_topic_name(const std::string & topic_name) const;
  cv::Mat convert_ros_image_to_bgr(const sensor_msgs::msg::Image::SharedPtr & image_msg);
  std::string build_ffmpeg_command(
    const std::string & output_path,
    uint32_t width,
    uint32_t height,
    double fps);

  // Initialize FFmpeg writer after FPS detection
  bool initialize_writer(
    const std::string & topic_name,
    uint32_t width,
    uint32_t height,
    double fps);

  // Calculate FPS from collected timestamps
  double calculate_fps_from_timestamps(const std::vector<int64_t> & timestamps) const;

  // Write buffered frames to initialized writer
  void flush_buffered_frames(const std::string & topic_name);

  // Write single frame to FFmpeg pipe
  void write_frame_to_pipe(FFmpegWriterInfo & writer_info, const cv::Mat & frame);
};

}  // namespace rosbag_recorder

#endif  // ROSBAG_RECORDER__IMAGE_COMPRESSOR_HPP_
