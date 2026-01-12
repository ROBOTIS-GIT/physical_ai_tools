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

class ImageCompressor
{
public:
  explicit ImageCompressor(const std::string & output_dir);
  ~ImageCompressor();

  // Initialize FFmpeg writer for a specific topic
  bool initialize_writer(
    const std::string & topic_name,
    uint32_t width,
    uint32_t height,
    double fps = 30.0);

  // Add image frame to MP4 and return metadata
  ImageMetadata add_frame(
    const std::string & topic_name,
    const sensor_msgs::msg::Image::SharedPtr & image_msg);

  // Finalize and close all video writers
  void finalize_all();

  // Finalize specific video writer
  void finalize_writer(const std::string & topic_name);

  // Check if writer exists for topic
  bool has_writer(const std::string & topic_name) const;

private:
  struct FFmpegWriterInfo
  {
    FILE * pipe;
    uint32_t frame_count;
    std::string output_path;
    uint32_t width;
    uint32_t height;
    double fps;
  };

  std::string output_dir_;
  std::unordered_map<std::string, FFmpegWriterInfo> writers_;

  std::string sanitize_topic_name(const std::string & topic_name) const;
  cv::Mat convert_ros_image_to_bgr(const sensor_msgs::msg::Image::SharedPtr & image_msg);
  std::string build_ffmpeg_command(
    const std::string & output_path,
    uint32_t width,
    uint32_t height,
    double fps);
};

}  // namespace rosbag_recorder

#endif  // ROSBAG_RECORDER__IMAGE_COMPRESSOR_HPP_
