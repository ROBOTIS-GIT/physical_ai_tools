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

#include "rosbag_recorder/image_compressor.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <sstream>

namespace rosbag_recorder
{

ImageCompressor::ImageCompressor(const std::string & output_dir)
: output_dir_(output_dir)
{
  // Create output directory if it doesn't exist
  std::filesystem::create_directories(output_dir_);
}

ImageCompressor::~ImageCompressor()
{
  finalize_all();
}

std::string ImageCompressor::sanitize_topic_name(const std::string & topic_name) const
{
  std::string sanitized = topic_name;
  // Replace / with _
  std::replace(sanitized.begin(), sanitized.end(), '/', '_');
  // Remove leading underscore if present
  if (!sanitized.empty() && sanitized[0] == '_') {
    sanitized = sanitized.substr(1);
  }
  return sanitized;
}

std::string ImageCompressor::build_ffmpeg_command(
  const std::string & output_path,
  uint32_t width,
  uint32_t height,
  double fps)
{
  std::ostringstream cmd;

  // FFmpeg command for H.264 encoding via pipe
  // -y: overwrite output file
  // -f rawvideo: input format is raw video
  // -vcodec rawvideo: input codec is raw
  // -s WxH: input resolution
  // -pix_fmt bgr24: input pixel format (OpenCV default)
  // -r FPS: input frame rate
  // -i -: read from stdin
  // -c:v libx264: use H.264 encoder
  // -preset fast: encoding speed/quality tradeoff
  // -crf 23: quality (0-51, lower is better, 23 is default)
  // -pix_fmt yuv420p: output pixel format for compatibility

  cmd << "ffmpeg -y -f rawvideo -vcodec rawvideo "
      << "-s " << width << "x" << height << " "
      << "-pix_fmt bgr24 "
      << "-r " << fps << " "
      << "-i - "
      << "-c:v libx264 "
      << "-preset fast "
      << "-crf 23 "
      << "-pix_fmt yuv420p "
      << "-loglevel error "
      << "\"" << output_path << "\" 2>/dev/null";

  return cmd.str();
}

bool ImageCompressor::initialize_writer(
  const std::string & topic_name,
  uint32_t width,
  uint32_t height,
  double fps)
{
  if (has_writer(topic_name)) {
    return true;  // Already initialized
  }

  std::string sanitized_name = sanitize_topic_name(topic_name);
  std::string output_path = output_dir_ + "/" + sanitized_name + ".mp4";

  // Build FFmpeg command
  std::string cmd = build_ffmpeg_command(output_path, width, height, fps);

  // Open pipe to FFmpeg process
  FILE * pipe = popen(cmd.c_str(), "w");
  if (!pipe) {
    return false;
  }

  FFmpegWriterInfo info;
  info.pipe = pipe;
  info.frame_count = 0;
  info.output_path = output_path;
  info.width = width;
  info.height = height;
  info.fps = fps;

  writers_[topic_name] = info;
  return true;
}

cv::Mat ImageCompressor::convert_ros_image_to_bgr(
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  // Create cv::Mat from ROS image data
  int cv_type = CV_8UC3;  // Default to 3-channel 8-bit

  if (image_msg->encoding == "mono8") {
    cv_type = CV_8UC1;
  } else if (image_msg->encoding == "mono16") {
    cv_type = CV_16UC1;
  } else if (image_msg->encoding == "bgr8" || image_msg->encoding == "rgb8") {
    cv_type = CV_8UC3;
  } else if (image_msg->encoding == "bgra8" || image_msg->encoding == "rgba8") {
    cv_type = CV_8UC4;
  }

  cv::Mat raw_image(
    image_msg->height,
    image_msg->width,
    cv_type,
    const_cast<uint8_t *>(image_msg->data.data()),
    image_msg->step);

  cv::Mat bgr_image;

  // Convert to BGR format for FFmpeg
  if (image_msg->encoding == "rgb8") {
    cv::cvtColor(raw_image, bgr_image, cv::COLOR_RGB2BGR);
  } else if (image_msg->encoding == "rgba8") {
    cv::cvtColor(raw_image, bgr_image, cv::COLOR_RGBA2BGR);
  } else if (image_msg->encoding == "bgra8") {
    cv::cvtColor(raw_image, bgr_image, cv::COLOR_BGRA2BGR);
  } else if (image_msg->encoding == "mono8") {
    cv::cvtColor(raw_image, bgr_image, cv::COLOR_GRAY2BGR);
  } else if (image_msg->encoding == "mono16") {
    cv::Mat mono8;
    raw_image.convertTo(mono8, CV_8UC1, 255.0 / 65535.0);
    cv::cvtColor(mono8, bgr_image, cv::COLOR_GRAY2BGR);
  } else {
    // Assume BGR8 or compatible
    bgr_image = raw_image.clone();
  }

  return bgr_image;
}

ImageMetadata ImageCompressor::add_frame(
  const std::string & topic_name,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  // Initialize writer if not exists
  if (!has_writer(topic_name)) {
    bool success = initialize_writer(
      topic_name,
      image_msg->width,
      image_msg->height);

    if (!success) {
      throw std::runtime_error(
              "Failed to initialize FFmpeg writer for topic: " + topic_name);
    }
  }

  auto & writer_info = writers_[topic_name];

  // Convert ROS image to BGR Mat
  cv::Mat frame = convert_ros_image_to_bgr(image_msg);

  // Resize if dimensions don't match
  if (static_cast<uint32_t>(frame.cols) != writer_info.width ||
    static_cast<uint32_t>(frame.rows) != writer_info.height)
  {
    cv::resize(frame, frame, cv::Size(writer_info.width, writer_info.height));
  }

  // Ensure frame is contiguous
  if (!frame.isContinuous()) {
    frame = frame.clone();
  }

  // Write raw frame data to FFmpeg pipe
  size_t frame_size = frame.total() * frame.elemSize();
  size_t written = fwrite(frame.data, 1, frame_size, writer_info.pipe);

  if (written != frame_size) {
    throw std::runtime_error("Failed to write frame to FFmpeg pipe");
  }

  // Create metadata
  ImageMetadata metadata;
  metadata.frame_index = writer_info.frame_count;
  metadata.timestamp_ns = image_msg->header.stamp.sec * 1000000000LL +
    image_msg->header.stamp.nanosec;
  metadata.width = image_msg->width;
  metadata.height = image_msg->height;
  metadata.encoding = image_msg->encoding;

  writer_info.frame_count++;

  return metadata;
}

void ImageCompressor::finalize_writer(const std::string & topic_name)
{
  auto it = writers_.find(topic_name);
  if (it != writers_.end()) {
    if (it->second.pipe) {
      pclose(it->second.pipe);
      it->second.pipe = nullptr;
    }
    writers_.erase(it);
  }
}

void ImageCompressor::finalize_all()
{
  for (auto & [topic_name, writer_info] : writers_) {
    if (writer_info.pipe) {
      pclose(writer_info.pipe);
      writer_info.pipe = nullptr;
    }
  }
  writers_.clear();
}

bool ImageCompressor::has_writer(const std::string & topic_name) const
{
  return writers_.find(topic_name) != writers_.end();
}

}  // namespace rosbag_recorder
