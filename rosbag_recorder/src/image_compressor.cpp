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
#include <iomanip>
#include <stdexcept>
#include <sstream>

namespace rosbag_recorder
{

ImageCompressor::ImageCompressor(const std::string & output_dir, double fps)
: output_dir_(output_dir),
  fps_(fps)
{
  std::filesystem::create_directories(output_dir_);
  std::fprintf(stderr, "[ImageCompressor] Created with fixed FPS: %.2f\n", fps_);
}

ImageCompressor::~ImageCompressor()
{
  finalize_all();
}

std::string ImageCompressor::sanitize_topic_name(const std::string & topic_name) const
{
  std::string sanitized = topic_name;
  std::replace(sanitized.begin(), sanitized.end(), '/', '_');
  if (!sanitized.empty() && sanitized[0] == '_') {
    sanitized = sanitized.substr(1);
  }
  return sanitized;
}

std::string ImageCompressor::build_ffmpeg_command(
  const std::string & output_path,
  uint32_t width,
  uint32_t height)
{
  std::ostringstream cmd;

  cmd << "ffmpeg -y -f rawvideo -vcodec rawvideo "
      << "-s " << width << "x" << height << " "
      << "-pix_fmt bgr24 "
      << "-r " << std::fixed << std::setprecision(2) << fps_ << " "
      << "-i - "
      << "-r " << std::fixed << std::setprecision(2) << fps_ << " "
      << "-c:v libx264 "
      << "-preset fast "
      << "-crf 23 "
      << "-pix_fmt yuv420p "
      << "-movflags +faststart "
      << "-loglevel error "
      << "\"" << output_path << "\" 2>/dev/null";

  return cmd.str();
}

bool ImageCompressor::initialize_writer(
  const std::string & topic_name,
  uint32_t width,
  uint32_t height)
{
  if (has_active_writer(topic_name)) {
    return true;
  }

  std::string sanitized_name = sanitize_topic_name(topic_name);
  std::string output_path = output_dir_ + "/" + sanitized_name + ".mp4";

  std::fprintf(
    stderr,
    "[ImageCompressor] Initializing writer for %s: %ux%u @ %.2f fps -> %s\n",
    topic_name.c_str(), width, height, fps_, output_path.c_str());

  std::string cmd = build_ffmpeg_command(output_path, width, height);

  FILE * pipe = popen(cmd.c_str(), "w");
  if (!pipe) {
    std::fprintf(stderr, "[ImageCompressor] Failed to open FFmpeg pipe for %s\n", topic_name.c_str());
    return false;
  }

  FFmpegWriterInfo info;
  info.pipe = pipe;
  info.frame_count = 0;
  info.output_path = output_path;
  info.width = width;
  info.height = height;
  info.is_initialized = true;

  writers_[topic_name] = info;
  return true;
}

cv::Mat ImageCompressor::convert_ros_image_to_bgr(
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  int cv_type = CV_8UC3;

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
    bgr_image = raw_image.clone();
  }

  return bgr_image;
}

void ImageCompressor::write_frame_to_pipe(FFmpegWriterInfo & writer_info, const cv::Mat & frame)
{
  cv::Mat output_frame = frame;

  if (static_cast<uint32_t>(frame.cols) != writer_info.width ||
    static_cast<uint32_t>(frame.rows) != writer_info.height)
  {
    cv::resize(frame, output_frame, cv::Size(writer_info.width, writer_info.height));
  }

  if (!output_frame.isContinuous()) {
    output_frame = output_frame.clone();
  }

  size_t frame_size = output_frame.total() * output_frame.elemSize();
  size_t written = fwrite(output_frame.data, 1, frame_size, writer_info.pipe);

  if (written != frame_size) {
    throw std::runtime_error("Failed to write frame to FFmpeg pipe");
  }

  writer_info.frame_count++;
}

ImageMetadata ImageCompressor::add_frame(
  const std::string & topic_name,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  int64_t timestamp_ns = image_msg->header.stamp.sec * 1000000000LL +
    image_msg->header.stamp.nanosec;

  cv::Mat frame = convert_ros_image_to_bgr(image_msg);

  if (!has_active_writer(topic_name)) {
    bool success = initialize_writer(topic_name, image_msg->width, image_msg->height);
    if (!success) {
      throw std::runtime_error("Failed to initialize FFmpeg writer for topic: " + topic_name);
    }
  }

  auto & writer_info = writers_[topic_name];
  write_frame_to_pipe(writer_info, frame);

  ImageMetadata metadata;
  metadata.frame_index = writer_info.frame_count - 1;
  metadata.timestamp_ns = timestamp_ns;
  metadata.width = image_msg->width;
  metadata.height = image_msg->height;
  metadata.encoding = image_msg->encoding;

  return metadata;
}

void ImageCompressor::finalize_writer(const std::string & topic_name)
{
  auto writer_it = writers_.find(topic_name);
  if (writer_it != writers_.end()) {
    std::fprintf(
      stderr,
      "[ImageCompressor] Finalizing %s: %u frames @ %.2f fps -> %s\n",
      topic_name.c_str(),
      writer_it->second.frame_count,
      fps_,
      writer_it->second.output_path.c_str());

    if (writer_it->second.pipe) {
      pclose(writer_it->second.pipe);
      writer_it->second.pipe = nullptr;
    }
    writers_.erase(writer_it);
  }
}

void ImageCompressor::finalize_all()
{
  std::vector<std::string> topics;
  for (const auto & [topic_name, _] : writers_) {
    topics.push_back(topic_name);
  }

  for (const auto & topic_name : topics) {
    finalize_writer(topic_name);
  }

  writers_.clear();
}

bool ImageCompressor::has_active_writer(const std::string & topic_name) const
{
  auto it = writers_.find(topic_name);
  return it != writers_.end() && it->second.is_initialized && it->second.pipe != nullptr;
}

}  // namespace rosbag_recorder
