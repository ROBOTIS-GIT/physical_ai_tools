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
#include <numeric>

namespace rosbag_recorder
{

ImageCompressor::ImageCompressor(
  const std::string & output_dir,
  size_t fps_detection_frames)
: output_dir_(output_dir),
  fps_detection_frames_(fps_detection_frames)
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
  // -r FPS (before -i): input frame rate
  // -i -: read from stdin
  // -r FPS (after -i): output frame rate (IMPORTANT: must match input for correct playback)
  // -c:v libx264: use H.264 encoder
  // -preset fast: encoding speed/quality tradeoff
  // -crf 23: quality (0-51, lower is better, 23 is default)
  // -pix_fmt yuv420p: output pixel format for compatibility
  // -movflags +faststart: move moov atom to beginning for web streaming

  cmd << "ffmpeg -y -f rawvideo -vcodec rawvideo "
      << "-s " << width << "x" << height << " "
      << "-pix_fmt bgr24 "
      << "-r " << std::fixed << std::setprecision(2) << fps << " "
      << "-i - "
      << "-r " << std::fixed << std::setprecision(2) << fps << " "
      << "-c:v libx264 "
      << "-preset fast "
      << "-crf 23 "
      << "-pix_fmt yuv420p "
      << "-movflags +faststart "
      << "-loglevel error "
      << "\"" << output_path << "\" 2>/dev/null";

  return cmd.str();
}

double ImageCompressor::calculate_fps_from_timestamps(
  const std::vector<int64_t> & timestamps) const
{
  if (timestamps.size() < 2) {
    return 30.0;  // Default fallback
  }

  // Calculate intervals between consecutive timestamps
  std::vector<double> intervals;
  intervals.reserve(timestamps.size() - 1);

  for (size_t i = 1; i < timestamps.size(); ++i) {
    double interval_sec = static_cast<double>(timestamps[i] - timestamps[i - 1]) / 1e9;
    if (interval_sec > 0) {
      intervals.push_back(interval_sec);
    }
  }

  if (intervals.empty()) {
    return 30.0;  // Default fallback
  }

  // Calculate median interval (more robust than mean)
  std::sort(intervals.begin(), intervals.end());
  double median_interval;
  size_t mid = intervals.size() / 2;
  if (intervals.size() % 2 == 0) {
    median_interval = (intervals[mid - 1] + intervals[mid]) / 2.0;
  } else {
    median_interval = intervals[mid];
  }

  // Convert interval to FPS
  double fps = 1.0 / median_interval;

  // Clamp to reasonable range (10-120 FPS)
  // Minimum 10 FPS to prevent extremely slow playback from timestamp issues
  fps = std::max(10.0, std::min(120.0, fps));

  return fps;
}

bool ImageCompressor::initialize_writer(
  const std::string & topic_name,
  uint32_t width,
  uint32_t height,
  double fps)
{
  if (has_active_writer(topic_name)) {
    return true;  // Already initialized
  }

  std::string sanitized_name = sanitize_topic_name(topic_name);
  std::string output_path = output_dir_ + "/" + sanitized_name + ".mp4";

  // Log the detected FPS for debugging
  std::fprintf(
    stderr,
    "[ImageCompressor] Initializing writer for %s: %ux%u @ %.2f fps -> %s\n",
    topic_name.c_str(), width, height, fps, output_path.c_str());

  // Build FFmpeg command
  std::string cmd = build_ffmpeg_command(output_path, width, height, fps);

  // Open pipe to FFmpeg process
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
  info.fps = fps;
  info.is_initialized = true;

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

void ImageCompressor::write_frame_to_pipe(FFmpegWriterInfo & writer_info, const cv::Mat & frame)
{
  cv::Mat output_frame = frame;

  // Resize if dimensions don't match
  if (static_cast<uint32_t>(frame.cols) != writer_info.width ||
    static_cast<uint32_t>(frame.rows) != writer_info.height)
  {
    cv::resize(frame, output_frame, cv::Size(writer_info.width, writer_info.height));
  }

  // Ensure frame is contiguous
  if (!output_frame.isContinuous()) {
    output_frame = output_frame.clone();
  }

  // Write raw frame data to FFmpeg pipe
  size_t frame_size = output_frame.total() * output_frame.elemSize();
  size_t written = fwrite(output_frame.data, 1, frame_size, writer_info.pipe);

  if (written != frame_size) {
    throw std::runtime_error("Failed to write frame to FFmpeg pipe");
  }

  writer_info.frame_count++;
}

void ImageCompressor::flush_buffered_frames(const std::string & topic_name)
{
  auto buffer_it = topic_buffers_.find(topic_name);
  auto writer_it = writers_.find(topic_name);

  if (buffer_it == topic_buffers_.end() || writer_it == writers_.end()) {
    return;
  }

  auto & buffer = buffer_it->second;
  auto & writer_info = writer_it->second;

  // Write all buffered frames
  for (const auto & buffered_frame : buffer.frames) {
    write_frame_to_pipe(writer_info, buffered_frame.frame);
  }

  // Clear buffer after flushing
  buffer.frames.clear();
}

ImageMetadata ImageCompressor::add_frame(
  const std::string & topic_name,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  // Get timestamp from message
  int64_t timestamp_ns = image_msg->header.stamp.sec * 1000000000LL +
    image_msg->header.stamp.nanosec;

  // Convert ROS image to BGR Mat
  cv::Mat frame = convert_ros_image_to_bgr(image_msg);

  // Check if we have an active writer
  if (has_active_writer(topic_name)) {
    // Writer already initialized, directly write frame
    auto & writer_info = writers_[topic_name];
    write_frame_to_pipe(writer_info, frame);

    // Create metadata
    ImageMetadata metadata;
    metadata.frame_index = writer_info.frame_count - 1;  // Already incremented in write_frame
    metadata.timestamp_ns = timestamp_ns;
    metadata.width = image_msg->width;
    metadata.height = image_msg->height;
    metadata.encoding = image_msg->encoding;

    return metadata;
  }

  // Initialize buffer for this topic if not exists
  if (topic_buffers_.find(topic_name) == topic_buffers_.end()) {
    TopicBufferInfo buffer_info;
    buffer_info.detected_fps = 0.0;
    buffer_info.fps_detected = false;
    topic_buffers_[topic_name] = buffer_info;
  }

  auto & buffer = topic_buffers_[topic_name];

  // Add frame to buffer
  BufferedFrame buffered_frame;
  buffered_frame.frame = frame.clone();
  buffered_frame.timestamp_ns = timestamp_ns;
  buffered_frame.width = image_msg->width;
  buffered_frame.height = image_msg->height;
  buffered_frame.encoding = image_msg->encoding;
  buffer.frames.push_back(buffered_frame);
  buffer.timestamps.push_back(timestamp_ns);

  // Check if we have enough frames for FPS detection
  if (!buffer.fps_detected && buffer.timestamps.size() >= fps_detection_frames_) {
    // Calculate FPS from timestamps
    buffer.detected_fps = calculate_fps_from_timestamps(buffer.timestamps);
    buffer.fps_detected = true;

    // Initialize writer with detected FPS
    bool success = initialize_writer(
      topic_name,
      image_msg->width,
      image_msg->height,
      buffer.detected_fps);

    if (!success) {
      throw std::runtime_error(
              "Failed to initialize FFmpeg writer for topic: " + topic_name);
    }

    // Flush all buffered frames
    flush_buffered_frames(topic_name);
  }

  // Create metadata (frame index is position in buffer or actual frame count)
  ImageMetadata metadata;
  if (has_active_writer(topic_name)) {
    metadata.frame_index = writers_[topic_name].frame_count - 1;
  } else {
    metadata.frame_index = buffer.frames.size() - 1;
  }
  metadata.timestamp_ns = timestamp_ns;
  metadata.width = image_msg->width;
  metadata.height = image_msg->height;
  metadata.encoding = image_msg->encoding;

  return metadata;
}

void ImageCompressor::finalize_writer(const std::string & topic_name)
{
  // First flush any remaining buffered frames
  auto buffer_it = topic_buffers_.find(topic_name);
  if (buffer_it != topic_buffers_.end()) {
    auto & buffer = buffer_it->second;

    // If we have buffered frames but writer not initialized yet,
    // initialize with default FPS and flush
    if (!buffer.frames.empty() && !has_active_writer(topic_name)) {
      double fps = buffer.fps_detected ? buffer.detected_fps : 30.0;
      if (!buffer.frames.empty()) {
        initialize_writer(
          topic_name,
          buffer.frames.front().width,
          buffer.frames.front().height,
          fps);
        flush_buffered_frames(topic_name);
      }
    }

    topic_buffers_.erase(buffer_it);
  }

  // Close writer
  auto writer_it = writers_.find(topic_name);
  if (writer_it != writers_.end()) {
    // Log finalization info
    std::fprintf(
      stderr,
      "[ImageCompressor] Finalizing %s: %u frames @ %.2f fps -> %s\n",
      topic_name.c_str(),
      writer_it->second.frame_count,
      writer_it->second.fps,
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
  // Flush and close all writers
  std::vector<std::string> topics;
  for (const auto & [topic_name, _] : topic_buffers_) {
    topics.push_back(topic_name);
  }
  for (const auto & [topic_name, _] : writers_) {
    if (std::find(topics.begin(), topics.end(), topic_name) == topics.end()) {
      topics.push_back(topic_name);
    }
  }

  for (const auto & topic_name : topics) {
    finalize_writer(topic_name);
  }

  topic_buffers_.clear();
  writers_.clear();
}

bool ImageCompressor::has_active_writer(const std::string & topic_name) const
{
  auto it = writers_.find(topic_name);
  return it != writers_.end() && it->second.is_initialized && it->second.pipe != nullptr;
}

bool ImageCompressor::is_tracking(const std::string & topic_name) const
{
  return has_active_writer(topic_name) ||
         topic_buffers_.find(topic_name) != topic_buffers_.end();
}

double ImageCompressor::get_detected_fps(const std::string & topic_name) const
{
  // Check writer first
  auto writer_it = writers_.find(topic_name);
  if (writer_it != writers_.end() && writer_it->second.is_initialized) {
    return writer_it->second.fps;
  }

  // Check buffer
  auto buffer_it = topic_buffers_.find(topic_name);
  if (buffer_it != topic_buffers_.end() && buffer_it->second.fps_detected) {
    return buffer_it->second.detected_fps;
  }

  return 0.0;  // Not yet detected
}

}  // namespace rosbag_recorder
