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

#include "rosbag_recorder/synced_image_compressor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace rosbag_recorder
{

ImageCompressorRaw::ImageCompressorRaw(
  const std::string & output_dir,
  const std::vector<std::string> & topics)
: ImageCompressorRaw(output_dir, topics, Config{})
{
}

ImageCompressorRaw::ImageCompressorRaw(
  const std::string & output_dir,
  const std::vector<std::string> & topics,
  const Config & config)
: config_(config),
  output_dir_(output_dir),
  topics_(topics)
{
  std::filesystem::create_directories(output_dir_);

  for (const auto & topic : topics_) {
    topic_data_[topic] = std::make_unique<TopicData>();
    stats_.topic_stats[topic] = RecordingStats::TopicStats();
  }
}

ImageCompressorRaw::~ImageCompressorRaw()
{
  if (encoding_thread_.joinable()) {
    encoding_thread_.join();
  }
}

void ImageCompressorRaw::set_frame_callback(FrameCallback callback)
{
  frame_callback_ = std::move(callback);
}

void ImageCompressorRaw::set_encoding_complete_callback(EncodingCompleteCallback callback)
{
  encoding_complete_callback_ = std::move(callback);
}

void ImageCompressorRaw::add_frame(
  const std::string & topic,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  if (state_ != CompressorState::RECORDING) {
    return;
  }

  auto it = topic_data_.find(topic);
  if (it == topic_data_.end()) {
    return;
  }

  auto & data = it->second;
  std::lock_guard<std::mutex> lock(data->mutex);

  int64_t timestamp_ns = image_msg->header.stamp.sec * 1000000000LL +
    image_msg->header.stamp.nanosec;

  TimestampedFrame frame;
  frame.frame = convert_ros_image_to_bgr(image_msg);
  frame.timestamp_ns = timestamp_ns;
  frame.width = image_msg->width;
  frame.height = image_msg->height;

  data->frame_buffer.push_back(std::move(frame));

  if (frame_callback_) {
    FrameOutput output;
    output.topic = topic;
    output.frame_index = data->frame_count;
    output.timestamp_ns = timestamp_ns;
    output.width = image_msg->width;
    output.height = image_msg->height;
    frame_callback_(output);
  }

  data->frame_count++;

  {
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.topic_stats[topic].received_frames++;
    stats_.total_frames++;
  }

  update_ram_usage();
}

void ImageCompressorRaw::start_recording()
{
  if (state_ != CompressorState::IDLE) {
    return;
  }

  if (encoding_thread_.joinable()) {
    encoding_thread_.join();
  }

  for (auto & [topic, data] : topic_data_) {
    std::lock_guard<std::mutex> lock(data->mutex);
    data->frame_buffer.clear();
    data->frame_count = 0;
  }

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_ = RecordingStats();
    for (const auto & topic : topics_) {
      stats_.topic_stats[topic] = RecordingStats::TopicStats();
    }
    stats_.recording_start_ns = get_current_time_ns();
  }

  state_ = CompressorState::RECORDING;

  std::fprintf(stderr, "[ImageCompressorRaw] Recording started\n");
}

void ImageCompressorRaw::stop_recording()
{
  if (state_ != CompressorState::RECORDING) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.recording_end_ns = get_current_time_ns();
  }

  state_ = CompressorState::ENCODING;

  std::fprintf(stderr, "[ImageCompressorRaw] Recording stopped. Starting encoding thread...\n");

  encoding_thread_ = std::thread(&ImageCompressorRaw::encoding_thread_func, this);
}

CompressorState ImageCompressorRaw::get_state() const
{
  return state_.load();
}

bool ImageCompressorRaw::is_recording() const
{
  return state_ == CompressorState::RECORDING;
}

bool ImageCompressorRaw::is_encoding() const
{
  return state_ == CompressorState::ENCODING;
}

RecordingStats ImageCompressorRaw::get_stats() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

void ImageCompressorRaw::encoding_thread_func()
{
  std::fprintf(stderr, "[ImageCompressorRaw] Encoding thread started\n");

  bool success = true;
  std::string message = "Encoding complete";

  try {
    encode_all_videos();
  } catch (const std::exception & e) {
    success = false;
    message = std::string("Encoding failed: ") + e.what();
    std::fprintf(stderr, "[ImageCompressorRaw] %s\n", message.c_str());
  }

  state_ = CompressorState::IDLE;

  std::fprintf(
    stderr,
    "[ImageCompressorRaw] Encoding complete. Total frames: %u\n",
    stats_.total_frames);

  if (encoding_complete_callback_) {
    encoding_complete_callback_(success, message);
  }
}

void ImageCompressorRaw::encode_all_videos()
{
  for (const auto & topic : topics_) {
    auto & data = topic_data_[topic];
    std::lock_guard<std::mutex> lock(data->mutex);

    if (data->frame_buffer.empty()) {
      continue;
    }

    EncodeTask task;
    task.topic = topic;
    task.output_path = output_dir_ + "/" + sanitize_topic_name(topic) + ".mp4";

    task.frames.reserve(data->frame_buffer.size());
    while (!data->frame_buffer.empty()) {
      task.frames.push_back(std::move(data->frame_buffer.front().frame));
      data->frame_buffer.pop_front();
    }

    std::fprintf(
      stderr,
      "[ImageCompressorRaw] Encoding %zu frames for %s @ %.2f fps\n",
      task.frames.size(),
      topic.c_str(),
      config_.target_fps);

    encode_video(task);
  }
}

void ImageCompressorRaw::encode_video(const EncodeTask & task)
{
  if (task.frames.empty()) {
    return;
  }

  uint32_t width = task.frames[0].cols;
  uint32_t height = task.frames[0].rows;

  std::string cmd = build_ffmpeg_command(task.output_path, width, height);

  FILE * pipe = popen(cmd.c_str(), "w");
  if (!pipe) {
    std::fprintf(
      stderr,
      "[ImageCompressorRaw] Failed to open FFmpeg pipe for %s\n",
      task.topic.c_str());
    return;
  }

  for (const auto & frame : task.frames) {
    cv::Mat output_frame = frame;
    if (!output_frame.isContinuous()) {
      output_frame = output_frame.clone();
    }

    size_t frame_size = output_frame.total() * output_frame.elemSize();
    fwrite(output_frame.data, 1, frame_size, pipe);
  }

  pclose(pipe);

  std::fprintf(
    stderr,
    "[ImageCompressorRaw] Encoded %s: %zu frames -> %s\n",
    task.topic.c_str(),
    task.frames.size(),
    task.output_path.c_str());
}

cv::Mat ImageCompressorRaw::convert_ros_image_to_bgr(
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

std::string ImageCompressorRaw::build_ffmpeg_command(
  const std::string & output_path,
  uint32_t width,
  uint32_t height)
{
  std::ostringstream cmd;

  cmd << "ffmpeg -y -f rawvideo -vcodec rawvideo "
      << "-s " << width << "x" << height << " "
      << "-pix_fmt bgr24 "
      << "-r " << std::fixed << std::setprecision(2) << config_.target_fps << " "
      << "-i - "
      << "-r " << std::fixed << std::setprecision(2) << config_.target_fps << " "
      << "-c:v libx264 "
      << "-preset " << config_.ffmpeg_preset << " "
      << "-crf " << config_.ffmpeg_crf << " "
      << "-pix_fmt yuv420p "
      << "-movflags +faststart "
      << "-loglevel error "
      << "\"" << output_path << "\" 2>/dev/null";

  return cmd.str();
}

std::string ImageCompressorRaw::sanitize_topic_name(const std::string & topic_name) const
{
  std::string sanitized = topic_name;
  std::replace(sanitized.begin(), sanitized.end(), '/', '_');
  if (!sanitized.empty() && sanitized[0] == '_') {
    sanitized = sanitized.substr(1);
  }
  return sanitized;
}

void ImageCompressorRaw::update_ram_usage()
{
  size_t total_bytes = 0;

  for (const auto & [topic, data] : topic_data_) {
    for (const auto & frame : data->frame_buffer) {
      total_bytes += frame.frame.total() * frame.frame.elemSize();
    }
  }

  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.current_ram_usage_bytes = total_bytes;
  stats_.peak_ram_usage_bytes = std::max(stats_.peak_ram_usage_bytes, total_bytes);

  if (total_bytes > config_.ram_limit_bytes * 0.8) {
    std::fprintf(
      stderr,
      "[ImageCompressorRaw] WARNING: RAM usage at %.1f%% (%.2f GB / %.2f GB limit)\n",
      (total_bytes * 100.0) / config_.ram_limit_bytes,
      total_bytes / (1024.0 * 1024.0 * 1024.0),
      config_.ram_limit_bytes / (1024.0 * 1024.0 * 1024.0));
  }
}

int64_t ImageCompressorRaw::get_current_time_ns() const
{
  auto now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();
}

}  // namespace rosbag_recorder
