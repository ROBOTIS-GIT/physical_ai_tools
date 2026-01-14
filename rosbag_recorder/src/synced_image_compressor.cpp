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
#include <fstream>
#include <iomanip>
#include <sstream>

namespace rosbag_recorder
{

SyncedImageCompressor::SyncedImageCompressor(
  const std::string & output_dir,
  const std::vector<std::string> & topics)
: SyncedImageCompressor(output_dir, topics, Config{})
{
}

SyncedImageCompressor::SyncedImageCompressor(
  const std::string & output_dir,
  const std::vector<std::string> & topics,
  const Config & config)
: config_(config),
  output_dir_(output_dir),
  topics_(topics)
{
  std::filesystem::create_directories(output_dir_);

  frame_duration_ns_ = static_cast<int64_t>(1e9 / config_.target_fps);
  frames_per_chunk_ = static_cast<uint32_t>(config_.chunk_duration_sec * config_.target_fps);

  for (const auto & topic : topics_) {
    topic_data_[topic] = std::make_unique<TopicData>();
    stats_.topic_stats[topic] = RecordingStats::TopicStats();
  }

  if (config_.enable_background_encoding) {
    encoding_active_ = true;
    encoding_thread_ = std::thread(&SyncedImageCompressor::encoding_thread_func, this);
  }
}

SyncedImageCompressor::~SyncedImageCompressor()
{
  stop_recording();

  encoding_active_ = false;
  encoding_cv_.notify_all();

  if (encoding_thread_.joinable()) {
    encoding_thread_.join();
  }
}

void SyncedImageCompressor::set_synced_frame_callback(SyncedFrameCallback callback)
{
  synced_frame_callback_ = std::move(callback);
}

void SyncedImageCompressor::add_incoming_frame(
  const std::string & topic,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  auto it = topic_data_.find(topic);
  if (it == topic_data_.end()) {
    return;
  }

  auto & data = it->second;
  std::lock_guard<std::mutex> lock(data->mutex);

  int64_t timestamp_ns = image_msg->header.stamp.sec * 1000000000LL +
    image_msg->header.stamp.nanosec;

  data->latest_frame.frame = convert_ros_image_to_bgr(image_msg);
  data->latest_frame.timestamp_ns = timestamp_ns;
  data->latest_frame.width = image_msg->width;
  data->latest_frame.height = image_msg->height;
  data->latest_frame.encoding = image_msg->encoding;
  data->has_data = true;

  {
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.topic_stats[topic].received_frames++;
  }

  if (state_ == State::INITIALIZING) {
    try_initialize();
  }
}

void SyncedImageCompressor::start_recording()
{
  if (state_ != State::IDLE) {
    return;
  }

  state_ = State::INITIALIZING;

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.recording_start_ns = get_current_time_ns();
  }

  std::fprintf(stderr, "[SyncedImageCompressor] Recording started, waiting for initialization...\n");
}

void SyncedImageCompressor::stop_recording()
{
  if (state_ != State::RECORDING && state_ != State::INITIALIZING) {
    return;
  }

  state_ = State::STOPPING;

  sampling_active_ = false;
  sampling_cv_.notify_all();

  if (sampling_thread_.joinable()) {
    sampling_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.recording_end_ns = get_current_time_ns();
  }

  finalize_all_chunks();

  state_ = State::IDLE;

  std::fprintf(
    stderr,
    "[SyncedImageCompressor] Recording stopped. Total frames: %u\n",
    stats_.total_synced_frames);
}

bool SyncedImageCompressor::is_recording() const
{
  return state_ == State::RECORDING;
}

bool SyncedImageCompressor::is_initialized() const
{
  return initialized_;
}

RecordingStats SyncedImageCompressor::get_stats() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

double SyncedImageCompressor::get_encoding_time_remaining() const
{
  uint32_t pending = pending_chunks_.load();
  return pending * config_.chunk_duration_sec * 0.5;
}

void SyncedImageCompressor::wait_for_encoding_complete()
{
  while (pending_chunks_.load() > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void SyncedImageCompressor::try_initialize()
{
  bool all_have_data = true;
  int64_t max_timestamp = 0;
  int64_t min_timestamp = INT64_MAX;

  for (const auto & topic : topics_) {
    auto & data = topic_data_[topic];
    if (!data->has_data) {
      all_have_data = false;
      break;
    }

    max_timestamp = std::max(max_timestamp, data->latest_frame.timestamp_ns);
    min_timestamp = std::min(min_timestamp, data->latest_frame.timestamp_ns);
  }

  if (!all_have_data) {
    return;
  }

  int64_t time_diff_ns = max_timestamp - min_timestamp;
  double time_diff_sec = time_diff_ns / 1e9;

  if (time_diff_sec > config_.init_window_sec) {
    return;
  }

  anchor_timestamp_ns_ = max_timestamp;
  frame_index_ = 0;
  initialized_ = true;
  state_ = State::RECORDING;

  std::fprintf(
    stderr,
    "[SyncedImageCompressor] Initialized. Anchor: %.3f sec, Max diff: %.3f ms\n",
    anchor_timestamp_ns_ / 1e9,
    time_diff_sec * 1000);

  sampling_active_ = true;
  sampling_thread_ = std::thread([this]() {
      auto next_sample_time = std::chrono::steady_clock::now();

      while (sampling_active_) {
        sample_synced_frame();

        next_sample_time += std::chrono::nanoseconds(frame_duration_ns_);
        std::this_thread::sleep_until(next_sample_time);
      }
    });
}

void SyncedImageCompressor::sample_synced_frame()
{
  if (state_ != State::RECORDING) {
    return;
  }

  int64_t target_time = anchor_timestamp_ns_ + frame_index_ * frame_duration_ns_;

  SyncedFrameOutput output;
  output.frame_index = frame_index_;
  output.anchor_timestamp_ns = target_time;

  bool all_valid = true;

  for (const auto & topic : topics_) {
    auto & data = topic_data_[topic];
    std::lock_guard<std::mutex> lock(data->mutex);

    if (!data->has_data) {
      all_valid = false;
      break;
    }

    if (data->latest_frame.timestamp_ns > target_time) {
      continue;
    }

    int64_t staleness_ns = target_time - data->latest_frame.timestamp_ns;

    TimestampedFrame synced_frame;
    synced_frame.frame = data->latest_frame.frame.clone();
    synced_frame.timestamp_ns = data->latest_frame.timestamp_ns;
    synced_frame.width = data->latest_frame.width;
    synced_frame.height = data->latest_frame.height;
    synced_frame.encoding = data->latest_frame.encoding;

    output.frames[topic] = synced_frame;
    output.staleness_ns[topic] = staleness_ns;

    data->frame_buffer.push_back(synced_frame);

    {
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      auto & topic_stat = stats_.topic_stats[topic];
      topic_stat.synced_frames++;
      topic_stat.total_staleness_ns += staleness_ns;
      topic_stat.max_staleness_ns = std::max(topic_stat.max_staleness_ns, staleness_ns);
      topic_stat.min_staleness_ns = std::min(topic_stat.min_staleness_ns, staleness_ns);
    }
  }

  if (output.frames.size() == topics_.size()) {
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.total_synced_frames++;
    }

    if (synced_frame_callback_) {
      synced_frame_callback_(output);
    }

    frame_index_++;

    if (frame_index_ > 0 && frame_index_ % frames_per_chunk_ == 0) {
      uint32_t chunk_idx = (frame_index_ / frames_per_chunk_) - 1;
      trigger_chunk_encoding(chunk_idx);
    }

    update_ram_usage();
  }
}

void SyncedImageCompressor::trigger_chunk_encoding(uint32_t chunk_index)
{
  std::fprintf(
    stderr,
    "[SyncedImageCompressor] Triggering encoding for chunk %u\n",
    chunk_index);

  for (const auto & topic : topics_) {
    auto & data = topic_data_[topic];
    std::lock_guard<std::mutex> lock(data->mutex);

    ChunkTask task;
    task.topic = topic;
    task.chunk_index = chunk_index;
    task.output_path = output_dir_ + "/" + sanitize_topic_name(topic) +
      "_chunk" + std::to_string(chunk_index) + ".mp4";

    size_t frames_to_encode = std::min(
      static_cast<size_t>(frames_per_chunk_),
      data->frame_buffer.size());

    task.frames.reserve(frames_to_encode);
    for (size_t i = 0; i < frames_to_encode; ++i) {
      task.frames.push_back(std::move(data->frame_buffer.front().frame));
      data->frame_buffer.pop_front();
    }

    {
      std::lock_guard<std::mutex> enc_lock(encoding_mutex_);
      encoding_queue_.push_back(std::move(task));
      pending_chunks_++;
    }
    encoding_cv_.notify_one();
  }
}

void SyncedImageCompressor::encoding_thread_func()
{
  while (encoding_active_) {
    ChunkTask task;

    {
      std::unique_lock<std::mutex> lock(encoding_mutex_);
      encoding_cv_.wait(lock, [this]() {
          return !encoding_queue_.empty() || !encoding_active_;
        });

      if (!encoding_active_ && encoding_queue_.empty()) {
        break;
      }

      if (!encoding_queue_.empty()) {
        task = std::move(encoding_queue_.front());
        encoding_queue_.pop_front();
      }
    }

    if (!task.frames.empty()) {
      encode_chunk(task);
      pending_chunks_--;
    }
  }
}

void SyncedImageCompressor::encode_chunk(const ChunkTask & task)
{
  if (task.frames.empty()) {
    return;
  }

  uint32_t width = task.frames[0].cols;
  uint32_t height = task.frames[0].rows;

  std::string cmd = build_ffmpeg_command(
    task.output_path, width, height, config_.target_fps);

  FILE * pipe = popen(cmd.c_str(), "w");
  if (!pipe) {
    std::fprintf(
      stderr,
      "[SyncedImageCompressor] Failed to open FFmpeg pipe for %s\n",
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
    "[SyncedImageCompressor] Encoded chunk %u for %s: %zu frames -> %s\n",
    task.chunk_index,
    task.topic.c_str(),
    task.frames.size(),
    task.output_path.c_str());
}

void SyncedImageCompressor::finalize_all_chunks()
{
  for (const auto & topic : topics_) {
    auto & data = topic_data_[topic];
    std::lock_guard<std::mutex> lock(data->mutex);

    if (data->frame_buffer.empty()) {
      continue;
    }

    uint32_t chunk_index = frame_index_ / frames_per_chunk_;

    ChunkTask task;
    task.topic = topic;
    task.chunk_index = chunk_index;
    task.output_path = output_dir_ + "/" + sanitize_topic_name(topic) +
      "_chunk" + std::to_string(chunk_index) + ".mp4";

    task.frames.reserve(data->frame_buffer.size());
    while (!data->frame_buffer.empty()) {
      task.frames.push_back(std::move(data->frame_buffer.front().frame));
      data->frame_buffer.pop_front();
    }

    {
      std::lock_guard<std::mutex> enc_lock(encoding_mutex_);
      encoding_queue_.push_back(std::move(task));
      pending_chunks_++;
    }
    encoding_cv_.notify_one();
  }

  wait_for_encoding_complete();

  concat_chunks();
}

void SyncedImageCompressor::concat_chunks()
{
  for (const auto & topic : topics_) {
    std::string base_name = sanitize_topic_name(topic);
    std::string final_output = output_dir_ + "/" + base_name + ".mp4";
    std::string concat_list_path = output_dir_ + "/" + base_name + "_concat.txt";

    std::vector<std::string> chunk_files;
    for (uint32_t i = 0; ; ++i) {
      std::string chunk_path = output_dir_ + "/" + base_name + "_chunk" + std::to_string(i) + ".mp4";
      if (std::filesystem::exists(chunk_path)) {
        chunk_files.push_back(chunk_path);
      } else {
        break;
      }
    }

    if (chunk_files.empty()) {
      continue;
    }

    if (chunk_files.size() == 1) {
      std::filesystem::rename(chunk_files[0], final_output);
      continue;
    }

    std::ofstream concat_list(concat_list_path);
    for (const auto & chunk_file : chunk_files) {
      concat_list << "file '" << chunk_file << "'\n";
    }
    concat_list.close();

    std::ostringstream cmd;
    cmd << "ffmpeg -y -f concat -safe 0 -i \"" << concat_list_path << "\" "
        << "-c copy \"" << final_output << "\" 2>/dev/null";

    int result = system(cmd.str().c_str());
    if (result == 0) {
      for (const auto & chunk_file : chunk_files) {
        std::filesystem::remove(chunk_file);
      }
      std::filesystem::remove(concat_list_path);

      std::fprintf(
        stderr,
        "[SyncedImageCompressor] Concatenated %zu chunks for %s -> %s\n",
        chunk_files.size(),
        topic.c_str(),
        final_output.c_str());
    }
  }
}

cv::Mat SyncedImageCompressor::convert_ros_image_to_bgr(
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

std::string SyncedImageCompressor::build_ffmpeg_command(
  const std::string & output_path,
  uint32_t width,
  uint32_t height,
  double fps)
{
  std::ostringstream cmd;

  cmd << "ffmpeg -y -f rawvideo -vcodec rawvideo "
      << "-s " << width << "x" << height << " "
      << "-pix_fmt bgr24 "
      << "-r " << std::fixed << std::setprecision(2) << fps << " "
      << "-i - "
      << "-r " << std::fixed << std::setprecision(2) << fps << " "
      << "-c:v libx264 "
      << "-preset " << config_.ffmpeg_preset << " "
      << "-crf " << config_.ffmpeg_crf << " "
      << "-pix_fmt yuv420p "
      << "-movflags +faststart "
      << "-loglevel error "
      << "\"" << output_path << "\" 2>/dev/null";

  return cmd.str();
}

std::string SyncedImageCompressor::sanitize_topic_name(const std::string & topic_name) const
{
  std::string sanitized = topic_name;
  std::replace(sanitized.begin(), sanitized.end(), '/', '_');
  if (!sanitized.empty() && sanitized[0] == '_') {
    sanitized = sanitized.substr(1);
  }
  return sanitized;
}

void SyncedImageCompressor::update_ram_usage()
{
  size_t total_bytes = 0;

  for (const auto & topic : topics_) {
    auto & data = topic_data_[topic];
    for (const auto & frame : data->frame_buffer) {
      total_bytes += frame.frame.total() * frame.frame.elemSize();
    }
  }

  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.current_ram_usage_bytes = total_bytes;
  stats_.peak_ram_usage_bytes = std::max(stats_.peak_ram_usage_bytes, total_bytes);
}

int64_t SyncedImageCompressor::get_current_time_ns() const
{
  auto now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();
}

}  // namespace rosbag_recorder
