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

#include "rosbag_recorder/synced_image_bag_recorder.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag_recorder
{

SyncedImageBagRecorder::SyncedImageBagRecorder()
: rclcpp::Node("synced_image_bag_recorder")
{
  RCLCPP_INFO(this->get_logger(), "Starting synced image bag recorder node");

  send_command_srv_ = this->create_service<rosbag_recorder::srv::SendCommand>(
    "rosbag_recorder/send_command",
    std::bind(
      &SyncedImageBagRecorder::handle_send_command, this,
      std::placeholders::_1, std::placeholders::_2));
}

void SyncedImageBagRecorder::handle_send_command(
  const std::shared_ptr<rosbag_recorder::srv::SendCommand::Request> req,
  std::shared_ptr<rosbag_recorder::srv::SendCommand::Response> res)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  RCLCPP_INFO(this->get_logger(), "Received command: %d", req->command);

  try {
    switch (req->command) {
      case rosbag_recorder::srv::SendCommand::Request::PREPARE:
        handle_prepare(req->topics);
        res->success = true;
        res->message = "Recording prepared";
        break;
      case rosbag_recorder::srv::SendCommand::Request::START:
        handle_start(req->uri);
        res->success = true;
        res->message = "Recording started";
        break;
      case rosbag_recorder::srv::SendCommand::Request::STOP:
        handle_stop();
        res->success = true;
        res->message = "Recording stopped";
        break;
      case rosbag_recorder::srv::SendCommand::Request::STOP_AND_DELETE:
        handle_stop_and_delete();
        res->success = true;
        res->message = "Recording stopped and bag deleted";
        break;
      case rosbag_recorder::srv::SendCommand::Request::FINISH:
        handle_finish();
        res->success = true;
        res->message = "Recording finished";
        break;
      default:
        res->success = false;
        res->message = "Invalid command";
        RCLCPP_ERROR(this->get_logger(), "Invalid command: %d", req->command);
        break;
    }
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Failed to execute command: %s", e.what());
  }
}

bool SyncedImageBagRecorder::is_image_topic(const std::string & topic_type) const
{
  return topic_type == "sensor_msgs/msg/Image";
}

void SyncedImageBagRecorder::handle_prepare(const std::vector<std::string> & topics)
{
  RCLCPP_INFO(this->get_logger(), "Prepare Rosbag recording");

  if (is_recording_) {
    throw std::runtime_error("Already recording");
  }

  if (topics.empty()) {
    throw std::runtime_error("Topics are required");
  }

  try {
    topics_to_record_ = topics;
    image_topics_.clear();
    non_image_topics_.clear();

    auto names_and_types = this->get_topic_names_and_types();

    for (const auto & topic : topics_to_record_) {
      auto it = names_and_types.find(topic);
      if (it == names_and_types.end()) {
        continue;
      }

      const std::string & type = it->second.front();
      type_for_topic_[topic] = type;

      if (is_image_topic(type)) {
        image_topics_.push_back(topic);
      } else {
        non_image_topics_.push_back(topic);
      }
    }

    create_subscriptions();

    RCLCPP_INFO(
      this->get_logger(),
      "Recording prepared: topics=%zu (image=%zu, non-image=%zu)",
      topics_to_record_.size(), image_topics_.size(), non_image_topics_.size());
  } catch (const std::exception & e) {
    writer_.reset();
    throw std::runtime_error(
            std::string("Failed to prepare recording: ") + e.what());
  }
}

void SyncedImageBagRecorder::handle_start(const std::string & uri)
{
  RCLCPP_INFO(this->get_logger(), "Start Rosbag recording");

  if (is_recording_) {
    throw std::runtime_error("Already recording");
  }

  if (uri.empty()) {
    throw std::runtime_error("Bag URI is required");
  }

  try {
    current_bag_uri_ = uri;
    delete_bag_directory(current_bag_uri_);

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(current_bag_uri_);

    std::string video_output_dir = current_bag_uri_ + "/videos";

    SyncedImageCompressor::Config config;
    config.target_fps = 30.0;
    config.init_window_sec = 0.15;
    config.chunk_duration_sec = 30.0;
    config.enable_background_encoding = true;

    synced_compressor_ = std::make_unique<SyncedImageCompressor>(
      video_output_dir, image_topics_, config);

    synced_compressor_->set_synced_frame_callback(
      std::bind(&SyncedImageBagRecorder::on_synced_frame, this, std::placeholders::_1));

    auto names_and_types = this->get_topic_names_and_types();
    auto missing_topics = get_missing_topics(names_and_types);

    if (!missing_topics.empty()) {
      writer_.reset();
      synced_compressor_.reset();
      type_for_topic_.clear();

      delete_bag_directory(current_bag_uri_);
      current_bag_uri_.clear();

      std::ostringstream oss;
      oss << "Types not found for topics:";
      for (const auto & t : missing_topics) {
        oss << " " << t;
      }

      RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", oss.str().c_str());
      throw std::runtime_error(oss.str());
    }

    create_topics_in_bag(names_and_types);

    frame_counts_.clear();
    for (const auto & topic : image_topics_) {
      frame_counts_[topic] = 0;
    }

    synced_compressor_->start_recording();

  } catch (const std::exception & e) {
    throw std::runtime_error(
            std::string("Failed to start recording: ") + e.what());
  }

  is_recording_ = true;

  RCLCPP_INFO(
    this->get_logger(), "Recording started: uri=%s topics=%zu",
    current_bag_uri_.c_str(), topics_to_record_.size());
}

void SyncedImageBagRecorder::handle_stop()
{
  RCLCPP_INFO(this->get_logger(), "Stop Rosbag recording");

  if (!is_recording_) {
    throw std::runtime_error("Not recording");
  }

  try {
    if (synced_compressor_) {
      synced_compressor_->stop_recording();

      RCLCPP_INFO(this->get_logger(), "Waiting for encoding to complete...");
      synced_compressor_->wait_for_encoding_complete();

      write_stats_report();

      synced_compressor_.reset();
    }

    writer_.reset();
    type_for_topic_.clear();
    current_bag_uri_.clear();
    is_recording_ = false;

    RCLCPP_INFO(this->get_logger(), "Recording stopped");
  } catch (const std::exception & e) {
    throw std::runtime_error(
            std::string("Failed to stop recording: ") + e.what());
  }
}

void SyncedImageBagRecorder::handle_stop_and_delete()
{
  RCLCPP_INFO(this->get_logger(), "Stop and delete Rosbag recording");

  if (!is_recording_) {
    throw std::runtime_error("Not recording");
  }

  try {
    is_recording_ = false;

    if (synced_compressor_) {
      synced_compressor_->stop_recording();
      synced_compressor_.reset();
    }

    writer_.reset();
    type_for_topic_.clear();

    delete_bag_directory(current_bag_uri_);

    current_bag_uri_.clear();

    RCLCPP_INFO(this->get_logger(), "Recording stopped and bag deleted");
  } catch (const std::exception & e) {
    throw std::runtime_error(
            std::string("Failed to stop recording and delete bag: ") + e.what());
  }
}

void SyncedImageBagRecorder::handle_finish()
{
  RCLCPP_INFO(this->get_logger(), "Finish Rosbag recording");

  generic_subscriptions_.clear();
  image_subscriptions_.clear();

  if (is_recording_) {
    handle_stop();
  }
}

std::vector<std::string> SyncedImageBagRecorder::get_missing_topics(
  const std::map<std::string, std::vector<std::string>> & names_and_types)
{
  std::vector<std::string> missing_topics;

  for (const auto & topic : topics_to_record_) {
    auto it = names_and_types.find(topic);

    if (it == names_and_types.end() || it->second.empty()) {
      missing_topics.push_back(topic);
      continue;
    }
  }
  return missing_topics;
}

void SyncedImageBagRecorder::create_topics_in_bag(
  const std::map<std::string, std::vector<std::string>> & names_and_types)
{
  if (!writer_) {
    RCLCPP_ERROR(this->get_logger(), "Writer not initialized");
    return;
  }

  if (topics_to_record_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No topics to record");
    return;
  }

  for (const auto & topic : topics_to_record_) {
    auto it = names_and_types.find(topic);
    const std::string & type = it->second.front();

    rosbag2_storage::TopicMetadata meta;

    if (is_image_topic(type)) {
      meta.name = topic + "/metadata";
      meta.type = "rosbag_recorder/msg/ImageMetadata";
    } else {
      meta.name = topic;
      meta.type = type;
      type_for_topic_[topic] = type;
    }

    meta.serialization_format = rmw_get_serialization_format();
    writer_->create_topic(meta);
  }
}

void SyncedImageBagRecorder::delete_bag_directory(const std::string & bag_uri)
{
  if (bag_uri.empty()) {
    return;
  }

  std::filesystem::path bag_path(bag_uri);
  if (std::filesystem::exists(bag_path)) {
    std::filesystem::remove_all(bag_path);
    RCLCPP_INFO(this->get_logger(), "Deleted bag directory: %s", bag_uri.c_str());
  }
}

void SyncedImageBagRecorder::create_subscriptions()
{
  RCLCPP_INFO(this->get_logger(), "Creating subscriptions");

  generic_subscriptions_.clear();
  image_subscriptions_.clear();

  for (const auto & topic : non_image_topics_) {
    auto it = type_for_topic_.find(topic);
    if (it == type_for_topic_.end()) {
      continue;
    }

    const std::string & type = it->second;
    auto options = rclcpp::SubscriptionOptions();
    auto sub = this->create_generic_subscription(
      topic,
      type,
      rclcpp::QoS(100),
      [this, topic](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
        this->handle_serialized_message(topic, serialized_msg);
      },
      options);
    generic_subscriptions_.push_back(sub);
  }

  for (const auto & topic : image_topics_) {
    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
      topic,
      rclcpp::QoS(100),
      [this, topic](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->handle_image_message(topic, msg);
      });
    image_subscriptions_.push_back(sub);
  }
}

void SyncedImageBagRecorder::handle_serialized_message(
  const std::string & topic,
  const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  if (!is_recording_ || !writer_) {
    return;
  }

  const auto it = type_for_topic_.find(topic);
  if (it == type_for_topic_.end()) {
    return;
  }

  const std::string & type = it->second;
  writer_->write(serialized_msg, topic, type, this->now());
}

void SyncedImageBagRecorder::handle_image_message(
  const std::string & topic,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  if (!is_recording_ || !synced_compressor_) {
    return;
  }

  synced_compressor_->add_incoming_frame(topic, image_msg);
}

void SyncedImageBagRecorder::on_synced_frame(const SyncedFrameOutput & output)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  if (!is_recording_ || !writer_) {
    return;
  }

  for (const auto & [topic, frame] : output.frames) {
    rosbag_recorder::msg::ImageMetadata metadata_msg;

    rclcpp::Time timestamp(frame.timestamp_ns);
    metadata_msg.header.stamp = timestamp;
    metadata_msg.header.frame_id = topic;

    metadata_msg.frame_index = frame_counts_[topic]++;
    metadata_msg.width = frame.width;
    metadata_msg.height = frame.height;
    metadata_msg.encoding = frame.encoding;
    metadata_msg.source_topic = topic;

    std::string sanitized = topic;
    std::replace(sanitized.begin(), sanitized.end(), '/', '_');
    if (!sanitized.empty() && sanitized[0] == '_') {
      sanitized = sanitized.substr(1);
    }
    metadata_msg.video_file_path = "videos/" + sanitized + ".mp4";

    rclcpp::Serialization<rosbag_recorder::msg::ImageMetadata> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&metadata_msg, &serialized_msg);

    std::string metadata_topic = topic + "/metadata";
    std::string metadata_type = "rosbag_recorder/msg/ImageMetadata";

    writer_->write(
      std::make_shared<rclcpp::SerializedMessage>(serialized_msg),
      metadata_topic,
      metadata_type,
      timestamp);
  }
}

void SyncedImageBagRecorder::write_stats_report()
{
  if (!synced_compressor_ || current_bag_uri_.empty()) {
    return;
  }

  auto stats = synced_compressor_->get_stats();

  std::string report_path = current_bag_uri_ + "/sync_stats_report.json";
  std::ofstream report(report_path);

  report << "{\n";
  report << "  \"total_synced_frames\": " << stats.total_synced_frames << ",\n";
  report << "  \"recording_duration_sec\": "
         << (stats.recording_end_ns - stats.recording_start_ns) / 1e9 << ",\n";
  report << "  \"peak_ram_usage_mb\": "
         << stats.peak_ram_usage_bytes / (1024.0 * 1024.0) << ",\n";
  report << "  \"topics\": {\n";

  bool first = true;
  for (const auto & [topic, topic_stats] : stats.topic_stats) {
    if (!first) {
      report << ",\n";
    }
    first = false;

    double avg_staleness_ms = topic_stats.synced_frames > 0 ?
      (topic_stats.total_staleness_ns / static_cast<double>(topic_stats.synced_frames)) / 1e6 :
      0.0;

    report << "    \"" << topic << "\": {\n";
    report << "      \"received_frames\": " << topic_stats.received_frames << ",\n";
    report << "      \"synced_frames\": " << topic_stats.synced_frames << ",\n";
    report << "      \"dropped_frames\": " << topic_stats.dropped_frames << ",\n";
    report << "      \"avg_staleness_ms\": " << std::fixed << std::setprecision(2)
           << avg_staleness_ms << ",\n";
    report << "      \"max_staleness_ms\": " << std::fixed << std::setprecision(2)
           << topic_stats.max_staleness_ns / 1e6 << ",\n";
    report << "      \"min_staleness_ms\": " << std::fixed << std::setprecision(2)
           << (topic_stats.min_staleness_ns == INT64_MAX ? 0.0 :
               topic_stats.min_staleness_ns / 1e6) << "\n";
    report << "    }";
  }

  report << "\n  },\n";

  report << "  \"frame_counts\": {\n";
  first = true;
  for (const auto & [topic, count] : frame_counts_) {
    if (!first) {
      report << ",\n";
    }
    first = false;
    report << "    \"" << topic << "\": " << count;
  }
  report << "\n  }\n";

  report << "}\n";
  report.close();

  RCLCPP_INFO(this->get_logger(), "Stats report written to: %s", report_path.c_str());

  bool all_equal = true;
  uint32_t first_count = 0;
  for (const auto & [topic, count] : frame_counts_) {
    if (first_count == 0) {
      first_count = count;
    } else if (count != first_count) {
      all_equal = false;
      break;
    }
  }

  if (all_equal) {
    RCLCPP_INFO(
      this->get_logger(),
      "Frame count verification PASSED: All cameras have %u frames",
      first_count);
  } else {
    RCLCPP_WARN(this->get_logger(), "Frame count verification FAILED: Counts differ!");
    for (const auto & [topic, count] : frame_counts_) {
      RCLCPP_WARN(this->get_logger(), "  %s: %u frames", topic.c_str(), count);
    }
  }
}

}  // namespace rosbag_recorder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosbag_recorder::SyncedImageBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
