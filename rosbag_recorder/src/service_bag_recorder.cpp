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
// Author: Woojin Wie, Kiwoong Park


#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"

#include "rosbag_recorder/service_bag_recorder.hpp"


ServiceBagRecorder::ServiceBagRecorder()
: rclcpp::Node("service_bag_recorder")
{
  RCLCPP_INFO(this->get_logger(), "Starting rosbag recorder node");

  send_command_srv_ = this->create_service<rosbag_recorder::srv::SendCommand>(
    "rosbag_recorder/send_command",
    std::bind(
      &ServiceBagRecorder::handle_send_command, this, std::placeholders::_1,
      std::placeholders::_2));
}

void ServiceBagRecorder::handle_send_command(
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

void ServiceBagRecorder::handle_prepare(const std::vector<std::string> & topics)
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

    auto names_and_types = this->get_topic_names_and_types();

    for (const auto & topic : topics_to_record_) {
      auto it = names_and_types.find(topic);
      const std::string & type = it->second.front();

      type_for_topic_[topic] = type;
    }

    create_subscriptions();

    RCLCPP_INFO(
      this->get_logger(),
      "Recording prepared: topics=%zu",
      topics_to_record_.size());
  } catch (const std::exception & e) {
    writer_.reset();
    throw std::runtime_error(std::string("Failed to prepare recording: ") + e.what());
  }
}

void ServiceBagRecorder::handle_start(const std::string & uri)
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

    // Check if a bag already exists at the specified path and delete it
    delete_bag_directory(current_bag_uri_);

    // Configure storage options
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = current_bag_uri_;
    storage_options.storage_id = storage_id_;

    RCLCPP_INFO(
      this->get_logger(),
      "Using storage format: %s",
      storage_id_.c_str());

    // Configure compression via storage_config_uri
    // For SQLite3: use key=value format
    // For MCAP: create YAML config file
    if (enable_compression_) {
      if (storage_id_ == "sqlite3") {
        // SQLite3 uses key=value format in storage_config_uri
        storage_options.storage_config_uri =
          "compression_mode=" + compression_mode_ +
          ";compression_format=" + compression_format_;

        RCLCPP_INFO(
          this->get_logger(),
          "SQLite3 compression enabled: mode=%s, format=%s",
          compression_mode_.c_str(),
          compression_format_.c_str());
      } else if (storage_id_ == "mcap") {
        // MCAP: SequentialCompressionWriter handles compression, no need for storage_config_uri
        // Leave storage_config_uri empty - compression is handled by SequentialCompressionWriter
        storage_options.storage_config_uri = "";
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Compression disabled");
    }

    // Configure converter options (serialization)
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    // Configure compression options for SequentialCompressionWriter
    rosbag2_compression::CompressionOptions compression_options;

    if (enable_compression_) {
      // CompressionMode is an enum
      if (compression_mode_ == "message") {
        compression_options.compression_mode =
          rosbag2_compression::CompressionMode::MESSAGE;
      } else if (compression_mode_ == "file") {
        compression_options.compression_mode =
          rosbag2_compression::CompressionMode::FILE;
      } else {
        compression_options.compression_mode =
          rosbag2_compression::CompressionMode::NONE;
      }

      compression_options.compression_format = compression_format_;

      RCLCPP_INFO(
        this->get_logger(),
        "Using SequentialCompressionWriter with compression: mode=%s, format=%s",
        compression_mode_.c_str(),
        compression_format_.c_str());
    } else {
      compression_options.compression_mode =
        rosbag2_compression::CompressionMode::NONE;
    }

    // Create SequentialCompressionWriter if compression is enabled
    // Otherwise use regular Writer
    if (enable_compression_) {
      // SequentialCompressionWriter handles compression automatically
      auto compression_writer = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(
        compression_options
      );

      // SequentialCompressionWriter implements BaseWriterInterface
      // We need to wrap it in a Writer
      writer_ = std::make_unique<rosbag2_cpp::Writer>(
        std::move(compression_writer)
      );
    } else {
      writer_ = std::make_unique<rosbag2_cpp::Writer>();
    }

    // Log storage options before opening
    RCLCPP_INFO(
      this->get_logger(),
      "Opening writer with storage_id=%s",
      storage_options.storage_id.c_str());

    // Open writer with storage and converter options
    writer_->open(storage_options, converter_options);

    // Log compression status for verification
    if (enable_compression_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Compression ENABLED using SequentialCompressionWriter: "
        "storage=%s, mode=%s, format=%s",
        storage_id_.c_str(),
        compression_mode_.c_str(),
        compression_format_.c_str());
      RCLCPP_INFO(
        this->get_logger(),
        "Compression will be applied at %s level. "
        "Check metadata.yaml after recording to verify (compression_format should be '%s').",
        compression_mode_.c_str(),
        compression_format_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Compression disabled");
    }

    auto names_and_types = this->get_topic_names_and_types();
    auto missing_topics = get_missing_topics(names_and_types);

    if (!missing_topics.empty()) {
      writer_.reset();
      type_for_topic_.clear();

      // Delete the bag folder since we can't record the requested topics
      RCLCPP_INFO(
        this->get_logger(),
        "Deleting bag directory due to missing topic types: %s",
        current_bag_uri_.c_str());
      delete_bag_directory(current_bag_uri_);
      current_bag_uri_.clear();

      std::ostringstream oss;
      oss << "Types not found for topics:";
      for (const auto & t : missing_topics) {
        oss << " " << t;
      }

      RCLCPP_INFO(this->get_logger(), "Failed to start recording: %s", oss.str().c_str());

      throw std::runtime_error(oss.str());
    }

    create_topics_in_bag(names_and_types);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to start recording: ") + e.what());
  }

  is_recording_ = true;

  RCLCPP_INFO(
    this->get_logger(), "Recording started: uri=%s topics=%zu",
    current_bag_uri_.c_str(), topics_to_record_.size());

  if (enable_compression_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Recording with compression enabled. After recording, verify compression in metadata.yaml: "
      "ros2 bag info %s",
      current_bag_uri_.c_str());
  }
}

void ServiceBagRecorder::handle_stop()
{
  RCLCPP_INFO(this->get_logger(), "Stop Rosbag recording");

  if (!is_recording_) {
    throw std::runtime_error("Not recording");
  }

  try {
    writer_.reset();
    type_for_topic_.clear();
    current_bag_uri_.clear();
    is_recording_ = false;
    RCLCPP_INFO(this->get_logger(), "Recording stopped");
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to stop recording: ") + e.what());
  }
}

void ServiceBagRecorder::handle_stop_and_delete()
{
  RCLCPP_INFO(this->get_logger(), "Stop and delete Rosbag recording");

  if (!is_recording_) {
    throw std::runtime_error("Not recording");
  }

  try {
    is_recording_ = false;

    writer_.reset();
    type_for_topic_.clear();

    delete_bag_directory(current_bag_uri_);

    current_bag_uri_.clear();

    RCLCPP_INFO(this->get_logger(), "Recording stopped and bag deleted");
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to stop recording and delete bag: ") + e.what());
  }
}

void ServiceBagRecorder::handle_finish()
{
  RCLCPP_INFO(this->get_logger(), "Finish Rosbag recording");

  subscriptions_.clear();

  if (is_recording_) {
    handle_stop();
  }
}

std::vector<std::string> ServiceBagRecorder::get_missing_topics(
  const std::map<std::string, std::vector<std::string>> & names_and_types)
{
// Resolve types for requested topics
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

void ServiceBagRecorder::create_topics_in_bag(
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

    type_for_topic_[topic] = type;

    rosbag2_storage::TopicMetadata meta;
    meta.name = topic;
    meta.type = type;
    meta.serialization_format = rmw_get_serialization_format();

    writer_->create_topic(meta);
  }
}

void ServiceBagRecorder::delete_bag_directory(const std::string & bag_uri)
{
  if (bag_uri.empty()) {
    return;
  }

  std::filesystem::path bag_path(bag_uri);
  if (std::filesystem::exists(bag_path)) {
    std::filesystem::remove_all(bag_path);
    RCLCPP_INFO(
      this->get_logger(), "Deleted bag directory: %s",
      bag_uri.c_str());
  }
}

void ServiceBagRecorder::create_subscriptions()
{
  RCLCPP_INFO(this->get_logger(), "Creating subscriptions");

  subscriptions_.clear();

  // Create generic subscriptions for all topics
  for (const auto & [topic, type] : type_for_topic_) {
    auto options = rclcpp::SubscriptionOptions();
    auto sub = this->create_generic_subscription(
      topic,
      type,
      rclcpp::QoS(100),
      [this, topic](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
        this->handle_serialized_message(topic, serialized_msg);
      },
      options);
    subscriptions_.push_back(sub);
  }
}

void ServiceBagRecorder::handle_serialized_message(
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
