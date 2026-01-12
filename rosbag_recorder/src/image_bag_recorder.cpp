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

#include "rosbag_recorder/image_bag_recorder.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag_recorder
{

ImageBagRecorder::ImageBagRecorder()
: rclcpp::Node("image_bag_recorder")
{
  RCLCPP_INFO(this->get_logger(), "Starting image bag recorder node");

  send_command_srv_ = this->create_service<rosbag_recorder::srv::SendCommand>(
    "rosbag_recorder/send_command",
    std::bind(
      &ImageBagRecorder::handle_send_command, this,
      std::placeholders::_1, std::placeholders::_2));
}

void ImageBagRecorder::handle_send_command(
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

bool ImageBagRecorder::is_image_topic(const std::string & topic_type) const
{
  return topic_type == "sensor_msgs/msg/Image";
}

void ImageBagRecorder::handle_prepare(const std::vector<std::string> & topics)
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

void ImageBagRecorder::handle_start(const std::string & uri)
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

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(current_bag_uri_);

    // Create image compressor for MP4 videos
    std::string video_output_dir = current_bag_uri_ + "/videos";
    image_compressor_ = std::make_unique<ImageCompressor>(video_output_dir);

    auto names_and_types = this->get_topic_names_and_types();
    auto missing_topics = get_missing_topics(names_and_types);

    if (!missing_topics.empty()) {
      writer_.reset();
      image_compressor_.reset();
      type_for_topic_.clear();

      delete_bag_directory(current_bag_uri_);
      current_bag_uri_.clear();

      std::ostringstream oss;
      oss << "Types not found for topics:";
      for (const auto & t : missing_topics) {
        oss << " " << t;
      }

      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to start recording: %s", oss.str().c_str());

      throw std::runtime_error(oss.str());
    }

    create_topics_in_bag(names_and_types);
  } catch (const std::exception & e) {
    throw std::runtime_error(
            std::string("Failed to start recording: ") + e.what());
  }

  is_recording_ = true;

  RCLCPP_INFO(
    this->get_logger(), "Recording started: uri=%s topics=%zu",
    current_bag_uri_.c_str(), topics_to_record_.size());
}

void ImageBagRecorder::handle_stop()
{
  RCLCPP_INFO(this->get_logger(), "Stop Rosbag recording");

  if (!is_recording_) {
    throw std::runtime_error("Not recording");
  }

  try {
    // Finalize all video writers
    if (image_compressor_) {
      image_compressor_->finalize_all();
      image_compressor_.reset();
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

void ImageBagRecorder::handle_stop_and_delete()
{
  RCLCPP_INFO(this->get_logger(), "Stop and delete Rosbag recording");

  if (!is_recording_) {
    throw std::runtime_error("Not recording");
  }

  try {
    is_recording_ = false;

    if (image_compressor_) {
      image_compressor_->finalize_all();
      image_compressor_.reset();
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

void ImageBagRecorder::handle_finish()
{
  RCLCPP_INFO(this->get_logger(), "Finish Rosbag recording");

  generic_subscriptions_.clear();
  image_subscriptions_.clear();

  if (is_recording_) {
    handle_stop();
  }
}

std::vector<std::string> ImageBagRecorder::get_missing_topics(
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

void ImageBagRecorder::create_topics_in_bag(
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
      // For image topics, store metadata instead of full images
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

void ImageBagRecorder::delete_bag_directory(const std::string & bag_uri)
{
  if (bag_uri.empty()) {
    return;
  }

  std::filesystem::path bag_path(bag_uri);
  if (std::filesystem::exists(bag_path)) {
    std::filesystem::remove_all(bag_path);
    RCLCPP_INFO(
      this->get_logger(), "Deleted bag directory: %s", bag_uri.c_str());
  }
}

void ImageBagRecorder::create_subscriptions()
{
  RCLCPP_INFO(this->get_logger(), "Creating subscriptions");

  generic_subscriptions_.clear();
  image_subscriptions_.clear();

  // Create generic subscriptions for non-image topics
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

  // Create typed subscriptions for image topics
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

void ImageBagRecorder::handle_serialized_message(
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

void ImageBagRecorder::handle_image_message(
  const std::string & topic,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  if (!is_recording_ || !writer_ || !image_compressor_) {
    return;
  }

  try {
    // Add frame to MP4 and get metadata
    auto metadata_info = image_compressor_->add_frame(topic, image_msg);

    // Create metadata message
    rosbag_recorder::msg::ImageMetadata metadata_msg;
    metadata_msg.header = image_msg->header;
    metadata_msg.frame_index = metadata_info.frame_index;
    metadata_msg.width = metadata_info.width;
    metadata_msg.height = metadata_info.height;
    metadata_msg.encoding = metadata_info.encoding;
    metadata_msg.source_topic = topic;

    // Generate relative path to video file
    std::string sanitized = topic;
    std::replace(sanitized.begin(), sanitized.end(), '/', '_');
    if (!sanitized.empty() && sanitized[0] == '_') {
      sanitized = sanitized.substr(1);
    }
    metadata_msg.video_file_path = "videos/" + sanitized + ".mp4";

    // Serialize and write metadata to bag
    rclcpp::Serialization<rosbag_recorder::msg::ImageMetadata> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&metadata_msg, &serialized_msg);

    std::string metadata_topic = topic + "/metadata";
    std::string metadata_type = "rosbag_recorder/msg/ImageMetadata";

    writer_->write(
      std::make_shared<rclcpp::SerializedMessage>(serialized_msg),
      metadata_topic,
      metadata_type,
      this->now());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to process image from topic %s: %s",
      topic.c_str(), e.what());
  }
}

}  // namespace rosbag_recorder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosbag_recorder::ImageBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
