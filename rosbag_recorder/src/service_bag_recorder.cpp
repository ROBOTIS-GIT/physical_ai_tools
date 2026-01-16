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
// Author: Woojin Wie, Kiwoong Park, Dongyun Kim


#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include "rosbag_recorder/service_bag_recorder.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"


ServiceBagRecorder::ServiceBagRecorder()
: rclcpp::Node("service_bag_recorder")
{
  RCLCPP_INFO(this->get_logger(), "Starting rosbag recorder node with image compression");

  this->declare_parameter<bool>("compress_images", true);
  this->declare_parameter<double>("video_fps", 15.0);

  compress_images_ = this->get_parameter("compress_images").as_bool();
  video_fps_ = this->get_parameter("video_fps").as_double();

  RCLCPP_INFO(
    this->get_logger(), "Image compression: %s, FPS: %.1f",
    compress_images_ ? "enabled" : "disabled", video_fps_);

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
        handle_prepare(req->topics, req->robot_type);
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
      case rosbag_recorder::srv::SendCommand::Request::CHECK_READY:
        handle_check_ready(res);
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

bool ServiceBagRecorder::is_image_topic(const std::string & topic_type) const
{
  return topic_type == "sensor_msgs/msg/Image";
}

bool ServiceBagRecorder::is_compressed_image_topic(const std::string & topic_type) const
{
  return topic_type == "sensor_msgs/msg/CompressedImage";
}

void ServiceBagRecorder::handle_prepare(
  const std::vector<std::string> & topics,
  const std::string & robot_type)
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
    current_robot_type_ = robot_type;
    image_topics_.clear();
    compressed_image_topics_.clear();
    non_image_topics_.clear();
    camera_mappings_.clear();
    action_topic_mappings_.clear();
    joint_order_.clear();

    if (!robot_type.empty()) {
      load_robot_config(robot_type);
    }

    auto names_and_types = this->get_topic_names_and_types();

    for (const auto & topic : topics_to_record_) {
      auto it = names_and_types.find(topic);
      if (it == names_and_types.end()) {
        continue;
      }

      const std::string & type = it->second.front();
      type_for_topic_[topic] = type;

      if (compress_images_ && is_image_topic(type)) {
        image_topics_.push_back(topic);
      } else if (compress_images_ && is_compressed_image_topic(type)) {
        compressed_image_topics_.push_back(topic);
      } else {
        non_image_topics_.push_back(topic);
      }
    }

    add_tf_topics();
    create_subscriptions();

    topic_health_checker_.clear();
    for (const auto & topic : image_topics_) {
      topic_health_checker_.register_topic_auto(topic, true);
    }
    for (const auto & topic : compressed_image_topics_) {
      topic_health_checker_.register_topic_auto(topic, true);
    }
    for (const auto & topic : non_image_topics_) {
      topic_health_checker_.register_topic_auto(topic, false);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Recording prepared: topics=%zu (image=%zu, compressed=%zu, other=%zu), robot_type=%s",
      topics_to_record_.size(),
      image_topics_.size(),
      compressed_image_topics_.size(),
      non_image_topics_.size(),
      robot_type.c_str());
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

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(current_bag_uri_);

    if (compress_images_ && (!image_topics_.empty() || !compressed_image_topics_.empty())) {
      std::string video_output_dir = current_bag_uri_ + "/videos";
      image_compressor_ = std::make_unique<rosbag_recorder::ImageCompressor>(
        video_output_dir, video_fps_);
      RCLCPP_INFO(
        this->get_logger(), "Image compressor initialized: %s (fps=%.1f)",
        video_output_dir.c_str(), video_fps_);
    }

    auto names_and_types = this->get_topic_names_and_types();
    auto missing_topics = get_missing_topics(names_and_types);

    if (!missing_topics.empty()) {
      writer_.reset();
      image_compressor_.reset();
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

    record_robot_description();

    save_robot_config_yaml(current_bag_uri_);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to start recording: ") + e.what());
  }

  is_recording_ = true;

  RCLCPP_INFO(
    this->get_logger(), "Recording started: uri=%s topics=%zu",
    current_bag_uri_.c_str(), topics_to_record_.size());
}

void ServiceBagRecorder::handle_stop()
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
      RCLCPP_INFO(this->get_logger(), "Image compressor finalized");
    }

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
    throw std::runtime_error(std::string("Failed to stop recording and delete bag: ") + e.what());
  }
}

void ServiceBagRecorder::handle_finish()
{
  RCLCPP_INFO(this->get_logger(), "Finish Rosbag recording");

  generic_subscriptions_.clear();
  image_subscriptions_.clear();
  compressed_image_subscriptions_.clear();

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
    if (it == names_and_types.end()) {
      continue;
    }
    const std::string & type = it->second.front();

    type_for_topic_[topic] = type;

    rosbag2_storage::TopicMetadata meta;

    // For image topics with compression enabled, store metadata instead
    if (compress_images_ && (is_image_topic(type) || is_compressed_image_topic(type))) {
      meta.name = topic + "/metadata";
      meta.type = "rosbag_recorder/msg/ImageMetadata";
    } else {
      meta.name = topic;
      meta.type = type;
    }

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

  generic_subscriptions_.clear();
  image_subscriptions_.clear();
  compressed_image_subscriptions_.clear();

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

  // Create typed subscriptions for Image topics
  for (const auto & topic : image_topics_) {
    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
      topic,
      rclcpp::QoS(100),
      [this, topic](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->handle_image_message(topic, msg);
      });
    image_subscriptions_.push_back(sub);
  }

  // Create typed subscriptions for CompressedImage topics
  for (const auto & topic : compressed_image_topics_) {
    auto sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic,
      rclcpp::QoS(100),
      [this, topic](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        this->handle_compressed_image_message(topic, msg);
      });
    compressed_image_subscriptions_.push_back(sub);
  }
}

void ServiceBagRecorder::handle_serialized_message(
  const std::string & topic,
  const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  topic_health_checker_.record_message_now(topic);

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

void ServiceBagRecorder::handle_image_message(
  const std::string & topic,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  topic_health_checker_.record_message_now(topic);

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

void ServiceBagRecorder::handle_compressed_image_message(
  const std::string & topic,
  const sensor_msgs::msg::CompressedImage::SharedPtr & compressed_msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  topic_health_checker_.record_message_now(topic);

  if (!is_recording_ || !writer_ || !image_compressor_) {
    return;
  }

  try {
    // Decompress the image
    cv::Mat frame = cv::imdecode(
      cv::Mat(compressed_msg->data), cv::IMREAD_COLOR);

    if (frame.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to decompress image from topic %s", topic.c_str());
      return;
    }

    // Create a temporary Image message for the compressor
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    image_msg->header = compressed_msg->header;
    image_msg->width = frame.cols;
    image_msg->height = frame.rows;
    image_msg->encoding = "bgr8";
    image_msg->step = frame.cols * 3;
    image_msg->data.assign(frame.data, frame.data + frame.total() * frame.elemSize());

    // Add frame to MP4 and get metadata
    auto metadata_info = image_compressor_->add_frame(topic, image_msg);

    // Create metadata message
    rosbag_recorder::msg::ImageMetadata metadata_msg;
    metadata_msg.header = compressed_msg->header;
    metadata_msg.frame_index = metadata_info.frame_index;
    metadata_msg.width = image_msg->width;
    metadata_msg.height = image_msg->height;
    metadata_msg.encoding = compressed_msg->format;
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
      "Failed to process compressed image from topic %s: %s",
      topic.c_str(), e.what());
  }
}

bool ServiceBagRecorder::load_robot_config(const std::string & robot_type)
{
  try {
    std::string package_share_dir =
      ament_index_cpp::get_package_share_directory("physical_ai_server");
    std::string config_path = package_share_dir + "/config/" + robot_type + "_config.yaml";

    RCLCPP_INFO(this->get_logger(), "Loading robot config from: %s", config_path.c_str());

    if (!std::filesystem::exists(config_path)) {
      RCLCPP_WARN(this->get_logger(), "Robot config file not found: %s", config_path.c_str());
      return false;
    }

    YAML::Node config = YAML::LoadFile(config_path);

    auto camera_topic_list =
      config["physical_ai_server"]["ros__parameters"][robot_type]["camera_topic_list"];

    if (!camera_topic_list || !camera_topic_list.IsSequence()) {
      RCLCPP_WARN(this->get_logger(), "No camera_topic_list found in config");
      return false;
    }

    camera_mappings_.clear();
    for (const auto & item : camera_topic_list) {
      std::string entry = item.as<std::string>();
      size_t colon_pos = entry.find(':');
      if (colon_pos != std::string::npos) {
        CameraMapping mapping;
        mapping.name = entry.substr(0, colon_pos);
        mapping.topic = entry.substr(colon_pos + 1);
        camera_mappings_.push_back(mapping);
        RCLCPP_INFO(
          this->get_logger(), "Camera mapping: %s -> %s",
          mapping.name.c_str(), mapping.topic.c_str());
      }
    }

    // Load joint_topic_list for action topics (leader topics)
    auto joint_topic_list =
      config["physical_ai_server"]["ros__parameters"][robot_type]["joint_topic_list"];

    action_topic_mappings_.clear();
    if (joint_topic_list && joint_topic_list.IsSequence()) {
      for (const auto & item : joint_topic_list) {
        std::string entry = item.as<std::string>();
        size_t colon_pos = entry.find(':');
        if (colon_pos != std::string::npos) {
          std::string name = entry.substr(0, colon_pos);
          std::string topic = entry.substr(colon_pos + 1);
          // Only add leader topics as action topics
          if (name.find("leader") != std::string::npos) {
            JointMapping mapping;
            mapping.name = name;
            mapping.topic = topic;
            action_topic_mappings_.push_back(mapping);
            RCLCPP_INFO(
              this->get_logger(), "Action topic mapping: %s -> %s",
              mapping.name.c_str(), mapping.topic.c_str());
          }
        }
      }
      RCLCPP_INFO(
        this->get_logger(), "Loaded %zu action topic mappings from config",
        action_topic_mappings_.size());
    }

    // Load joint_order from joint_list and joint_order map
    auto joint_order_node =
      config["physical_ai_server"]["ros__parameters"][robot_type]["joint_order"];
    auto joint_list =
      config["physical_ai_server"]["ros__parameters"][robot_type]["joint_list"];

    joint_order_.clear();
    if (joint_order_node && joint_order_node.IsMap() && joint_list && joint_list.IsSequence()) {
      for (const auto & group_name : joint_list) {
        std::string group = group_name.as<std::string>();
        auto joints = joint_order_node[group];
        if (joints && joints.IsSequence()) {
          for (const auto & joint : joints) {
            joint_order_.push_back(joint.as<std::string>());
          }
        }
      }
      RCLCPP_INFO(
        this->get_logger(), "Loaded %zu joints from joint_order",
        joint_order_.size());
    }

    RCLCPP_INFO(
      this->get_logger(), "Loaded %zu camera mappings from config",
      camera_mappings_.size());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load robot config: %s", e.what());
    return false;
  }
}

void ServiceBagRecorder::save_robot_config_yaml(const std::string & bag_uri)
{
  if (current_robot_type_.empty() && camera_mappings_.empty() && action_topic_mappings_.empty()) {
    return;
  }

  try {
    std::string config_path = bag_uri + "/robot_config.yaml";
    YAML::Emitter out;
    out << YAML::BeginMap;

    if (!current_robot_type_.empty()) {
      out << YAML::Key << "robot_type" << YAML::Value << current_robot_type_;
    }

    if (!camera_mappings_.empty()) {
      out << YAML::Key << "camera_mapping" << YAML::Value << YAML::BeginMap;
      for (const auto & mapping : camera_mappings_) {
        out << YAML::Key << mapping.topic << YAML::Value << mapping.name;
      }
      out << YAML::EndMap;
    }

    if (!action_topic_mappings_.empty()) {
      out << YAML::Key << "action_topics" << YAML::Value << YAML::BeginMap;
      for (const auto & mapping : action_topic_mappings_) {
        out << YAML::Key << mapping.name << YAML::Value << mapping.topic;
      }
      out << YAML::EndMap;
    }

    if (!joint_order_.empty()) {
      out << YAML::Key << "joint_order" << YAML::Value << YAML::BeginSeq;
      for (const auto & joint : joint_order_) {
        out << joint;
      }
      out << YAML::EndSeq;
    }

    out << YAML::EndMap;

    std::ofstream fout(config_path);
    fout << out.c_str();
    fout.close();

    RCLCPP_INFO(this->get_logger(), "Saved robot config to: %s", config_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save robot config: %s", e.what());
  }
}

std::string ServiceBagRecorder::get_camera_name_for_topic(const std::string & topic) const
{
  for (const auto & mapping : camera_mappings_) {
    if (mapping.topic == topic) {
      return mapping.name;
    }
  }
  return "";
}

void ServiceBagRecorder::handle_check_ready(
  std::shared_ptr<rosbag_recorder::srv::SendCommand::Response> res)
{
  bool is_ready = topic_health_checker_.is_all_stable();
  auto pending = topic_health_checker_.get_pending_topics();

  res->success = true;
  res->ready = is_ready;
  res->pending_topics = pending;

  if (is_ready) {
    res->message = "All topics are stable";
    RCLCPP_INFO(this->get_logger(), "CHECK_READY: All topics stable");
  } else {
    res->message = "Waiting for topics to stabilize: " + std::to_string(pending.size()) + " pending";
    RCLCPP_DEBUG(
      this->get_logger(),
      "CHECK_READY: %zu topics pending", pending.size());
  }
}

void ServiceBagRecorder::add_tf_topics()
{
  auto names_and_types = this->get_topic_names_and_types();

  std::vector<std::string> tf_topics_to_add = {TF_TOPIC, TF_STATIC_TOPIC, ROBOT_DESCRIPTION_TOPIC};

  for (const auto & topic : tf_topics_to_add) {
    if (std::find(topics_to_record_.begin(), topics_to_record_.end(), topic) !=
        topics_to_record_.end()) {
      continue;
    }

    auto it = names_and_types.find(topic);
    if (it != names_and_types.end() && !it->second.empty()) {
      topics_to_record_.push_back(topic);
      type_for_topic_[topic] = it->second.front();
      RCLCPP_INFO(this->get_logger(), "Auto-added TF topic: %s", topic.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "TF topic not available: %s", topic.c_str());
    }
  }
}

void ServiceBagRecorder::record_robot_description()
{
  if (!writer_) {
    return;
  }

  try {
    auto client = this->create_client<rcl_interfaces::srv::GetParameters>(
      "/robot_state_publisher/get_parameters");

    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(this->get_logger(), "robot_state_publisher service not available");
      return;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");

    auto future = client->async_send_request(request);
    auto status = future.wait_for(std::chrono::seconds(5));

    if (status != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Timeout getting robot_description parameter");
      return;
    }

    auto response = future.get();
    if (response->values.empty() ||
        response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      RCLCPP_WARN(this->get_logger(), "robot_description parameter not found or wrong type");
      return;
    }

    std_msgs::msg::String urdf_msg;
    urdf_msg.data = response->values[0].string_value;

    rclcpp::Serialization<std_msgs::msg::String> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&urdf_msg, &serialized_msg);

    writer_->write(
      std::make_shared<rclcpp::SerializedMessage>(serialized_msg),
      ROBOT_DESCRIPTION_TOPIC,
      "std_msgs/msg/String",
      this->now());

    RCLCPP_INFO(this->get_logger(), "Recorded robot_description (URDF) to bag");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to record robot_description: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
