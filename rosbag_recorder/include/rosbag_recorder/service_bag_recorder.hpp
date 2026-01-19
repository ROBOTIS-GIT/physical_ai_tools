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


#ifndef ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_
#define ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "rosbag_recorder/srv/send_command.hpp"
#include "rosbag_recorder/msg/image_metadata.hpp"
#include "rosbag_recorder/image_compressor.hpp"
#include "rosbag_recorder/topic_health_checker.hpp"


struct CameraMapping
{
  std::string name;
  std::string topic;
};

struct JointMapping
{
  std::string name;
  std::string topic;
};


class ServiceBagRecorder : public rclcpp::Node
{
public:
  ServiceBagRecorder();

private:
  void handle_send_command(
    const std::shared_ptr<rosbag_recorder::srv::SendCommand::Request> req,
    std::shared_ptr<rosbag_recorder::srv::SendCommand::Response> res);

  void handle_prepare(const std::vector<std::string> & topics, const std::string & robot_type);
  void handle_start(const std::string & uri);
  void handle_stop();
  void handle_stop_and_delete();
  void handle_finish();
  void handle_check_ready(
    std::shared_ptr<rosbag_recorder::srv::SendCommand::Response> res);

  bool load_robot_config(const std::string & robot_type);
  void save_robot_config_yaml(const std::string & bag_uri);
  std::string get_camera_name_for_topic(const std::string & topic) const;

  void handle_serialized_message(
    const std::string & topic,
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg);

  void handle_image_message(
    const std::string & topic,
    const sensor_msgs::msg::Image::SharedPtr & image_msg);

  void handle_compressed_image_message(
    const std::string & topic,
    const sensor_msgs::msg::CompressedImage::SharedPtr & compressed_msg);

  std::vector<std::string> get_missing_topics(
    const std::map<std::string, std::vector<std::string>> & names_and_types);
  void create_topics_in_bag(
    const std::map<std::string, std::vector<std::string>> & names_and_types);
  void delete_bag_directory(const std::string & bag_uri);
  void create_subscriptions();
  bool is_image_topic(const std::string & topic_type) const;
  bool is_compressed_image_topic(const std::string & topic_type) const;
  void add_tf_topics();
  void record_robot_description();

  /**
   * Extract timestamp from serialized ROS message.
   * Uses message header if available, falls back to current time.
   */
  rclcpp::Time extract_timestamp_from_serialized(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
    const std::string & topic);

  rclcpp::Service<rosbag_recorder::srv::SendCommand>::SharedPtr send_command_srv_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subscriptions_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscriptions_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr>
    compressed_image_subscriptions_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unique_ptr<rosbag_recorder::ImageCompressor> image_compressor_;

  std::unordered_map<std::string, std::string> type_for_topic_;
  std::vector<std::string> image_topics_;
  std::vector<std::string> compressed_image_topics_;
  std::vector<std::string> non_image_topics_;

  bool is_recording_{false};
  bool is_buffering_{false};
  bool compress_images_{true};
  double video_fps_{15.0};
  std::string current_bag_uri_;
  std::string current_robot_type_;
  std::vector<std::string> topics_to_record_{};
  std::vector<CameraMapping> camera_mappings_;
  std::vector<JointMapping> action_topic_mappings_;
  std::vector<std::string> joint_order_;
  std::mutex mutex_;
  std::mutex buffer_mutex_;

  struct BufferedSerializedMsg {
    std::string topic;
    std::shared_ptr<rclcpp::SerializedMessage> msg;
    std::chrono::steady_clock::time_point receive_time;
  };
  std::vector<BufferedSerializedMsg> serialized_buffer_;

  struct BufferedImageMsg {
    std::string topic;
    sensor_msgs::msg::Image::SharedPtr msg;
    std::chrono::steady_clock::time_point receive_time;
  };
  std::vector<BufferedImageMsg> image_buffer_;

  struct BufferedCompressedImageMsg {
    std::string topic;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    std::chrono::steady_clock::time_point receive_time;
  };
  std::vector<BufferedCompressedImageMsg> compressed_image_buffer_;

  rosbag_recorder::TopicHealthChecker topic_health_checker_;

  static constexpr const char* TF_TOPIC = "/tf";
  static constexpr const char* TF_STATIC_TOPIC = "/tf_static";
  static constexpr const char* ROBOT_DESCRIPTION_TOPIC = "/robot_description";
};

#endif  // ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_
