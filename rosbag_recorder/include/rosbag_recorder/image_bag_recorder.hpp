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

#ifndef ROSBAG_RECORDER__IMAGE_BAG_RECORDER_HPP_
#define ROSBAG_RECORDER__IMAGE_BAG_RECORDER_HPP_

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

#include "rosbag_recorder/srv/send_command.hpp"
#include "rosbag_recorder/msg/image_metadata.hpp"
#include "rosbag_recorder/image_compressor.hpp"

namespace rosbag_recorder
{

class ImageBagRecorder : public rclcpp::Node
{
public:
  ImageBagRecorder();

private:
  void handle_send_command(
    const std::shared_ptr<rosbag_recorder::srv::SendCommand::Request> req,
    std::shared_ptr<rosbag_recorder::srv::SendCommand::Response> res);

  void handle_prepare(const std::vector<std::string> & topics);
  void handle_start(const std::string & uri);
  void handle_stop();
  void handle_stop_and_delete();
  void handle_finish();

  void handle_serialized_message(
    const std::string & topic,
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg);

  void handle_image_message(
    const std::string & topic,
    const sensor_msgs::msg::Image::SharedPtr & image_msg);

  std::vector<std::string> get_missing_topics(
    const std::map<std::string, std::vector<std::string>> & names_and_types);

  void create_topics_in_bag(
    const std::map<std::string, std::vector<std::string>> & names_and_types);

  void delete_bag_directory(const std::string & bag_uri);
  void create_subscriptions();
  bool is_image_topic(const std::string & topic_type) const;

  rclcpp::Service<rosbag_recorder::srv::SendCommand>::SharedPtr send_command_srv_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subscriptions_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscriptions_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unique_ptr<ImageCompressor> image_compressor_;

  std::unordered_map<std::string, std::string> type_for_topic_;
  std::vector<std::string> image_topics_;
  std::vector<std::string> non_image_topics_;

  bool is_recording_{false};
  std::string current_bag_uri_;
  std::vector<std::string> topics_to_record_{};
  std::mutex mutex_;
};

}  // namespace rosbag_recorder

#endif  // ROSBAG_RECORDER__IMAGE_BAG_RECORDER_HPP_
