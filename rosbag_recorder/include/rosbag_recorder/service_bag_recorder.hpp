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

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "rosbag_recorder/srv/send_command.hpp"
#include "rosbag_recorder/msg/image_metadata.hpp"
#include "rosbag_recorder/msg/recording_monitor.hpp"
#include "rosbag_recorder/image_compressor.hpp"


class ServiceBagRecorder : public rclcpp::Node
{
public:
  ServiceBagRecorder();

  // Callback groups for parallel processing with MultiThreadedExecutor
  rclcpp::CallbackGroup::SharedPtr camera_callback_group_;
  rclcpp::CallbackGroup::SharedPtr joint_callback_group_;
  rclcpp::CallbackGroup::SharedPtr other_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

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
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
    const rclcpp::MessageInfo & message_info);

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
  std::vector<std::string> wait_for_first_messages();
  bool is_image_topic(const std::string & topic_type) const;
  bool is_compressed_image_topic(const std::string & topic_type) const;
  rclcpp::QoS get_qos_for_topic(const std::string & topic);
  rclcpp::CallbackGroup::SharedPtr get_callback_group_for_topic(const std::string & topic);
  void log_statistics();
  void monitor_tick();

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

  // Track which topics are camera topics for callback group assignment
  std::unordered_set<std::string> camera_topics_;
  std::unordered_set<std::string> joint_topics_;

  std::atomic<bool> is_recording_ {false};
  bool compress_images_{true};
  std::string current_bag_uri_;
  std::vector<std::string> topics_to_record_ {};
  std::mutex mutex_;
  std::mutex metrics_mutex_;

  // Preflight: verify each topic is actually publishing before recording starts
  std::atomic<bool> preflight_active_{false};
  std::unordered_set<std::string> preflight_received_topics_;
  std::mutex preflight_mutex_;
  std::condition_variable preflight_cv_;
  int preflight_wait_timeout_ms_{2000};

  // Statistics for monitoring
  std::atomic<uint64_t> messages_received_{0};
  std::atomic<uint64_t> messages_written_{0};

  // Per-topic live monitor: counts + last-recv timestamp updated from the
  // hot path; rate / EMA / status fields are touched only by the monitor
  // timer thread so they don't need atomic accessors.
  struct TopicMetric
  {
    std::atomic<uint64_t> message_count{0};
    std::atomic<uint64_t> last_recv_ns{0};
    double ema_hz{0.0};
    uint64_t last_count_snapshot{0};
    uint64_t last_tick_ns{0};
    uint64_t subscribe_start_ns{0};
    bool ema_initialised{false};
    bool stalled{false};
  };
  std::unordered_map<std::string, std::unique_ptr<TopicMetric>> per_topic_metrics_;

  rclcpp::Publisher<rosbag_recorder::msg::RecordingMonitor>::SharedPtr monitor_pub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  double monitor_publish_hz_{1.0};
  int monitor_stall_window_ms_{2000};
  double monitor_stall_ratio_{0.2};
  double monitor_slow_ratio_{0.6};
  double monitor_ema_alpha_{0.2};
  double monitor_min_baseline_hz_{1.0};
  int monitor_warmup_ms_{3000};
  uint64_t monitor_start_ns_{0};

  // Storage configuration
  static constexpr size_t CACHE_SIZE_BYTES = 1024 * 1024 * 1024;  // 1GB
  static constexpr const char * STORAGE_ID = "mcap";
};

#endif  // ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_
