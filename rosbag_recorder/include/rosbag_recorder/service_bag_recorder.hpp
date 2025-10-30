#ifndef ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_
#define ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "rosbag_recorder/srv/send_command.hpp"


class ServiceBagRecorder : public rclcpp::Node
{
public:
  ServiceBagRecorder();

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

  std::vector<std::string> get_missing_topics(
    const std::map<std::string, std::vector<std::string>> & names_and_types);
  void create_topics_in_bag(
    const std::map<std::string, std::vector<std::string>> & names_and_types);
  void delete_bag_directory(const std::string & bag_uri);
  void create_subscriptions();

  rclcpp::Service<rosbag_recorder::srv::SendCommand>::SharedPtr send_command_srv_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unordered_map<std::string, std::string> type_for_topic_;
  bool is_recording_ {false};
  std::string current_bag_uri_;
  std::vector<std::string> topics_to_record_ {};
  std::mutex mutex_;
};

#endif  // ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_
