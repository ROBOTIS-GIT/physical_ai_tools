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

#include "rosbag_recorder/srv/set_record_config.hpp"
#include "std_srvs/srv/trigger.hpp"

class ServiceBagRecorder : public rclcpp::Node
{
public:
  ServiceBagRecorder();

private:
  void handle_set_record_config(
    const std::shared_ptr<rosbag_recorder::srv::SetRecordConfig::Request> req,
    std::shared_ptr<rosbag_recorder::srv::SetRecordConfig::Response> res);

  void handle_prepare(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_stop_and_delete(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void handle_serialized_message(
    const std::string & topic,
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg);

  std::vector<std::string> get_missing_topics(
    const std::map<std::string, std::vector<std::string>> & names_and_types);
  void create_topics_in_bag(
    const std::map<std::string, std::vector<std::string>> & names_and_types);
  void delete_bag_directory(const std::string & bag_uri);
  void create_subscriptions();

  rclcpp::Service<rosbag_recorder::srv::SetRecordConfig>::SharedPtr set_record_config_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_and_delete_srv_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unordered_map<std::string, std::string> type_for_topic_;
  bool is_recording_ {false};
  std::string current_bag_uri_;
  std::vector<std::string> topics_to_record_ {};
  std::mutex mutex_;
};

#endif  // ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_
