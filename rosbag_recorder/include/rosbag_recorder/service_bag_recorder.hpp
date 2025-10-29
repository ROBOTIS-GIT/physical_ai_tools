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

#include <rosbag_recorder_msgs/srv/start_recording.hpp>
#include <rosbag_recorder_msgs/srv/stop_recording.hpp>
#include <rosbag_recorder_msgs/srv/stop_and_delete_recording.hpp>

class ServiceBagRecorder : public rclcpp::Node {
public:
  ServiceBagRecorder();

private:
  void handle_start(const std::shared_ptr<rosbag_recorder_msgs::srv::StartRecording::Request> req,
                    std::shared_ptr<rosbag_recorder_msgs::srv::StartRecording::Response> res);

  void handle_stop(const std::shared_ptr<rosbag_recorder_msgs::srv::StopRecording::Request> req,
                   std::shared_ptr<rosbag_recorder_msgs::srv::StopRecording::Response> res);

  void handle_stop_and_delete(const std::shared_ptr<rosbag_recorder_msgs::srv::StopAndDeleteRecording::Request> req,
                             std::shared_ptr<rosbag_recorder_msgs::srv::StopAndDeleteRecording::Response> res);

  void handle_serialized_message(const std::string & topic,
                                 const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg);

  rclcpp::Service<rosbag_recorder_msgs::srv::StartRecording>::SharedPtr start_srv_;
  rclcpp::Service<rosbag_recorder_msgs::srv::StopRecording>::SharedPtr stop_srv_;
  rclcpp::Service<rosbag_recorder_msgs::srv::StopAndDeleteRecording>::SharedPtr stop_and_delete_srv_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unordered_map<std::string, std::string> type_for_topic_;
  bool is_recording_ {false};
  std::string current_bag_uri_;
  std::mutex mutex_;
};

#endif  // ROSBAG_RECORDER__SERVICE_BAG_RECORDER_HPP_


