#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag_recorder/service_bag_recorder.hpp"

ServiceBagRecorder::ServiceBagRecorder() : rclcpp::Node("service_bag_recorder")
{
  start_srv_ = this->create_service<rosbag_recorder_msgs::srv::StartRecording>(
    "start_recording",
    std::bind(&ServiceBagRecorder::handle_start, this, std::placeholders::_1, std::placeholders::_2));

  stop_srv_ = this->create_service<rosbag_recorder_msgs::srv::StopRecording>(
    "stop_recording",
    std::bind(&ServiceBagRecorder::handle_stop, this, std::placeholders::_1, std::placeholders::_2));
}

void ServiceBagRecorder::handle_start(const std::shared_ptr<rosbag_recorder_msgs::srv::StartRecording::Request> req,
                  std::shared_ptr<rosbag_recorder_msgs::srv::StartRecording::Response> res)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (is_recording_) {
      res->success = false;
      res->message = "Already recording";
      return;
    }

    if (req->uri.empty() || req->topics.empty()) {
      res->success = false;
      res->message = "uri and topics must be provided";
      return;
    }

    try {
      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      writer_->open(req->uri);

      // Resolve types for requested topics
      std::vector<std::string> missing_topics;
      auto names_and_types = this->get_topic_names_and_types();
      for (const auto & topic : req->topics) {
        auto it = names_and_types.find(topic);
        if (it == names_and_types.end() || it->second.empty()) {
          missing_topics.push_back(topic);
          continue;
        }
        const std::string & type = it->second.front();
        type_for_topic_[topic] = type;

        rosbag2_storage::TopicMetadata meta;
        meta.name = topic;
        meta.type = type;
        meta.serialization_format = rmw_get_serialization_format();
        writer_->create_topic(meta);
      }

      if (!missing_topics.empty()) {
        writer_.reset();
        type_for_topic_.clear();
        std::ostringstream oss;
        oss << "Types not found for topics:";
        for (const auto & t : missing_topics) { oss << " " << t; }
        res->success = false;
        res->message = oss.str();
        return;
      }

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

      is_recording_ = true;
      res->success = true;
      res->message = "Recording started";
      RCLCPP_INFO(this->get_logger(), "Recording started: uri=%s topics=%zu", req->uri.c_str(), req->topics.size());
    } catch (const std::exception & e) {
      writer_.reset();
      is_recording_ = false;
      res->success = false;
      res->message = std::string("Failed to start recording: ") + e.what();
    }
  }

void ServiceBagRecorder::handle_stop(const std::shared_ptr<rosbag_recorder_msgs::srv::StopRecording::Request> /*req*/,
                 std::shared_ptr<rosbag_recorder_msgs::srv::StopRecording::Response> res)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (!is_recording_) {
      res->success = false;
      res->message = "Not recording";
      return;
    }

    try {
      subscriptions_.clear();
      writer_.reset();
      type_for_topic_.clear();
      is_recording_ = false;
      res->success = true;
      res->message = "Recording stopped";
      RCLCPP_INFO(this->get_logger(), "Recording stopped");
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("Failed to stop recording: ") + e.what();
    }
  }

void ServiceBagRecorder::handle_serialized_message(const std::string & topic,
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


