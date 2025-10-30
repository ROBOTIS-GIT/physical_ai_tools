#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag_recorder/service_bag_recorder.hpp"


ServiceBagRecorder::ServiceBagRecorder()
: rclcpp::Node("service_bag_recorder")
{
  RCLCPP_INFO(this->get_logger(), "Starting rosbag recorder node");

  set_record_config_srv_ = this->create_service<rosbag_recorder::srv::SetRecordConfig>(
    "rosbag_recorder/set_record_config",
    std::bind(
      &ServiceBagRecorder::handle_set_record_config, this, std::placeholders::_1,
      std::placeholders::_2));

  prepare_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "rosbag_recorder/command/prepare",
    std::bind(
      &ServiceBagRecorder::handle_prepare, this, std::placeholders::_1,
      std::placeholders::_2));

  start_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "rosbag_recorder/command/start",
    std::bind(
      &ServiceBagRecorder::handle_start, this, std::placeholders::_1,
      std::placeholders::_2));

  stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "rosbag_recorder/command/stop",
    std::bind(
      &ServiceBagRecorder::handle_stop, this, std::placeholders::_1,
      std::placeholders::_2));

  stop_and_delete_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "rosbag_recorder/command/stop_and_delete",
    std::bind(
      &ServiceBagRecorder::handle_stop_and_delete, this, std::placeholders::_1,
      std::placeholders::_2));
}

void ServiceBagRecorder::handle_set_record_config(
  const std::shared_ptr<rosbag_recorder::srv::SetRecordConfig::Request> req,
  std::shared_ptr<rosbag_recorder::srv::SetRecordConfig::Response> res)
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

  current_bag_uri_ = req->uri;
  topics_to_record_ = req->topics;

  res->success = true;
  res->message = "Record config set successfully";
}

void ServiceBagRecorder::handle_prepare(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  if (is_recording_) {
    res->success = false;
    res->message = "Already recording";
    return;
  }

  if (current_bag_uri_.empty() || topics_to_record_.empty()) {
    res->success = false;
    res->message = "Record config not set";
    return;
  }

  try {
    // Check if a bag already exists at the specified path and delete it
    delete_bag_directory(current_bag_uri_);

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(current_bag_uri_);

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

      res->success = false;
      res->message = oss.str();
      return;
    }

    create_subscriptions();

    RCLCPP_INFO(
      this->get_logger(),
      "Recording prepared: uri=%s topics=%zu",
      current_bag_uri_.c_str(),
      topics_to_record_.size());

    res->success = true;
    res->message = "Recording prepared";
  } catch (const std::exception & e) {
    writer_.reset();
    res->success = false;
    res->message = std::string("Failed to prepare recording: ") + e.what();
  }
}

void ServiceBagRecorder::handle_start(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  if (is_recording_) {
    res->success = false;
    res->message = "Already recording";
    return;
  }

  is_recording_ = true;

  res->success = true;
  res->message = "Recording started";

  RCLCPP_INFO(
    this->get_logger(), "Recording started: uri=%s topics=%zu",
    current_bag_uri_.c_str(), topics_to_record_.size());
}

void ServiceBagRecorder::handle_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
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
    current_bag_uri_.clear();
    is_recording_ = false;
    res->success = true;
    res->message = "Recording stopped";
    RCLCPP_INFO(this->get_logger(), "Recording stopped");
  } catch (const std::exception & e) {
    res->success = false;
    res->message = std::string("Failed to stop recording: ") + e.what();
  }
}

void ServiceBagRecorder::handle_stop_and_delete(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  if (!is_recording_) {
    res->success = false;
    res->message = "Not recording";
    return;
  }

  try {
    // Stop recording first
    subscriptions_.clear();
    writer_.reset();
    type_for_topic_.clear();
    is_recording_ = false;

    delete_bag_directory(current_bag_uri_);

    current_bag_uri_.clear();

    res->success = true;
    res->message = "Recording stopped and bag deleted";

    RCLCPP_INFO(this->get_logger(), "Recording stopped and bag deleted");
  } catch (const std::exception & e) {
    res->success = false;
    res->message = std::string("Failed to stop recording and delete bag: ") + e.what();
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
