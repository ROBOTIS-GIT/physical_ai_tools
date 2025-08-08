#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

#include <rosbag2_storage/topic_metadata.hpp>

#include "rosbag_recorder/joint_states_recorder.hpp"

JointStatesRecorder::JointStatesRecorder()
: rclcpp::Node("joint_states_recorder")
{
  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  // Open the bag file with a timestamp-based name
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "joint_states_recording_" << std::put_time(std::localtime(&now_c), "%Y_%m_%d_%H_%M_%S");
  writer_->open(ss.str());

  // Create topic metadata
  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = "joint_states";
  topic_metadata.type = "sensor_msgs/msg/JointState";
  topic_metadata.serialization_format = rmw_get_serialization_format();

  // Register topic in bag
  writer_->create_topic(topic_metadata);

  // Create the subscription
  subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      this->joint_states_callback(msg);
    });

  RCLCPP_INFO(get_logger(), "Started recording /joint_states to bag: %s", ss.str().c_str());
}

void JointStatesRecorder::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  writer_->write(*msg, "joint_states", msg->header.stamp);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatesRecorder>());
  rclcpp::shutdown();
  return 0;
}