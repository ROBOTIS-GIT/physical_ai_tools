// Copyright 2025
#ifndef ROSBAG_RECORDER__JOINT_STATES_RECORDER_HPP_
#define ROSBAG_RECORDER__JOINT_STATES_RECORDER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosbag2_cpp/writer.hpp>

class JointStatesRecorder : public rclcpp::Node
{
public:
  JointStatesRecorder();

private:
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

#endif  // ROSBAG_RECORDER__JOINT_STATES_RECORDER_HPP_


