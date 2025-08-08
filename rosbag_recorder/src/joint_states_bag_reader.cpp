#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStatesBagReader : public rclcpp::Node {
public:
  explicit JointStatesBagReader(const std::string & bag_filename)
  : rclcpp::Node("joint_states_bag_reader")
  {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_filename;
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);

    timer_ = this->create_wall_timer(50ms, [this]() { return this->timer_callback(); });
    RCLCPP_INFO(this->get_logger(), "Opened bag: %s", bag_filename.c_str());
  }

private:
  void timer_callback()
  {
    if (!reader_->has_next()) {
      RCLCPP_INFO_ONCE(this->get_logger(), "Reached end of bag");
      return;
    }

    // Read next serialized message
    rosbag2_storage::SerializedBagMessageSharedPtr serialized = reader_->read_next();

    // Only process joint_states topic (with or without leading slash)
    if (!(serialized->topic_name == "/joint_states" || serialized->topic_name == "joint_states")) {
      return;
    }

    // Deserialize to sensor_msgs/JointState
    rclcpp::SerializedMessage cdr(*serialized->serialized_data);
    auto msg = std::make_shared<sensor_msgs::msg::JointState>();
    serialization_.deserialize_message(&cdr, msg.get());

    // Print a concise line with time, names, and first few positions
    const auto & stamp = msg->header.stamp;
    std::cout << "t=" << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec
              << " topic=joint_states"
              << " names=[";
    for (size_t i = 0; i < msg->name.size(); ++i) {
      std::cout << msg->name[i];
      if (i + 1 < msg->name.size()) std::cout << ",";
    }
    std::cout << "] positions=[";
    for (size_t i = 0; i < msg->position.size(); ++i) {
      std::cout << msg->position[i];
      if (i + 1 < msg->position.size()) std::cout << ",";
    }
    std::cout << "]\n";
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Serialization<sensor_msgs::msg::JointState> serialization_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag_path>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatesBagReader>(argv[1]));
  rclcpp::shutdown();
  return 0;
}


