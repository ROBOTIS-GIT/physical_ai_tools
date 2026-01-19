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
// Author: Sisyphus (AI Agent)

#include "rosbag_recorder/synced_image_bag_recorder.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cv_bridge/cv_bridge.hpp>

namespace rosbag_recorder
{

SyncedImageBagRecorder::SyncedImageBagRecorder()
: rclcpp::Node("synced_image_bag_recorder")
{
  RCLCPP_INFO(this->get_logger(), "Starting synced image bag recorder node");

  send_command_srv_ = this->create_service<rosbag_recorder::srv::SendCommand>(
    "rosbag_recorder/send_command",
    std::bind(
      &SyncedImageBagRecorder::handle_send_command, this,
      std::placeholders::_1, std::placeholders::_2));

  encoding_status_pub_ = this->create_publisher<rosbag_recorder::msg::EncodingStatus>(
    "rosbag_recorder/encoding_status", 10);
}

void SyncedImageBagRecorder::handle_send_command(
  const std::shared_ptr<rosbag_recorder::srv::SendCommand::Request> req,
  std::shared_ptr<rosbag_recorder::srv::SendCommand::Response> res)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  RCLCPP_INFO(this->get_logger(), "Received command: %d", req->command);

  try {
    switch (req->command) {
      case rosbag_recorder::srv::SendCommand::Request::PREPARE:
        if (req->topics.empty()) {
          res->success = false;
          res->message = "PREPARE requires topics parameter";
          return;
        }
        if (req->uri.empty()) {
          res->success = false;
          res->message = "PREPARE requires uri parameter";
          return;
        }
        handle_prepare(req->topics, req->uri);
        res->success = true;
        res->message = "PREPARE completed, all resources initialized";
        break;
      case rosbag_recorder::srv::SendCommand::Request::START:
        handle_start();
        res->success = true;
        res->message = "Recording started immediately";
        break;
      case rosbag_recorder::srv::SendCommand::Request::STOP:
        handle_stop();
        res->success = true;
        res->message = "Recording stopped, encoding started";
        break;
      case rosbag_recorder::srv::SendCommand::Request::STOP_AND_DELETE:
        handle_stop_and_delete();
        res->success = true;
        res->message = "Recording stopped and bag deleted";
        break;
      case rosbag_recorder::srv::SendCommand::Request::FINISH:
        handle_finish();
        res->success = true;
        res->message = "Recording finished";
        break;
      default:
        res->success = false;
        res->message = "Invalid command";
        RCLCPP_ERROR(this->get_logger(), "Invalid command: %d", req->command);
        break;
    }
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Failed to execute command: %s", e.what());
  }
}

bool SyncedImageBagRecorder::is_image_topic(const std::string & topic_type) const
{
  return topic_type == "sensor_msgs/msg/Image" ||
         topic_type == "sensor_msgs/msg/CompressedImage";
}

void SyncedImageBagRecorder::handle_prepare(
  const std::vector<std::string> & topics,
  const std::string & uri)
{
  RCLCPP_INFO(this->get_logger(), "Prepare Rosbag recording");

  if (is_recording_) {
    throw std::runtime_error("Already recording");
  }

  if (image_compressor_ && image_compressor_->is_encoding()) {
    throw std::runtime_error("Encoding in progress, please wait");
  }

  if (topics.empty()) {
    throw std::runtime_error("Topics are required");
  }

  if (uri.empty()) {
    throw std::runtime_error("Bag URI is required");
  }

  try {
    topics_to_record_ = topics;
    current_bag_uri_ = uri;
    image_topics_.clear();
    non_image_topics_.clear();

    // 1. Classify topics
    auto names_and_types = this->get_topic_names_and_types();

    for (const auto & topic : topics_to_record_) {
      auto it = names_and_types.find(topic);
      if (it == names_and_types.end()) {
        continue;
      }

      const std::string & type = it->second.front();
      type_for_topic_[topic] = type;

      if (is_image_topic(type)) {
        image_topics_.push_back(topic);
      } else {
        non_image_topics_.push_back(topic);
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "PREPARE: Classified %zu image topics, %zu non-image topics",
      image_topics_.size(), non_image_topics_.size());

    // 2. PRE-OPEN bag writer (eliminates START delay!)
    delete_bag_directory(current_bag_uri_);
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(current_bag_uri_);
    RCLCPP_INFO(this->get_logger(), "Bag writer pre-opened: %s", uri.c_str());

    // 3. PRE-CREATE topic metadata
    auto missing_topics = get_missing_topics(names_and_types);
    if (!missing_topics.empty()) {
      writer_.reset();
      type_for_topic_.clear();
      delete_bag_directory(current_bag_uri_);
      current_bag_uri_.clear();

      std::ostringstream oss;
      oss << "Types not found for topics:";
      for (const auto & t : missing_topics) {
        oss << " " << t;
      }
      RCLCPP_ERROR(this->get_logger(), "Failed to prepare: %s", oss.str().c_str());
      throw std::runtime_error(oss.str());
    }

    create_topics_in_bag(names_and_types);

    // 4. PRE-INITIALIZE compressor (eliminates START delay!)
    std::string video_output_dir = current_bag_uri_ + "/videos";
    ImageCompressorRaw::Config config;
    config.target_fps = 30.0;

    image_compressor_ = std::make_unique<ImageCompressorRaw>(
      video_output_dir, image_topics_, config);

    image_compressor_->set_frame_callback(
      std::bind(&SyncedImageBagRecorder::on_frame, this, std::placeholders::_1));

    image_compressor_->set_encoding_complete_callback(
      std::bind(
        &SyncedImageBagRecorder::on_encoding_complete, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Image compressor pre-initialized");

    // 5. Create subscriptions (DDS negotiation happens here)
    create_subscriptions();
    RCLCPP_INFO(
      this->get_logger(),
      "Subscriptions created, waiting for DDS negotiation...");

    // 6. Initialize frame counts
    frame_counts_.clear();
    for (const auto & topic : image_topics_) {
      frame_counts_[topic] = 0;
    }

    // 7. is_recording_ remains false (data will be dropped until START)
    RCLCPP_INFO(
      this->get_logger(),
      "PREPARE complete. All initialization done. Ready for START command.");

  } catch (const std::exception & e) {
    writer_.reset();
    image_compressor_.reset();
    type_for_topic_.clear();
    throw std::runtime_error(
            std::string("Failed to prepare recording: ") + e.what());
  }
}

void SyncedImageBagRecorder::handle_start()
{
  RCLCPP_INFO(this->get_logger(), "Start Rosbag recording");

  // All initialization was completed in PREPARE!
  // Verify resources are ready
  if (!writer_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "START failed: bag writer not initialized. Call PREPARE first.");
    throw std::runtime_error("Bag writer not initialized");
  }

  if (image_topics_.size() > 0 && !image_compressor_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "START failed: image compressor not initialized. Call PREPARE first.");
    throw std::runtime_error("Image compressor not initialized");
  }

  if (is_recording_) {
    throw std::runtime_error("Already recording");
  }

  if (image_compressor_ && image_compressor_->is_encoding()) {
    throw std::runtime_error("Encoding in progress, cannot start new recording");
  }

  try {
    // Start image compressor recording
    if (image_compressor_) {
      image_compressor_->start_recording();
    }

    // Simply enable recording (zero initialization delay!)
    is_recording_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Recording started IMMEDIATELY (no initialization delay)");

  } catch (const std::exception & e) {
    throw std::runtime_error(
            std::string("Failed to start recording: ") + e.what());
  }
}

void SyncedImageBagRecorder::handle_stop()
{
  RCLCPP_INFO(this->get_logger(), "Stop Rosbag recording");

  if (!is_recording_) {
    throw std::runtime_error("Not recording");
  }

  is_recording_ = false;

  if (image_compressor_) {
    image_compressor_->stop_recording();
  }

  if (writer_) {
    writer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Recording stopped, encoding in background");
}

void SyncedImageBagRecorder::handle_stop_and_delete()
{
  RCLCPP_INFO(this->get_logger(), "Stop and delete Rosbag recording");

  if (!is_recording_) {
    throw std::runtime_error("Not recording");
  }

  is_recording_ = false;

  if (image_compressor_) {
    image_compressor_.reset();
  }

  writer_.reset();
  type_for_topic_.clear();

  delete_bag_directory(current_bag_uri_);

  current_bag_uri_.clear();

  RCLCPP_INFO(this->get_logger(), "Recording stopped and bag deleted");
}

void SyncedImageBagRecorder::handle_finish()
{
  RCLCPP_INFO(this->get_logger(), "Finish Rosbag recording");

  generic_subscriptions_.clear();
  image_subscriptions_.clear();

  if (is_recording_) {
    handle_stop();
  }
}

void SyncedImageBagRecorder::on_encoding_complete(bool success, const std::string & message)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Encoding complete: success=%d, message=%s, bag_path=%s",
    success, message.c_str(), current_bag_uri_.c_str());

  write_stats_report();

  rosbag_recorder::msg::EncodingStatus status_msg;
  status_msg.header.stamp = this->now();
  status_msg.success = success;
  status_msg.message = message;
  status_msg.bag_path = current_bag_uri_;

  encoding_status_pub_->publish(status_msg);

  type_for_topic_.clear();
}

std::vector<std::string> SyncedImageBagRecorder::get_missing_topics(
  const std::map<std::string, std::vector<std::string>> & names_and_types)
{
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

void SyncedImageBagRecorder::create_topics_in_bag(
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

    rosbag2_storage::TopicMetadata meta;

    if (is_image_topic(type)) {
      meta.name = topic + "/metadata";
      meta.type = "rosbag_recorder/msg/ImageMetadata";
    } else {
      meta.name = topic;
      meta.type = type;
      type_for_topic_[topic] = type;
    }

    meta.serialization_format = rmw_get_serialization_format();
    writer_->create_topic(meta);
  }
}

void SyncedImageBagRecorder::delete_bag_directory(const std::string & bag_uri)
{
  if (bag_uri.empty()) {
    return;
  }

  std::filesystem::path bag_path(bag_uri);
  if (std::filesystem::exists(bag_path)) {
    std::filesystem::remove_all(bag_path);
    RCLCPP_INFO(this->get_logger(), "Deleted bag directory: %s", bag_uri.c_str());
  }
}

void SyncedImageBagRecorder::create_subscriptions()
{
  RCLCPP_INFO(this->get_logger(), "Creating subscriptions");

  generic_subscriptions_.clear();
  image_subscriptions_.clear();

  for (const auto & topic : non_image_topics_) {
    auto it = type_for_topic_.find(topic);
    if (it == type_for_topic_.end()) {
      continue;
    }

    const std::string & type = it->second;
    auto options = rclcpp::SubscriptionOptions();
    auto sub = this->create_generic_subscription(
      topic,
      type,
      rclcpp::QoS(100),
      [this, topic](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
        this->handle_serialized_message(topic, serialized_msg);
      },
      options);
    generic_subscriptions_.push_back(sub);
  }

  for (const auto & topic : image_topics_) {
    auto it = type_for_topic_.find(topic);
    if (it == type_for_topic_.end()) {
      continue;
    }

    const std::string & type = it->second;

    // Handle CompressedImage as generic subscription (serialize d form)
    if (type == "sensor_msgs/msg/CompressedImage") {
      auto options = rclcpp::SubscriptionOptions();
      auto sub = this->create_generic_subscription(
        topic,
        type,
        rclcpp::QoS(100),
        [this, topic](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
          this->handle_compressed_image_message(topic, serialized_msg);
        },
        options);
      generic_subscriptions_.push_back(sub);
    } else {
      // Handle regular Image
      auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        topic,
        rclcpp::QoS(100),
        [this, topic](const sensor_msgs::msg::Image::SharedPtr msg) {
          this->handle_image_message(topic, msg);
        });
      image_subscriptions_.push_back(sub);
    }
  }
}

void SyncedImageBagRecorder::handle_serialized_message(
  const std::string & topic,
  const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  // Drop messages if not recording (PREPARE phase)
  if (!is_recording_ || !writer_) {
    return;
  }

  const auto it = type_for_topic_.find(topic);
  if (it == type_for_topic_.end()) {
    return;
  }

  try {
    // Extract original timestamp from message
    rclcpp::Time msg_time = extract_timestamp_from_message(serialized_msg, topic);

    // Write with ORIGINAL timestamp (not this->now()!)
    const std::string & type = it->second;
    writer_->write(serialized_msg, topic, type, msg_time);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Recorded %s at timestamp %ld.%09ld",
      topic.c_str(),
      msg_time.seconds(),
      msg_time.nanoseconds() % 1000000000L);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to record message on %s: %s",
      topic.c_str(), e.what());
  }
}

void SyncedImageBagRecorder::handle_image_message(
  const std::string & topic,
  const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  // Drop images if not recording (PREPARE phase)
  if (!is_recording_ || !image_compressor_) {
    return;
  }

  try {
    // Compressor uses original timestamp from image_msg->header.stamp
    image_compressor_->add_frame(topic, image_msg);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Compressed image on %s at timestamp %u.%u",
      topic.c_str(),
      image_msg->header.stamp.sec,
      image_msg->header.stamp.nanosec);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to compress image on %s: %s",
      topic.c_str(), e.what());
  }
}

void SyncedImageBagRecorder::handle_compressed_image_message(
  const std::string & topic,
  const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg)
{
  // Drop images if not recording (PREPARE phase)
  if (!is_recording_ || !image_compressor_) {
    return;
  }

  try {
    // Deserialize CompressedImage
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
    sensor_msgs::msg::CompressedImage compressed_msg;
    serializer.deserialize_message(serialized_msg.get(), &compressed_msg);

    // Convert CompressedImage to Image using cv_bridge
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(compressed_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "cv_bridge exception on %s: %s",
        topic.c_str(), e.what());
      return;
    }

    // Create Image message with original timestamp
    auto image_msg = cv_ptr->toImageMsg();
    image_msg->header = compressed_msg.header;  // Preserve original timestamp

    // Pass to compressor
    image_compressor_->add_frame(topic, image_msg);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Decompressed and compressed image on %s at timestamp %u.%u",
      topic.c_str(),
      compressed_msg.header.stamp.sec,
      compressed_msg.header.stamp.nanosec);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to handle compressed image on %s: %s",
      topic.c_str(), e.what());
  }
}

void SyncedImageBagRecorder::on_frame(const FrameOutput & output)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  if (!is_recording_ || !writer_) {
    return;
  }

  rosbag_recorder::msg::ImageMetadata metadata_msg;

  rclcpp::Time timestamp(output.timestamp_ns);
  metadata_msg.header.stamp = timestamp;
  metadata_msg.header.frame_id = output.topic;

  metadata_msg.frame_index = frame_counts_[output.topic]++;
  metadata_msg.width = output.width;
  metadata_msg.height = output.height;
  metadata_msg.encoding = "";
  metadata_msg.source_topic = output.topic;

  std::string sanitized = output.topic;
  std::replace(sanitized.begin(), sanitized.end(), '/', '_');
  if (!sanitized.empty() && sanitized[0] == '_') {
    sanitized = sanitized.substr(1);
  }
  metadata_msg.video_file_path = "videos/" + sanitized + ".mp4";

  rclcpp::Serialization<rosbag_recorder::msg::ImageMetadata> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(&metadata_msg, &serialized_msg);

  std::string metadata_topic = output.topic + "/metadata";
  std::string metadata_type = "rosbag_recorder/msg/ImageMetadata";

  writer_->write(
    std::make_shared<rclcpp::SerializedMessage>(serialized_msg),
    metadata_topic,
    metadata_type,
    timestamp);
}

void SyncedImageBagRecorder::write_stats_report()
{
  if (!image_compressor_ || current_bag_uri_.empty()) {
    return;
  }

  auto stats = image_compressor_->get_stats();

  std::string report_path = current_bag_uri_ + "/stats_report.json";
  std::ofstream report(report_path);

  report << "{\n";
  report << "  \"total_frames\": " << stats.total_frames << ",\n";
  report << "  \"recording_duration_sec\": "
         << (stats.recording_end_ns - stats.recording_start_ns) / 1e9 << ",\n";
  report << "  \"peak_ram_usage_mb\": "
         << stats.peak_ram_usage_bytes / (1024.0 * 1024.0) << ",\n";
  report << "  \"topics\": {\n";

  bool first = true;
  for (const auto & [topic, topic_stats] : stats.topic_stats) {
    if (!first) {
      report << ",\n";
    }
    first = false;

    report << "    \"" << topic << "\": {\n";
    report << "      \"received_frames\": " << topic_stats.received_frames << "\n";
    report << "    }";
  }

  report << "\n  },\n";

  report << "  \"frame_counts\": {\n";
  first = true;
  for (const auto & [topic, count] : frame_counts_) {
    if (!first) {
      report << ",\n";
    }
    first = false;
    report << "    \"" << topic << "\": " << count;
  }
  report << "\n  }\n";

  report << "}\n";
  report.close();

  RCLCPP_INFO(this->get_logger(), "Stats report written to: %s", report_path.c_str());
}

rclcpp::Time SyncedImageBagRecorder::extract_timestamp_from_message(
  const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
  const std::string & topic)
{
  // Try to extract timestamp from std_msgs/Header
  // Most ROS2 messages (sensor_msgs, geometry_msgs) contain header.stamp

  try {
    // Get message type
    const auto it = type_for_topic_.find(topic);
    if (it == type_for_topic_.end()) {
      return this->now();
    }
    const std::string & msg_type = it->second;

    // Check if message type typically has a header
    const std::vector<std::string> header_types = {
      "sensor_msgs/msg/Image",
      "sensor_msgs/msg/CompressedImage",
      "sensor_msgs/msg/CameraInfo",
      "sensor_msgs/msg/JointState",
      "sensor_msgs/msg/LaserScan",
      "sensor_msgs/msg/Imu",
      "geometry_msgs/msg/TwistStamped",
      "geometry_msgs/msg/PoseStamped",
      "nav_msgs/msg/Odometry"
    };

    bool has_header = false;
    for (const auto & type : header_types) {
      if (msg_type.find(type) != std::string::npos) {
        has_header = true;
        break;
      }
    }

    if (has_header) {
      // Extract timestamp from CDR serialized data
      // Header structure: frame_id (string) + stamp (time)
      auto & buffer = serialized_msg->get_rcl_serialized_message();

      if (buffer.buffer_length >= 16) {
        // Skip frame_id string (4 bytes length + variable string)
        size_t offset = 4;
        uint32_t frame_id_len = 0;
        memcpy(&frame_id_len, buffer.buffer + offset, sizeof(uint32_t));
        offset += 4 + frame_id_len;

        // Read timestamp (8 bytes: int32 sec + uint32 nanosec)
        if (offset + 8 <= buffer.buffer_length) {
          int32_t sec = 0;
          uint32_t nanosec = 0;
          memcpy(&sec, buffer.buffer + offset, sizeof(int32_t));
          memcpy(&nanosec, buffer.buffer + offset + 4, sizeof(uint32_t));

          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[DEBUG] Extracted timestamp for %s: %d.%09u",
            topic.c_str(), sec, nanosec);

          return rclcpp::Time(sec, nanosec);
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to extract timestamp from %s: %s (using current time)",
      topic.c_str(), e.what());
  }

  // Fallback: use current time
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "[DEBUG] Using this->now() for %s (extraction failed or no header)",
    topic.c_str());
  return this->now();
}

}  // namespace rosbag_recorder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosbag_recorder::SyncedImageBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
