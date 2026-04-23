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
// Author: Woojin Wie, Kiwoong Park


#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "rosbag_recorder/service_bag_recorder.hpp"


ServiceBagRecorder::ServiceBagRecorder()
: rclcpp::Node("service_bag_recorder")
{
  RCLCPP_INFO(this->get_logger(), "Starting rosbag recorder node");

  // Preflight check: how long to wait for the first message on each requested
  // topic before failing the START command. Set <= 0 to disable.
  this->declare_parameter<int>("preflight_wait_timeout_ms", 2000);
  preflight_wait_timeout_ms_ =
    this->get_parameter("preflight_wait_timeout_ms").as_int();
  RCLCPP_INFO(
    this->get_logger(),
    "Preflight wait timeout: %d ms (<=0 disables the check)",
    preflight_wait_timeout_ms_);

  // Live per-topic monitor parameters. The monitor publishes a snapshot at
  // ``monitor_publish_hz`` while a recording is in progress; a topic is
  // judged STALLED when no message has arrived in ``monitor_stall_window_ms``
  // OR its rate falls below ``baseline * monitor_stall_ratio``. SLOW is the
  // halfway state at ``baseline * monitor_slow_ratio``. The baseline is an
  // EMA of healthy rates with weight ``monitor_ema_alpha``; ratios are not
  // applied below ``monitor_min_baseline_hz`` to avoid spurious alarms on
  // intrinsically slow topics.
  this->declare_parameter<double>("monitor_publish_hz", 1.0);
  this->declare_parameter<int>("monitor_stall_window_ms", 2000);
  this->declare_parameter<double>("monitor_stall_ratio", 0.2);
  this->declare_parameter<double>("monitor_slow_ratio", 0.6);
  this->declare_parameter<double>("monitor_ema_alpha", 0.2);
  this->declare_parameter<double>("monitor_min_baseline_hz", 1.0);
  monitor_publish_hz_ = this->get_parameter("monitor_publish_hz").as_double();
  monitor_stall_window_ms_ = this->get_parameter("monitor_stall_window_ms").as_int();
  monitor_stall_ratio_ = this->get_parameter("monitor_stall_ratio").as_double();
  monitor_slow_ratio_ = this->get_parameter("monitor_slow_ratio").as_double();
  monitor_ema_alpha_ = this->get_parameter("monitor_ema_alpha").as_double();
  monitor_min_baseline_hz_ = this->get_parameter("monitor_min_baseline_hz").as_double();
  this->declare_parameter<int>("monitor_warmup_ms", 3000);
  monitor_warmup_ms_ = this->get_parameter("monitor_warmup_ms").as_int();
  RCLCPP_INFO(
    this->get_logger(),
    "Monitor: %.2f Hz, stall_window=%d ms, slow=%.2f, stall=%.2f, "
    "ema_alpha=%.2f, min_baseline=%.2f Hz, warmup=%d ms",
    monitor_publish_hz_, monitor_stall_window_ms_, monitor_slow_ratio_,
    monitor_stall_ratio_, monitor_ema_alpha_, monitor_min_baseline_hz_,
    monitor_warmup_ms_);

  // Create callback groups for parallel processing
  camera_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  joint_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  other_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  service_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create service with dedicated callback group
  send_command_srv_ = this->create_service<rosbag_recorder::srv::SendCommand>(
    "rosbag_recorder/send_command",
    std::bind(
      &ServiceBagRecorder::handle_send_command, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_callback_group_);

  // Live monitor publisher + 1 Hz timer (parked on the "other" callback
  // group so it never blocks camera/joint message processing).
  monitor_pub_ = this->create_publisher<rosbag_recorder::msg::RecordingMonitor>(
    "rosbag_recorder/monitor", rclcpp::QoS(5));
  if (monitor_publish_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / monitor_publish_hz_);
    monitor_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ServiceBagRecorder::monitor_tick, this),
      other_callback_group_);
  }

  RCLCPP_INFO(this->get_logger(), "Rosbag recorder initialized with MultiThreadedExecutor support");
}

void ServiceBagRecorder::handle_send_command(
  const std::shared_ptr<rosbag_recorder::srv::SendCommand::Request> req,
  std::shared_ptr<rosbag_recorder::srv::SendCommand::Response> res)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  RCLCPP_INFO(this->get_logger(), "Received command: %d", req->command);

  try {
    switch (req->command) {
      case rosbag_recorder::srv::SendCommand::Request::PREPARE:
        handle_prepare(req->topics);
        res->success = true;
        res->message = "Recording prepared";
        break;
      case rosbag_recorder::srv::SendCommand::Request::START:
        handle_start(req->uri);
        res->success = true;
        res->message = "Recording started";
        break;
      case rosbag_recorder::srv::SendCommand::Request::STOP:
        handle_stop();
        res->success = true;
        res->message = "Recording stopped";
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

void ServiceBagRecorder::handle_prepare(const std::vector<std::string> & topics)
{
  RCLCPP_INFO(this->get_logger(), "Prepare Rosbag recording");

  if (is_recording_.load(std::memory_order_acquire)) {
    throw std::runtime_error("Already recording");
  }

  if (topics.empty()) {
    throw std::runtime_error("Topics are required");
  }

  // Deduplicate the topic list.
  std::vector<std::string> deduped;
  {
    std::unordered_set<std::string> seen;
    for (const auto & t : topics) {
      if (seen.insert(t).second) {
        deduped.push_back(t);
      }
    }
  }

  // Always recreate subscriptions on prepare. The previous skip-when-same
  // optimization preserved EMA baselines across episodes but also kept
  // broken subscriptions alive (e.g. QoS mismatch, late-appearing publisher),
  // so a user refresh could not recover them. Warmup (monitor_warmup_ms_)
  // suppresses false alarms during the first few seconds after recreation.
  topics_to_record_ = deduped;

  // Clean up any previous subscriptions / state.
  type_for_topic_.clear();
  camera_topics_.clear();
  joint_topics_.clear();
  generic_subscriptions_.clear();
  messages_received_ = 0;
  messages_written_ = 0;

  // Resolve topic types from the ROS graph. Topics not yet available
  // (e.g. robot not running) are skipped with a warning — they will be
  // caught by the preflight check when START is called.
  auto names_and_types = this->get_topic_names_and_types();
  RCLCPP_INFO(this->get_logger(), "Found %zu active topics in system", names_and_types.size());

  auto missing_topics = get_missing_topics(names_and_types);
  if (!missing_topics.empty()) {
    std::ostringstream oss;
    oss << "Skipping " << missing_topics.size() << " topics not yet in graph:";
    for (const auto & t : missing_topics) {
      oss << " " << t;
    }
    RCLCPP_WARN(this->get_logger(), "%s", oss.str().c_str());
  }

  // Categorize topics and populate type_for_topic_ (only available ones).
  for (const auto & topic : topics_to_record_) {
    auto it = names_and_types.find(topic);
    if (it != names_and_types.end() && !it->second.empty()) {
      const std::string & type = it->second.front();
      type_for_topic_[topic] = type;

      if (topic.find("image") != std::string::npos ||
        topic.find("camera") != std::string::npos)
      {
        camera_topics_.insert(topic);
      } else if (topic.find("joint") != std::string::npos ||
        topic.find("arm") != std::string::npos ||
        topic.find("head") != std::string::npos ||
        topic.find("lift") != std::string::npos)
      {
        joint_topics_.insert(topic);
      }
    }
  }

  // Create subscriptions — messages start flowing immediately so the
  // monitor can show live Hz before the user presses record.
  create_subscriptions();

  // Seed one fresh metric per topic with subscribe timestamp.
  const auto now_ns = static_cast<uint64_t>(
    std::chrono::steady_clock::now().time_since_epoch().count());
  per_topic_metrics_.clear();
  for (const auto & topic : topics_to_record_) {
    auto m = std::make_unique<TopicMetric>();
    m->subscribe_start_ns = now_ns;
    per_topic_metrics_.emplace(topic, std::move(m));
  }

  // Mark monitor warmup start so status checks have a grace period.
  monitor_start_ns_ = now_ns;

  RCLCPP_INFO(
    this->get_logger(),
    "Recording prepared: %zu topics subscribed, monitor active",
    topics_to_record_.size());
}

void ServiceBagRecorder::handle_start(const std::string & uri)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Start Rosbag recording: uri=%s topics_to_record=%zu subscriptions=%zu",
    uri.c_str(), topics_to_record_.size(), generic_subscriptions_.size());

  if (is_recording_.load(std::memory_order_acquire)) {
    throw std::runtime_error("Already recording");
  }

  if (uri.empty()) {
    throw std::runtime_error("Bag URI is required");
  }

  if (topics_to_record_.empty()) {
    throw std::runtime_error("No topics configured - PREPARE must be called first");
  }

  try {
    current_bag_uri_ = uri;

    // Re-resolve any topics that were missing during prepare.
    auto names_and_types = this->get_topic_names_and_types();
    {
      bool added_new = false;
      for (const auto & topic : topics_to_record_) {
        if (type_for_topic_.count(topic) > 0) {
          continue;  // already subscribed
        }
        auto it = names_and_types.find(topic);
        if (it != names_and_types.end() && !it->second.empty()) {
          const std::string & type = it->second.front();
          type_for_topic_[topic] = type;
          if (topic.find("image") != std::string::npos ||
            topic.find("camera") != std::string::npos) {
            camera_topics_.insert(topic);
          } else if (topic.find("joint") != std::string::npos ||
            topic.find("arm") != std::string::npos ||
            topic.find("head") != std::string::npos ||
            topic.find("lift") != std::string::npos) {
            joint_topics_.insert(topic);
          }
          added_new = true;
          RCLCPP_INFO(this->get_logger(), "Late-discovered topic: %s [%s]",
            topic.c_str(), type.c_str());
        }
      }
      if (added_new) {
        // Recreate all subscriptions to include newly discovered topics.
        create_subscriptions();
        // Seed metrics for new topics.
        const auto now_ns = static_cast<uint64_t>(
          std::chrono::steady_clock::now().time_since_epoch().count());
        for (const auto & topic : topics_to_record_) {
          if (per_topic_metrics_.count(topic) == 0) {
            auto m = std::make_unique<TopicMetric>();
            m->subscribe_start_ns = now_ns;
            per_topic_metrics_.emplace(topic, std::move(m));
          }
        }
      }
    }

    // Check for topics still missing from the graph.
    auto still_missing = get_missing_topics(names_and_types);
    if (!still_missing.empty()) {
      std::ostringstream oss;
      oss << "Topics not available in ROS graph:";
      for (const auto & t : still_missing) {
        oss << " " << t;
      }
      throw std::runtime_error(oss.str());
    }

    // Preflight: confirm every topic is actually publishing data.
    auto inactive_topics = wait_for_first_messages();
    if (!inactive_topics.empty()) {
      std::ostringstream oss;
      oss << "No messages received within " << preflight_wait_timeout_ms_
          << "ms from topics:";
      for (const auto & t : inactive_topics) {
        oss << " " << t;
      }
      throw std::runtime_error(oss.str());
    }

    // Open the bag writer.
    delete_bag_directory(current_bag_uri_);

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = current_bag_uri_;
    storage_options.storage_id = STORAGE_ID;  // "mcap"
    storage_options.max_cache_size = CACHE_SIZE_BYTES;  // 500MB
    storage_options.max_bagfile_size = 0;

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options);

    create_topics_in_bag(names_and_types);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to start recording: ") + e.what());
  }

  // Reset per-episode counters.
  messages_received_ = 0;
  messages_written_ = 0;

  is_recording_.store(true, std::memory_order_release);

  RCLCPP_INFO(
    this->get_logger(), "Recording started: uri=%s topics=%zu storage=%s cache=%zuMB",
    current_bag_uri_.c_str(), topics_to_record_.size(), STORAGE_ID,
    CACHE_SIZE_BYTES / (1024 * 1024));
}

void ServiceBagRecorder::handle_stop()
{
  RCLCPP_INFO(this->get_logger(), "Stop Rosbag recording");

  // Handle gracefully when not recording (e.g., if START failed)
  if (!is_recording_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(this->get_logger(), "Stop called but not recording - nothing to stop");
    return;
  }

  try {
    // Set flag first to stop callbacks from writing
    is_recording_.store(false, std::memory_order_release);

    // Log statistics before closing
    log_statistics();

    writer_.reset();
    current_bag_uri_.clear();

    RCLCPP_INFO(this->get_logger(), "Recording stopped (subscriptions kept alive)");
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to stop recording: ") + e.what());
  }
}

void ServiceBagRecorder::handle_stop_and_delete()
{
  RCLCPP_INFO(this->get_logger(), "Stop and delete Rosbag recording");

  // Handle gracefully when not recording (e.g., Cancel pressed before recording started)
  if (!is_recording_.load(std::memory_order_acquire)) {
    RCLCPP_INFO(this->get_logger(), "Not recording, nothing to delete");
    return;
  }

  try {
    // Set flag first to stop callbacks from writing
    is_recording_.store(false, std::memory_order_release);

    // Log statistics
    log_statistics();

    writer_.reset();

    delete_bag_directory(current_bag_uri_);

    current_bag_uri_.clear();

    RCLCPP_INFO(this->get_logger(), "Recording stopped and bag deleted");
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to stop recording and delete bag: ") + e.what());
  }
}

void ServiceBagRecorder::handle_finish()
{
  RCLCPP_INFO(this->get_logger(), "Finish Rosbag recording");

  // Log final statistics
  log_statistics();

  // Note: Don't clear subscriptions here - keep them for next episode recording
  // Subscriptions will only be cleared when:
  // 1. A new PREPARE command is received (robot type changed)
  // 2. The node is destroyed (program shutdown)

  if (is_recording_.load(std::memory_order_acquire)) {
    is_recording_.store(false, std::memory_order_release);
    writer_.reset();
    current_bag_uri_.clear();
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
    if (it == names_and_types.end() || it->second.empty()) {
      throw std::runtime_error("Type not found while creating bag topic: " + topic);
    }
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
  RCLCPP_INFO(this->get_logger(), "Creating subscriptions with callback groups");

  generic_subscriptions_.clear();

  // Create generic subscriptions for all topics
  for (const auto & [topic, type] : type_for_topic_) {
    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = get_callback_group_for_topic(topic);

    auto qos = get_qos_for_topic(topic);

    auto sub = this->create_generic_subscription(
      topic,
      type,
      qos,
      [this, topic](
        std::shared_ptr<rclcpp::SerializedMessage> serialized_msg,
        const rclcpp::MessageInfo & message_info) {
        this->handle_serialized_message(topic, serialized_msg, message_info);
      },
      options);

    generic_subscriptions_.push_back(sub);

    std::string group_name = "other";
    if (camera_topics_.count(topic)) {
      group_name = "camera";
    } else if (joint_topics_.count(topic)) {
      group_name = "joint";
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to topic: %s (group: %s, depth: %zu)",
      topic.c_str(), group_name.c_str(), qos.depth());
  }
}

std::vector<std::string> ServiceBagRecorder::wait_for_first_messages()
{
  if (preflight_wait_timeout_ms_ <= 0) {
    RCLCPP_INFO(this->get_logger(), "Preflight check disabled (timeout <= 0)");
    return {};
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Preflight: waiting up to %d ms for first message on each of %zu topics",
    preflight_wait_timeout_ms_, topics_to_record_.size());

  {
    std::scoped_lock<std::mutex> lk(preflight_mutex_);
    preflight_received_topics_.clear();
  }
  preflight_active_.store(true, std::memory_order_release);

  std::vector<std::string> missing;
  {
    std::unique_lock<std::mutex> lk(preflight_mutex_);
    preflight_cv_.wait_for(
      lk,
      std::chrono::milliseconds(preflight_wait_timeout_ms_),
      [this]() {
        for (const auto & topic : topics_to_record_) {
          if (preflight_received_topics_.count(topic) == 0) {
            return false;
          }
        }
        return true;
      });

    for (const auto & topic : topics_to_record_) {
      if (preflight_received_topics_.count(topic) == 0) {
        missing.push_back(topic);
      }
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Preflight done: %zu/%zu topics produced data",
      preflight_received_topics_.size(), topics_to_record_.size());
  }

  preflight_active_.store(false, std::memory_order_release);
  return missing;
}

rclcpp::QoS ServiceBagRecorder::get_qos_for_topic(const std::string & topic)
{
  // Camera topics: large buffer for high-bandwidth data
  if (camera_topics_.count(topic)) {
    return rclcpp::QoS(rclcpp::KeepLast(2000))
           .reliable()
           .durability_volatile();
  }

  // Joint topics: medium buffer
  if (joint_topics_.count(topic)) {
    return rclcpp::QoS(rclcpp::KeepLast(1000))
           .reliable()
           .durability_volatile();
  }

  // Other topics (tf, etc.): standard buffer
  return rclcpp::QoS(rclcpp::KeepLast(500))
         .reliable()
         .durability_volatile();
}

rclcpp::CallbackGroup::SharedPtr ServiceBagRecorder::get_callback_group_for_topic(
  const std::string & topic)
{
  if (camera_topics_.count(topic)) {
    return camera_callback_group_;
  }
  if (joint_topics_.count(topic)) {
    return joint_callback_group_;
  }
  return other_callback_group_;
}

void ServiceBagRecorder::log_statistics()
{
  uint64_t received = messages_received_.load();
  uint64_t written = messages_written_.load();
  uint64_t dropped = (received > written) ? (received - written) : 0;

  RCLCPP_INFO(
    this->get_logger(),
    "Recording statistics - Received: %lu, Written: %lu, Dropped: %lu",
    received, written, dropped);

  if (dropped > 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "WARNING: %lu messages were dropped during recording!", dropped);
  }

  // Per-topic forensic dump so failed episodes leave a trail of which
  // topics actually produced data and which were silent.
  for (const auto & topic : topics_to_record_) {
    auto it = per_topic_metrics_.find(topic);
    const uint64_t cnt = (it != per_topic_metrics_.end())
      ? it->second->message_count.load(std::memory_order_relaxed)
      : 0u;
    RCLCPP_INFO(
      this->get_logger(),
      "  topic %s -> %lu messages",
      topic.c_str(), cnt);
  }
}

void ServiceBagRecorder::monitor_tick()
{
  // Publish whenever per-topic metrics exist (i.e. after PREPARE), even when
  // not actively recording — so the operator can see topic health before
  // pressing record.
  if (per_topic_metrics_.empty()) {
    return;
  }

  const auto now_ns = static_cast<uint64_t>(
    std::chrono::steady_clock::now().time_since_epoch().count());

  const uint64_t warmup_ns =
    static_cast<uint64_t>(monitor_warmup_ms_) * 1000000ULL;
  const bool in_warmup =
    (monitor_start_ns_ != 0) &&
    (now_ns - monitor_start_ns_) < warmup_ns;

  rosbag_recorder::msg::RecordingMonitor msg;
  msg.topic_names.reserve(topics_to_record_.size());
  msg.rates_hz.reserve(topics_to_record_.size());
  msg.baseline_hz.reserve(topics_to_record_.size());
  msg.seconds_since_last.reserve(topics_to_record_.size());
  msg.status.reserve(topics_to_record_.size());

  const uint64_t stall_window_ns =
    static_cast<uint64_t>(monitor_stall_window_ms_) * 1000000ULL;

  for (const auto & topic : topics_to_record_) {
    auto it = per_topic_metrics_.find(topic);
    if (it == per_topic_metrics_.end()) {
      continue;
    }
    auto & m = *it->second;

    const uint64_t count_now = m.message_count.load(std::memory_order_relaxed);
    const uint64_t last_recv_ns = m.last_recv_ns.load(std::memory_order_relaxed);

    // Compute instantaneous rate over [last_tick, now]. First tick after a
    // PREPARE has last_tick_ns == 0, so we just seed it.
    double rate_hz = 0.0;
    if (m.last_tick_ns != 0 && now_ns > m.last_tick_ns) {
      const double dt_s =
        static_cast<double>(now_ns - m.last_tick_ns) / 1e9;
      if (dt_s > 0.0) {
        const uint64_t delta = count_now - m.last_count_snapshot;
        rate_hz = static_cast<double>(delta) / dt_s;
      }
    }
    m.last_tick_ns = now_ns;
    m.last_count_snapshot = count_now;

    // If last_recv_ns > 0, check against last received timestamp.
    // If last_recv_ns == 0 (never received), check against subscribe time —
    // if we've waited longer than the stall window with zero messages, it's stalled.
    bool no_recent_msg = false;
    if (last_recv_ns != 0) {
      no_recent_msg = (now_ns > last_recv_ns) &&
        ((now_ns - last_recv_ns) > stall_window_ns);
    } else if (m.subscribe_start_ns != 0) {
      no_recent_msg = (now_ns > m.subscribe_start_ns) &&
        ((now_ns - m.subscribe_start_ns) > stall_window_ns);
    }

    bool stalled = false;
    bool slow = false;

    if (!in_warmup) {
      const double effective_baseline =
        (m.ema_hz > monitor_min_baseline_hz_) ? m.ema_hz : 0.0;

      if (no_recent_msg) {
        stalled = true;
      } else if (effective_baseline > 0.0) {
        if (rate_hz < effective_baseline * monitor_stall_ratio_) {
          stalled = true;
        } else if (rate_hz < effective_baseline * monitor_slow_ratio_) {
          slow = true;
        }
      }

      // EMA only advances on healthy ticks.
      if (!stalled && !slow && rate_hz > 0.0) {
        if (!m.ema_initialised) {
          m.ema_hz = rate_hz;
          m.ema_initialised = true;
        } else {
          m.ema_hz = (monitor_ema_alpha_ * rate_hz) +
            ((1.0 - monitor_ema_alpha_) * m.ema_hz);
        }
      }
    }
    m.stalled = stalled;

    uint8_t status_byte = 0;
    if (stalled) {
      status_byte = 2;
    } else if (slow) {
      status_byte = 1;
    }

    const float seconds_since_last = (last_recv_ns == 0)
      ? -1.0f
      : static_cast<float>(
          static_cast<double>(now_ns - last_recv_ns) / 1e9);

    msg.topic_names.push_back(topic);
    msg.rates_hz.push_back(static_cast<float>(rate_hz));
    msg.baseline_hz.push_back(static_cast<float>(m.ema_hz));
    msg.seconds_since_last.push_back(seconds_since_last);
    msg.status.push_back(status_byte);
  }

  msg.total_received = messages_received_.load(std::memory_order_relaxed);
  msg.total_written = messages_written_.load(std::memory_order_relaxed);

  if (monitor_pub_) {
    monitor_pub_->publish(std::move(msg));
  }
}

void ServiceBagRecorder::handle_serialized_message(
  const std::string & topic,
  const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
  const rclcpp::MessageInfo & message_info)
{
  // Preflight tracking: must run before the is_recording_ early return.
  if (preflight_active_.load(std::memory_order_acquire)) {
    bool newly_inserted = false;
    {
      std::scoped_lock<std::mutex> lk(preflight_mutex_);
      newly_inserted = preflight_received_topics_.insert(topic).second;
    }
    if (newly_inserted) {
      preflight_cv_.notify_all();
    }
  }

  // Per-topic monitor counter — runs even when not recording so the
  // operator can see topic health before pressing record.
  if (!per_topic_metrics_.empty()) {
    const auto metric_it = per_topic_metrics_.find(topic);
    if (metric_it != per_topic_metrics_.end()) {
      metric_it->second->message_count.fetch_add(1, std::memory_order_relaxed);
      const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
      metric_it->second->last_recv_ns.store(
        static_cast<uint64_t>(now_ns), std::memory_order_relaxed);
    }
  }

  // Fast path for when not recording
  if (!is_recording_.load(std::memory_order_acquire)) {
    return;
  }

  messages_received_++;

  const auto it = type_for_topic_.find(topic);
  if (it == type_for_topic_.end()) {
    return;
  }
  const std::string & type = it->second;

  // Get timestamps from RMW
  const auto & rmw_info = message_info.get_rmw_message_info();

  // Use source_timestamp (when message was published) for rosbag timeline
  rclcpp::Time source_timestamp(rmw_info.source_timestamp, RCL_ROS_TIME);

  // Note: received_timestamp is also available via rmw_info.received_timestamp
  // MCAP format stores both: publishTime (source) and logTime (received)

  // Write to bag with lock - double-check writer_ inside lock to prevent race condition
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    // Second check inside lock - writer_ may have been reset by handle_stop()
    if (!writer_) {
      return;
    }
    writer_->write(serialized_msg, topic, type, source_timestamp);
  }

  messages_written_++;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Use MultiThreadedExecutor for parallel callback processing
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    4  // 4 threads: camera, joint, other, service
  );

  auto node = std::make_shared<ServiceBagRecorder>();
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Running with MultiThreadedExecutor (4 threads)");

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
