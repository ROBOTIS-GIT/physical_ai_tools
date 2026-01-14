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
// Author: Dongyun Kim

#ifndef ROSBAG_RECORDER__TOPIC_HEALTH_CHECKER_HPP_
#define ROSBAG_RECORDER__TOPIC_HEALTH_CHECKER_HPP_

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rosbag_recorder
{

/**
 * @brief Tracks topic message timestamps and determines if topics are stable.
 *
 * A topic is considered "stable" when:
 * 1. At least min_messages have been received
 * 2. The average interval between messages is within acceptable range
 *    (expected_interval * max_interval_ratio)
 */
class TopicHealthChecker
{
public:
  struct TopicConfig
  {
    double expected_hz;        // Expected frequency in Hz
    size_t min_messages;       // Minimum messages required for stability check
    double max_interval_ratio; // Max allowed ratio of actual/expected interval
  };

  struct TopicStatus
  {
    bool is_stable;
    size_t message_count;
    double actual_hz;
    double expected_hz;
    std::string reason;  // Reason if not stable
  };

  /**
   * @brief Default configuration values
   */
  static constexpr size_t DEFAULT_MIN_MESSAGES = 10;
  static constexpr double DEFAULT_MAX_INTERVAL_RATIO = 2.0;
  static constexpr double DEFAULT_IMAGE_HZ = 30.0;
  static constexpr double DEFAULT_JOINT_HZ = 100.0;

  TopicHealthChecker() = default;

  /**
   * @brief Register a topic to be monitored
   * @param topic Topic name
   * @param expected_hz Expected frequency
   * @param min_messages Minimum messages for stability (default: 10)
   * @param max_interval_ratio Max interval ratio (default: 2.0)
   */
  void register_topic(
    const std::string & topic,
    double expected_hz,
    size_t min_messages = DEFAULT_MIN_MESSAGES,
    double max_interval_ratio = DEFAULT_MAX_INTERVAL_RATIO);

  /**
   * @brief Register a topic with auto-detected frequency based on topic type
   * @param topic Topic name
   * @param is_image_topic Whether this is an image topic (30Hz) or joint topic (100Hz)
   */
  void register_topic_auto(const std::string & topic, bool is_image_topic);

  /**
   * @brief Record a message timestamp for a topic
   * @param topic Topic name
   * @param timestamp Message timestamp
   */
  void record_message(
    const std::string & topic,
    const std::chrono::steady_clock::time_point & timestamp);

  /**
   * @brief Record a message with current time
   * @param topic Topic name
   */
  void record_message_now(const std::string & topic);

  /**
   * @brief Check if a specific topic is stable
   * @param topic Topic name
   * @return TopicStatus with stability info
   */
  TopicStatus check_topic_status(const std::string & topic) const;

  /**
   * @brief Check if all registered topics are stable
   * @return true if all topics are stable
   */
  bool is_all_stable() const;

  /**
   * @brief Get list of topics that are not yet stable
   * @return Vector of topic names
   */
  std::vector<std::string> get_pending_topics() const;

  /**
   * @brief Clear all recorded timestamps (call when resetting)
   */
  void clear();

  /**
   * @brief Remove a topic from monitoring
   * @param topic Topic name
   */
  void unregister_topic(const std::string & topic);

  /**
   * @brief Get number of registered topics
   */
  size_t get_registered_topic_count() const;

private:
  struct TopicData
  {
    TopicConfig config;
    std::deque<std::chrono::steady_clock::time_point> timestamps;
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicData> topics_;

  // Maximum timestamps to keep per topic (to limit memory)
  static constexpr size_t MAX_TIMESTAMPS = 100;
};

}  // namespace rosbag_recorder

#endif  // ROSBAG_RECORDER__TOPIC_HEALTH_CHECKER_HPP_
