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

#include "rosbag_recorder/topic_health_checker.hpp"

#include <algorithm>
#include <numeric>

namespace rosbag_recorder
{

void TopicHealthChecker::register_topic(
  const std::string & topic,
  double expected_hz,
  size_t min_messages,
  double max_interval_ratio)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  TopicConfig config;
  config.expected_hz = expected_hz;
  config.min_messages = min_messages;
  config.max_interval_ratio = max_interval_ratio;

  TopicData data;
  data.config = config;

  topics_[topic] = data;
}

void TopicHealthChecker::register_topic_auto(const std::string & topic, bool is_image_topic)
{
  double expected_hz = is_image_topic ? DEFAULT_IMAGE_HZ : DEFAULT_JOINT_HZ;
  register_topic(topic, expected_hz);
}

void TopicHealthChecker::record_message(
  const std::string & topic,
  const std::chrono::steady_clock::time_point & timestamp)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  auto it = topics_.find(topic);
  if (it == topics_.end()) {
    return;  // Topic not registered
  }

  auto & data = it->second;
  data.timestamps.push_back(timestamp);

  // Limit memory usage
  while (data.timestamps.size() > MAX_TIMESTAMPS) {
    data.timestamps.pop_front();
  }
}

void TopicHealthChecker::record_message_now(const std::string & topic)
{
  record_message(topic, std::chrono::steady_clock::now());
}

TopicHealthChecker::TopicStatus TopicHealthChecker::check_topic_status(
  const std::string & topic) const
{
  std::scoped_lock<std::mutex> lock(mutex_);

  TopicStatus status;
  status.is_stable = false;
  status.message_count = 0;
  status.actual_hz = 0.0;
  status.expected_hz = 0.0;

  auto it = topics_.find(topic);
  if (it == topics_.end()) {
    status.reason = "Topic not registered";
    return status;
  }

  const auto & data = it->second;
  const auto & config = data.config;
  const auto & timestamps = data.timestamps;

  status.expected_hz = config.expected_hz;
  status.message_count = timestamps.size();

  // Check minimum message count
  if (timestamps.size() < config.min_messages) {
    status.reason = "Not enough messages: " + std::to_string(timestamps.size()) +
      "/" + std::to_string(config.min_messages);
    return status;
  }

  // Calculate actual frequency from recent messages
  if (timestamps.size() < 2) {
    status.reason = "Need at least 2 messages to calculate frequency";
    return status;
  }

  // Calculate intervals between consecutive messages
  std::vector<double> intervals;
  intervals.reserve(timestamps.size() - 1);

  for (size_t i = 1; i < timestamps.size(); ++i) {
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      timestamps[i] - timestamps[i - 1]);
    intervals.push_back(duration.count() / 1000000.0);  // Convert to seconds
  }

  // Calculate mean interval
  double sum = std::accumulate(intervals.begin(), intervals.end(), 0.0);
  double mean_interval = sum / intervals.size();

  // Calculate actual Hz
  status.actual_hz = (mean_interval > 0) ? (1.0 / mean_interval) : 0.0;

  // Calculate expected interval
  double expected_interval = 1.0 / config.expected_hz;
  double max_allowed_interval = expected_interval * config.max_interval_ratio;

  // Check if actual interval is within acceptable range
  if (mean_interval > max_allowed_interval) {
    status.reason = "Interval too large: " + std::to_string(mean_interval * 1000) +
      "ms > " + std::to_string(max_allowed_interval * 1000) + "ms";
    return status;
  }

  // All checks passed
  status.is_stable = true;
  status.reason = "Stable";
  return status;
}

bool TopicHealthChecker::is_all_stable() const
{
  std::scoped_lock<std::mutex> lock(mutex_);

  if (topics_.empty()) {
    return false;  // No topics registered
  }

  for (const auto & [topic, data] : topics_) {
    // Temporarily unlock for nested call (use internal check instead)
    const auto & config = data.config;
    const auto & timestamps = data.timestamps;

    if (timestamps.size() < config.min_messages) {
      return false;
    }

    if (timestamps.size() < 2) {
      return false;
    }

    // Calculate intervals
    std::vector<double> intervals;
    for (size_t i = 1; i < timestamps.size(); ++i) {
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        timestamps[i] - timestamps[i - 1]);
      intervals.push_back(duration.count() / 1000000.0);
    }

    double sum = std::accumulate(intervals.begin(), intervals.end(), 0.0);
    double mean_interval = sum / intervals.size();
    double expected_interval = 1.0 / config.expected_hz;
    double max_allowed_interval = expected_interval * config.max_interval_ratio;

    if (mean_interval > max_allowed_interval) {
      return false;
    }
  }

  return true;
}

std::vector<std::string> TopicHealthChecker::get_pending_topics() const
{
  std::scoped_lock<std::mutex> lock(mutex_);

  std::vector<std::string> pending;

  for (const auto & [topic, data] : topics_) {
    const auto & config = data.config;
    const auto & timestamps = data.timestamps;

    bool is_stable = true;

    if (timestamps.size() < config.min_messages) {
      is_stable = false;
    } else if (timestamps.size() >= 2) {
      std::vector<double> intervals;
      for (size_t i = 1; i < timestamps.size(); ++i) {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          timestamps[i] - timestamps[i - 1]);
        intervals.push_back(duration.count() / 1000000.0);
      }

      double sum = std::accumulate(intervals.begin(), intervals.end(), 0.0);
      double mean_interval = sum / intervals.size();
      double expected_interval = 1.0 / config.expected_hz;
      double max_allowed_interval = expected_interval * config.max_interval_ratio;

      if (mean_interval > max_allowed_interval) {
        is_stable = false;
      }
    } else {
      is_stable = false;
    }

    if (!is_stable) {
      pending.push_back(topic);
    }
  }

  return pending;
}

void TopicHealthChecker::clear()
{
  std::scoped_lock<std::mutex> lock(mutex_);

  for (auto & [topic, data] : topics_) {
    data.timestamps.clear();
  }
}

void TopicHealthChecker::unregister_topic(const std::string & topic)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  topics_.erase(topic);
}

size_t TopicHealthChecker::get_registered_topic_count() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  return topics_.size();
}

}  // namespace rosbag_recorder
