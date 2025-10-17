// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:msg/DatasetInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/dataset_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/msg/detail/dataset_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace msg
{

namespace builder
{

class Init_DatasetInfo_fps
{
public:
  explicit Init_DatasetInfo_fps(::physical_ai_interfaces::msg::DatasetInfo & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::msg::DatasetInfo fps(::physical_ai_interfaces::msg::DatasetInfo::_fps_type arg)
  {
    msg_.fps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::msg::DatasetInfo msg_;
};

class Init_DatasetInfo_total_tasks
{
public:
  explicit Init_DatasetInfo_total_tasks(::physical_ai_interfaces::msg::DatasetInfo & msg)
  : msg_(msg)
  {}
  Init_DatasetInfo_fps total_tasks(::physical_ai_interfaces::msg::DatasetInfo::_total_tasks_type arg)
  {
    msg_.total_tasks = std::move(arg);
    return Init_DatasetInfo_fps(msg_);
  }

private:
  ::physical_ai_interfaces::msg::DatasetInfo msg_;
};

class Init_DatasetInfo_total_episodes
{
public:
  explicit Init_DatasetInfo_total_episodes(::physical_ai_interfaces::msg::DatasetInfo & msg)
  : msg_(msg)
  {}
  Init_DatasetInfo_total_tasks total_episodes(::physical_ai_interfaces::msg::DatasetInfo::_total_episodes_type arg)
  {
    msg_.total_episodes = std::move(arg);
    return Init_DatasetInfo_total_tasks(msg_);
  }

private:
  ::physical_ai_interfaces::msg::DatasetInfo msg_;
};

class Init_DatasetInfo_robot_type
{
public:
  explicit Init_DatasetInfo_robot_type(::physical_ai_interfaces::msg::DatasetInfo & msg)
  : msg_(msg)
  {}
  Init_DatasetInfo_total_episodes robot_type(::physical_ai_interfaces::msg::DatasetInfo::_robot_type_type arg)
  {
    msg_.robot_type = std::move(arg);
    return Init_DatasetInfo_total_episodes(msg_);
  }

private:
  ::physical_ai_interfaces::msg::DatasetInfo msg_;
};

class Init_DatasetInfo_codebase_version
{
public:
  Init_DatasetInfo_codebase_version()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DatasetInfo_robot_type codebase_version(::physical_ai_interfaces::msg::DatasetInfo::_codebase_version_type arg)
  {
    msg_.codebase_version = std::move(arg);
    return Init_DatasetInfo_robot_type(msg_);
  }

private:
  ::physical_ai_interfaces::msg::DatasetInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::msg::DatasetInfo>()
{
  return physical_ai_interfaces::msg::builder::Init_DatasetInfo_codebase_version();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__BUILDER_HPP_
