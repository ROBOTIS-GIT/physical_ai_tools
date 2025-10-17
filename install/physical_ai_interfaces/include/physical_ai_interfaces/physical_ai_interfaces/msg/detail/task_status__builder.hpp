// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:msg/TaskStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/msg/detail/task_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace msg
{

namespace builder
{

class Init_TaskStatus_error
{
public:
  explicit Init_TaskStatus_error(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::msg::TaskStatus error(::physical_ai_interfaces::msg::TaskStatus::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_total_ram_size
{
public:
  explicit Init_TaskStatus_total_ram_size(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_error total_ram_size(::physical_ai_interfaces::msg::TaskStatus::_total_ram_size_type arg)
  {
    msg_.total_ram_size = std::move(arg);
    return Init_TaskStatus_error(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_used_ram_size
{
public:
  explicit Init_TaskStatus_used_ram_size(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_total_ram_size used_ram_size(::physical_ai_interfaces::msg::TaskStatus::_used_ram_size_type arg)
  {
    msg_.used_ram_size = std::move(arg);
    return Init_TaskStatus_total_ram_size(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_used_cpu
{
public:
  explicit Init_TaskStatus_used_cpu(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_used_ram_size used_cpu(::physical_ai_interfaces::msg::TaskStatus::_used_cpu_type arg)
  {
    msg_.used_cpu = std::move(arg);
    return Init_TaskStatus_used_ram_size(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_total_storage_size
{
public:
  explicit Init_TaskStatus_total_storage_size(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_used_cpu total_storage_size(::physical_ai_interfaces::msg::TaskStatus::_total_storage_size_type arg)
  {
    msg_.total_storage_size = std::move(arg);
    return Init_TaskStatus_used_cpu(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_used_storage_size
{
public:
  explicit Init_TaskStatus_used_storage_size(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_total_storage_size used_storage_size(::physical_ai_interfaces::msg::TaskStatus::_used_storage_size_type arg)
  {
    msg_.used_storage_size = std::move(arg);
    return Init_TaskStatus_total_storage_size(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_encoding_progress
{
public:
  explicit Init_TaskStatus_encoding_progress(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_used_storage_size encoding_progress(::physical_ai_interfaces::msg::TaskStatus::_encoding_progress_type arg)
  {
    msg_.encoding_progress = std::move(arg);
    return Init_TaskStatus_used_storage_size(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_current_task_instruction
{
public:
  explicit Init_TaskStatus_current_task_instruction(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_encoding_progress current_task_instruction(::physical_ai_interfaces::msg::TaskStatus::_current_task_instruction_type arg)
  {
    msg_.current_task_instruction = std::move(arg);
    return Init_TaskStatus_encoding_progress(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_current_scenario_number
{
public:
  explicit Init_TaskStatus_current_scenario_number(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_current_task_instruction current_scenario_number(::physical_ai_interfaces::msg::TaskStatus::_current_scenario_number_type arg)
  {
    msg_.current_scenario_number = std::move(arg);
    return Init_TaskStatus_current_task_instruction(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_current_episode_number
{
public:
  explicit Init_TaskStatus_current_episode_number(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_current_scenario_number current_episode_number(::physical_ai_interfaces::msg::TaskStatus::_current_episode_number_type arg)
  {
    msg_.current_episode_number = std::move(arg);
    return Init_TaskStatus_current_scenario_number(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_proceed_time
{
public:
  explicit Init_TaskStatus_proceed_time(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_current_episode_number proceed_time(::physical_ai_interfaces::msg::TaskStatus::_proceed_time_type arg)
  {
    msg_.proceed_time = std::move(arg);
    return Init_TaskStatus_current_episode_number(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_total_time
{
public:
  explicit Init_TaskStatus_total_time(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_proceed_time total_time(::physical_ai_interfaces::msg::TaskStatus::_total_time_type arg)
  {
    msg_.total_time = std::move(arg);
    return Init_TaskStatus_proceed_time(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_phase
{
public:
  explicit Init_TaskStatus_phase(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_total_time phase(::physical_ai_interfaces::msg::TaskStatus::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_TaskStatus_total_time(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_robot_type
{
public:
  explicit Init_TaskStatus_robot_type(::physical_ai_interfaces::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_phase robot_type(::physical_ai_interfaces::msg::TaskStatus::_robot_type_type arg)
  {
    msg_.robot_type = std::move(arg);
    return Init_TaskStatus_phase(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

class Init_TaskStatus_task_info
{
public:
  Init_TaskStatus_task_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskStatus_robot_type task_info(::physical_ai_interfaces::msg::TaskStatus::_task_info_type arg)
  {
    msg_.task_info = std::move(arg);
    return Init_TaskStatus_robot_type(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::msg::TaskStatus>()
{
  return physical_ai_interfaces::msg::builder::Init_TaskStatus_task_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__BUILDER_HPP_
