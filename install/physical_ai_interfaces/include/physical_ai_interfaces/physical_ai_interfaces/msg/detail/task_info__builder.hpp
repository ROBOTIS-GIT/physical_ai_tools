// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:msg/TaskInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/msg/detail/task_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace msg
{

namespace builder
{

class Init_TaskInfo_record_inference_mode
{
public:
  explicit Init_TaskInfo_record_inference_mode(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::msg::TaskInfo record_inference_mode(::physical_ai_interfaces::msg::TaskInfo::_record_inference_mode_type arg)
  {
    msg_.record_inference_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_use_optimized_save_mode
{
public:
  explicit Init_TaskInfo_use_optimized_save_mode(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_record_inference_mode use_optimized_save_mode(::physical_ai_interfaces::msg::TaskInfo::_use_optimized_save_mode_type arg)
  {
    msg_.use_optimized_save_mode = std::move(arg);
    return Init_TaskInfo_record_inference_mode(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_private_mode
{
public:
  explicit Init_TaskInfo_private_mode(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_use_optimized_save_mode private_mode(::physical_ai_interfaces::msg::TaskInfo::_private_mode_type arg)
  {
    msg_.private_mode = std::move(arg);
    return Init_TaskInfo_use_optimized_save_mode(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_push_to_hub
{
public:
  explicit Init_TaskInfo_push_to_hub(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_private_mode push_to_hub(::physical_ai_interfaces::msg::TaskInfo::_push_to_hub_type arg)
  {
    msg_.push_to_hub = std::move(arg);
    return Init_TaskInfo_private_mode(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_num_episodes
{
public:
  explicit Init_TaskInfo_num_episodes(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_push_to_hub num_episodes(::physical_ai_interfaces::msg::TaskInfo::_num_episodes_type arg)
  {
    msg_.num_episodes = std::move(arg);
    return Init_TaskInfo_push_to_hub(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_reset_time_s
{
public:
  explicit Init_TaskInfo_reset_time_s(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_num_episodes reset_time_s(::physical_ai_interfaces::msg::TaskInfo::_reset_time_s_type arg)
  {
    msg_.reset_time_s = std::move(arg);
    return Init_TaskInfo_num_episodes(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_episode_time_s
{
public:
  explicit Init_TaskInfo_episode_time_s(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_reset_time_s episode_time_s(::physical_ai_interfaces::msg::TaskInfo::_episode_time_s_type arg)
  {
    msg_.episode_time_s = std::move(arg);
    return Init_TaskInfo_reset_time_s(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_warmup_time_s
{
public:
  explicit Init_TaskInfo_warmup_time_s(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_episode_time_s warmup_time_s(::physical_ai_interfaces::msg::TaskInfo::_warmup_time_s_type arg)
  {
    msg_.warmup_time_s = std::move(arg);
    return Init_TaskInfo_episode_time_s(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_tags
{
public:
  explicit Init_TaskInfo_tags(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_warmup_time_s tags(::physical_ai_interfaces::msg::TaskInfo::_tags_type arg)
  {
    msg_.tags = std::move(arg);
    return Init_TaskInfo_warmup_time_s(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_fps
{
public:
  explicit Init_TaskInfo_fps(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_tags fps(::physical_ai_interfaces::msg::TaskInfo::_fps_type arg)
  {
    msg_.fps = std::move(arg);
    return Init_TaskInfo_tags(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_policy_path
{
public:
  explicit Init_TaskInfo_policy_path(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_fps policy_path(::physical_ai_interfaces::msg::TaskInfo::_policy_path_type arg)
  {
    msg_.policy_path = std::move(arg);
    return Init_TaskInfo_fps(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_task_instruction
{
public:
  explicit Init_TaskInfo_task_instruction(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_policy_path task_instruction(::physical_ai_interfaces::msg::TaskInfo::_task_instruction_type arg)
  {
    msg_.task_instruction = std::move(arg);
    return Init_TaskInfo_policy_path(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_user_id
{
public:
  explicit Init_TaskInfo_user_id(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_task_instruction user_id(::physical_ai_interfaces::msg::TaskInfo::_user_id_type arg)
  {
    msg_.user_id = std::move(arg);
    return Init_TaskInfo_task_instruction(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_task_type
{
public:
  explicit Init_TaskInfo_task_type(::physical_ai_interfaces::msg::TaskInfo & msg)
  : msg_(msg)
  {}
  Init_TaskInfo_user_id task_type(::physical_ai_interfaces::msg::TaskInfo::_task_type_type arg)
  {
    msg_.task_type = std::move(arg);
    return Init_TaskInfo_user_id(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

class Init_TaskInfo_task_name
{
public:
  Init_TaskInfo_task_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskInfo_task_type task_name(::physical_ai_interfaces::msg::TaskInfo::_task_name_type arg)
  {
    msg_.task_name = std::move(arg);
    return Init_TaskInfo_task_type(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TaskInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::msg::TaskInfo>()
{
  return physical_ai_interfaces::msg::builder::Init_TaskInfo_task_name();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__BUILDER_HPP_
