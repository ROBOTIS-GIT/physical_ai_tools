// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:msg/TrainingInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/training_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/msg/detail/training_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrainingInfo_save_freq
{
public:
  explicit Init_TrainingInfo_save_freq(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::msg::TrainingInfo save_freq(::physical_ai_interfaces::msg::TrainingInfo::_save_freq_type arg)
  {
    msg_.save_freq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_log_freq
{
public:
  explicit Init_TrainingInfo_log_freq(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_save_freq log_freq(::physical_ai_interfaces::msg::TrainingInfo::_log_freq_type arg)
  {
    msg_.log_freq = std::move(arg);
    return Init_TrainingInfo_save_freq(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_eval_freq
{
public:
  explicit Init_TrainingInfo_eval_freq(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_log_freq eval_freq(::physical_ai_interfaces::msg::TrainingInfo::_eval_freq_type arg)
  {
    msg_.eval_freq = std::move(arg);
    return Init_TrainingInfo_log_freq(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_steps
{
public:
  explicit Init_TrainingInfo_steps(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_eval_freq steps(::physical_ai_interfaces::msg::TrainingInfo::_steps_type arg)
  {
    msg_.steps = std::move(arg);
    return Init_TrainingInfo_eval_freq(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_batch_size
{
public:
  explicit Init_TrainingInfo_batch_size(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_steps batch_size(::physical_ai_interfaces::msg::TrainingInfo::_batch_size_type arg)
  {
    msg_.batch_size = std::move(arg);
    return Init_TrainingInfo_steps(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_num_workers
{
public:
  explicit Init_TrainingInfo_num_workers(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_batch_size num_workers(::physical_ai_interfaces::msg::TrainingInfo::_num_workers_type arg)
  {
    msg_.num_workers = std::move(arg);
    return Init_TrainingInfo_batch_size(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_seed
{
public:
  explicit Init_TrainingInfo_seed(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_num_workers seed(::physical_ai_interfaces::msg::TrainingInfo::_seed_type arg)
  {
    msg_.seed = std::move(arg);
    return Init_TrainingInfo_num_workers(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_policy_device
{
public:
  explicit Init_TrainingInfo_policy_device(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_seed policy_device(::physical_ai_interfaces::msg::TrainingInfo::_policy_device_type arg)
  {
    msg_.policy_device = std::move(arg);
    return Init_TrainingInfo_seed(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_output_folder_name
{
public:
  explicit Init_TrainingInfo_output_folder_name(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_policy_device output_folder_name(::physical_ai_interfaces::msg::TrainingInfo::_output_folder_name_type arg)
  {
    msg_.output_folder_name = std::move(arg);
    return Init_TrainingInfo_policy_device(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_policy_type
{
public:
  explicit Init_TrainingInfo_policy_type(::physical_ai_interfaces::msg::TrainingInfo & msg)
  : msg_(msg)
  {}
  Init_TrainingInfo_output_folder_name policy_type(::physical_ai_interfaces::msg::TrainingInfo::_policy_type_type arg)
  {
    msg_.policy_type = std::move(arg);
    return Init_TrainingInfo_output_folder_name(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

class Init_TrainingInfo_dataset
{
public:
  Init_TrainingInfo_dataset()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrainingInfo_policy_type dataset(::physical_ai_interfaces::msg::TrainingInfo::_dataset_type arg)
  {
    msg_.dataset = std::move(arg);
    return Init_TrainingInfo_policy_type(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::msg::TrainingInfo>()
{
  return physical_ai_interfaces::msg::builder::Init_TrainingInfo_dataset();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__BUILDER_HPP_
