// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/training_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/msg/detail/training_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrainingStatus_error
{
public:
  explicit Init_TrainingStatus_error(::physical_ai_interfaces::msg::TrainingStatus & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::msg::TrainingStatus error(::physical_ai_interfaces::msg::TrainingStatus::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingStatus msg_;
};

class Init_TrainingStatus_is_training
{
public:
  explicit Init_TrainingStatus_is_training(::physical_ai_interfaces::msg::TrainingStatus & msg)
  : msg_(msg)
  {}
  Init_TrainingStatus_error is_training(::physical_ai_interfaces::msg::TrainingStatus::_is_training_type arg)
  {
    msg_.is_training = std::move(arg);
    return Init_TrainingStatus_error(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingStatus msg_;
};

class Init_TrainingStatus_current_loss
{
public:
  explicit Init_TrainingStatus_current_loss(::physical_ai_interfaces::msg::TrainingStatus & msg)
  : msg_(msg)
  {}
  Init_TrainingStatus_is_training current_loss(::physical_ai_interfaces::msg::TrainingStatus::_current_loss_type arg)
  {
    msg_.current_loss = std::move(arg);
    return Init_TrainingStatus_is_training(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingStatus msg_;
};

class Init_TrainingStatus_current_step
{
public:
  explicit Init_TrainingStatus_current_step(::physical_ai_interfaces::msg::TrainingStatus & msg)
  : msg_(msg)
  {}
  Init_TrainingStatus_current_loss current_step(::physical_ai_interfaces::msg::TrainingStatus::_current_step_type arg)
  {
    msg_.current_step = std::move(arg);
    return Init_TrainingStatus_current_loss(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingStatus msg_;
};

class Init_TrainingStatus_training_info
{
public:
  Init_TrainingStatus_training_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrainingStatus_current_step training_info(::physical_ai_interfaces::msg::TrainingStatus::_training_info_type arg)
  {
    msg_.training_info = std::move(arg);
    return Init_TrainingStatus_current_step(msg_);
  }

private:
  ::physical_ai_interfaces::msg::TrainingStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::msg::TrainingStatus>()
{
  return physical_ai_interfaces::msg::builder::Init_TrainingStatus_training_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__BUILDER_HPP_
