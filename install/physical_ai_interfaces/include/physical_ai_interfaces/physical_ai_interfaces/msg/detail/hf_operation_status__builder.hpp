// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:msg/HFOperationStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/hf_operation_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/msg/detail/hf_operation_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace msg
{

namespace builder
{

class Init_HFOperationStatus_progress_percentage
{
public:
  explicit Init_HFOperationStatus_progress_percentage(::physical_ai_interfaces::msg::HFOperationStatus & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::msg::HFOperationStatus progress_percentage(::physical_ai_interfaces::msg::HFOperationStatus::_progress_percentage_type arg)
  {
    msg_.progress_percentage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::msg::HFOperationStatus msg_;
};

class Init_HFOperationStatus_progress_total
{
public:
  explicit Init_HFOperationStatus_progress_total(::physical_ai_interfaces::msg::HFOperationStatus & msg)
  : msg_(msg)
  {}
  Init_HFOperationStatus_progress_percentage progress_total(::physical_ai_interfaces::msg::HFOperationStatus::_progress_total_type arg)
  {
    msg_.progress_total = std::move(arg);
    return Init_HFOperationStatus_progress_percentage(msg_);
  }

private:
  ::physical_ai_interfaces::msg::HFOperationStatus msg_;
};

class Init_HFOperationStatus_progress_current
{
public:
  explicit Init_HFOperationStatus_progress_current(::physical_ai_interfaces::msg::HFOperationStatus & msg)
  : msg_(msg)
  {}
  Init_HFOperationStatus_progress_total progress_current(::physical_ai_interfaces::msg::HFOperationStatus::_progress_current_type arg)
  {
    msg_.progress_current = std::move(arg);
    return Init_HFOperationStatus_progress_total(msg_);
  }

private:
  ::physical_ai_interfaces::msg::HFOperationStatus msg_;
};

class Init_HFOperationStatus_message
{
public:
  explicit Init_HFOperationStatus_message(::physical_ai_interfaces::msg::HFOperationStatus & msg)
  : msg_(msg)
  {}
  Init_HFOperationStatus_progress_current message(::physical_ai_interfaces::msg::HFOperationStatus::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_HFOperationStatus_progress_current(msg_);
  }

private:
  ::physical_ai_interfaces::msg::HFOperationStatus msg_;
};

class Init_HFOperationStatus_repo_id
{
public:
  explicit Init_HFOperationStatus_repo_id(::physical_ai_interfaces::msg::HFOperationStatus & msg)
  : msg_(msg)
  {}
  Init_HFOperationStatus_message repo_id(::physical_ai_interfaces::msg::HFOperationStatus::_repo_id_type arg)
  {
    msg_.repo_id = std::move(arg);
    return Init_HFOperationStatus_message(msg_);
  }

private:
  ::physical_ai_interfaces::msg::HFOperationStatus msg_;
};

class Init_HFOperationStatus_local_path
{
public:
  explicit Init_HFOperationStatus_local_path(::physical_ai_interfaces::msg::HFOperationStatus & msg)
  : msg_(msg)
  {}
  Init_HFOperationStatus_repo_id local_path(::physical_ai_interfaces::msg::HFOperationStatus::_local_path_type arg)
  {
    msg_.local_path = std::move(arg);
    return Init_HFOperationStatus_repo_id(msg_);
  }

private:
  ::physical_ai_interfaces::msg::HFOperationStatus msg_;
};

class Init_HFOperationStatus_status
{
public:
  explicit Init_HFOperationStatus_status(::physical_ai_interfaces::msg::HFOperationStatus & msg)
  : msg_(msg)
  {}
  Init_HFOperationStatus_local_path status(::physical_ai_interfaces::msg::HFOperationStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_HFOperationStatus_local_path(msg_);
  }

private:
  ::physical_ai_interfaces::msg::HFOperationStatus msg_;
};

class Init_HFOperationStatus_operation
{
public:
  Init_HFOperationStatus_operation()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HFOperationStatus_status operation(::physical_ai_interfaces::msg::HFOperationStatus::_operation_type arg)
  {
    msg_.operation = std::move(arg);
    return Init_HFOperationStatus_status(msg_);
  }

private:
  ::physical_ai_interfaces::msg::HFOperationStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::msg::HFOperationStatus>()
{
  return physical_ai_interfaces::msg::builder::Init_HFOperationStatus_operation();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__BUILDER_HPP_
