// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/SendTrainingCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/send_training_command.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_TRAINING_COMMAND__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_TRAINING_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/send_training_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SendTrainingCommand_Request_resume_model_path
{
public:
  explicit Init_SendTrainingCommand_Request_resume_model_path(::physical_ai_interfaces::srv::SendTrainingCommand_Request & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::SendTrainingCommand_Request resume_model_path(::physical_ai_interfaces::srv::SendTrainingCommand_Request::_resume_model_path_type arg)
  {
    msg_.resume_model_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Request msg_;
};

class Init_SendTrainingCommand_Request_resume
{
public:
  explicit Init_SendTrainingCommand_Request_resume(::physical_ai_interfaces::srv::SendTrainingCommand_Request & msg)
  : msg_(msg)
  {}
  Init_SendTrainingCommand_Request_resume_model_path resume(::physical_ai_interfaces::srv::SendTrainingCommand_Request::_resume_type arg)
  {
    msg_.resume = std::move(arg);
    return Init_SendTrainingCommand_Request_resume_model_path(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Request msg_;
};

class Init_SendTrainingCommand_Request_training_info
{
public:
  explicit Init_SendTrainingCommand_Request_training_info(::physical_ai_interfaces::srv::SendTrainingCommand_Request & msg)
  : msg_(msg)
  {}
  Init_SendTrainingCommand_Request_resume training_info(::physical_ai_interfaces::srv::SendTrainingCommand_Request::_training_info_type arg)
  {
    msg_.training_info = std::move(arg);
    return Init_SendTrainingCommand_Request_resume(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Request msg_;
};

class Init_SendTrainingCommand_Request_command
{
public:
  Init_SendTrainingCommand_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendTrainingCommand_Request_training_info command(::physical_ai_interfaces::srv::SendTrainingCommand_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_SendTrainingCommand_Request_training_info(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SendTrainingCommand_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_SendTrainingCommand_Request_command();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SendTrainingCommand_Response_message
{
public:
  explicit Init_SendTrainingCommand_Response_message(::physical_ai_interfaces::srv::SendTrainingCommand_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::SendTrainingCommand_Response message(::physical_ai_interfaces::srv::SendTrainingCommand_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Response msg_;
};

class Init_SendTrainingCommand_Response_success
{
public:
  Init_SendTrainingCommand_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendTrainingCommand_Response_message success(::physical_ai_interfaces::srv::SendTrainingCommand_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SendTrainingCommand_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SendTrainingCommand_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_SendTrainingCommand_Response_success();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SendTrainingCommand_Event_response
{
public:
  explicit Init_SendTrainingCommand_Event_response(::physical_ai_interfaces::srv::SendTrainingCommand_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::SendTrainingCommand_Event response(::physical_ai_interfaces::srv::SendTrainingCommand_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Event msg_;
};

class Init_SendTrainingCommand_Event_request
{
public:
  explicit Init_SendTrainingCommand_Event_request(::physical_ai_interfaces::srv::SendTrainingCommand_Event & msg)
  : msg_(msg)
  {}
  Init_SendTrainingCommand_Event_response request(::physical_ai_interfaces::srv::SendTrainingCommand_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SendTrainingCommand_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Event msg_;
};

class Init_SendTrainingCommand_Event_info
{
public:
  Init_SendTrainingCommand_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendTrainingCommand_Event_request info(::physical_ai_interfaces::srv::SendTrainingCommand_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SendTrainingCommand_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendTrainingCommand_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SendTrainingCommand_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_SendTrainingCommand_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_TRAINING_COMMAND__BUILDER_HPP_
