// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/SendCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/send_command.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/send_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SendCommand_Request_task_info
{
public:
  explicit Init_SendCommand_Request_task_info(::physical_ai_interfaces::srv::SendCommand_Request & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::SendCommand_Request task_info(::physical_ai_interfaces::srv::SendCommand_Request::_task_info_type arg)
  {
    msg_.task_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendCommand_Request msg_;
};

class Init_SendCommand_Request_command
{
public:
  Init_SendCommand_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendCommand_Request_task_info command(::physical_ai_interfaces::srv::SendCommand_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_SendCommand_Request_task_info(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SendCommand_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_SendCommand_Request_command();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SendCommand_Response_message
{
public:
  explicit Init_SendCommand_Response_message(::physical_ai_interfaces::srv::SendCommand_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::SendCommand_Response message(::physical_ai_interfaces::srv::SendCommand_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendCommand_Response msg_;
};

class Init_SendCommand_Response_success
{
public:
  Init_SendCommand_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendCommand_Response_message success(::physical_ai_interfaces::srv::SendCommand_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SendCommand_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SendCommand_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_SendCommand_Response_success();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SendCommand_Event_response
{
public:
  explicit Init_SendCommand_Event_response(::physical_ai_interfaces::srv::SendCommand_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::SendCommand_Event response(::physical_ai_interfaces::srv::SendCommand_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendCommand_Event msg_;
};

class Init_SendCommand_Event_request
{
public:
  explicit Init_SendCommand_Event_request(::physical_ai_interfaces::srv::SendCommand_Event & msg)
  : msg_(msg)
  {}
  Init_SendCommand_Event_response request(::physical_ai_interfaces::srv::SendCommand_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SendCommand_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendCommand_Event msg_;
};

class Init_SendCommand_Event_info
{
public:
  Init_SendCommand_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendCommand_Event_request info(::physical_ai_interfaces::srv::SendCommand_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SendCommand_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SendCommand_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SendCommand_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_SendCommand_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__BUILDER_HPP_
