// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/SetRobotType.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/set_robot_type.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__SET_ROBOT_TYPE__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__SET_ROBOT_TYPE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/set_robot_type__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetRobotType_Request_robot_type
{
public:
  Init_SetRobotType_Request_robot_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::physical_ai_interfaces::srv::SetRobotType_Request robot_type(::physical_ai_interfaces::srv::SetRobotType_Request::_robot_type_type arg)
  {
    msg_.robot_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SetRobotType_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SetRobotType_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_SetRobotType_Request_robot_type();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetRobotType_Response_message
{
public:
  explicit Init_SetRobotType_Response_message(::physical_ai_interfaces::srv::SetRobotType_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::SetRobotType_Response message(::physical_ai_interfaces::srv::SetRobotType_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SetRobotType_Response msg_;
};

class Init_SetRobotType_Response_success
{
public:
  Init_SetRobotType_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetRobotType_Response_message success(::physical_ai_interfaces::srv::SetRobotType_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetRobotType_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SetRobotType_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SetRobotType_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_SetRobotType_Response_success();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetRobotType_Event_response
{
public:
  explicit Init_SetRobotType_Event_response(::physical_ai_interfaces::srv::SetRobotType_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::SetRobotType_Event response(::physical_ai_interfaces::srv::SetRobotType_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SetRobotType_Event msg_;
};

class Init_SetRobotType_Event_request
{
public:
  explicit Init_SetRobotType_Event_request(::physical_ai_interfaces::srv::SetRobotType_Event & msg)
  : msg_(msg)
  {}
  Init_SetRobotType_Event_response request(::physical_ai_interfaces::srv::SetRobotType_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetRobotType_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SetRobotType_Event msg_;
};

class Init_SetRobotType_Event_info
{
public:
  Init_SetRobotType_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetRobotType_Event_request info(::physical_ai_interfaces::srv::SetRobotType_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetRobotType_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::SetRobotType_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::SetRobotType_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_SetRobotType_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__SET_ROBOT_TYPE__BUILDER_HPP_
