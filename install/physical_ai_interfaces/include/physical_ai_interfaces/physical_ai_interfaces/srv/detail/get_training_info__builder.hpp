// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/GetTrainingInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_training_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_TRAINING_INFO__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_TRAINING_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/get_training_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetTrainingInfo_Request_model_path
{
public:
  Init_GetTrainingInfo_Request_model_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::physical_ai_interfaces::srv::GetTrainingInfo_Request model_path(::physical_ai_interfaces::srv::GetTrainingInfo_Request::_model_path_type arg)
  {
    msg_.model_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetTrainingInfo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetTrainingInfo_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_GetTrainingInfo_Request_model_path();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetTrainingInfo_Response_message
{
public:
  explicit Init_GetTrainingInfo_Response_message(::physical_ai_interfaces::srv::GetTrainingInfo_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::GetTrainingInfo_Response message(::physical_ai_interfaces::srv::GetTrainingInfo_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetTrainingInfo_Response msg_;
};

class Init_GetTrainingInfo_Response_success
{
public:
  explicit Init_GetTrainingInfo_Response_success(::physical_ai_interfaces::srv::GetTrainingInfo_Response & msg)
  : msg_(msg)
  {}
  Init_GetTrainingInfo_Response_message success(::physical_ai_interfaces::srv::GetTrainingInfo_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetTrainingInfo_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetTrainingInfo_Response msg_;
};

class Init_GetTrainingInfo_Response_training_info
{
public:
  Init_GetTrainingInfo_Response_training_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTrainingInfo_Response_success training_info(::physical_ai_interfaces::srv::GetTrainingInfo_Response::_training_info_type arg)
  {
    msg_.training_info = std::move(arg);
    return Init_GetTrainingInfo_Response_success(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetTrainingInfo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetTrainingInfo_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_GetTrainingInfo_Response_training_info();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetTrainingInfo_Event_response
{
public:
  explicit Init_GetTrainingInfo_Event_response(::physical_ai_interfaces::srv::GetTrainingInfo_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::GetTrainingInfo_Event response(::physical_ai_interfaces::srv::GetTrainingInfo_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetTrainingInfo_Event msg_;
};

class Init_GetTrainingInfo_Event_request
{
public:
  explicit Init_GetTrainingInfo_Event_request(::physical_ai_interfaces::srv::GetTrainingInfo_Event & msg)
  : msg_(msg)
  {}
  Init_GetTrainingInfo_Event_response request(::physical_ai_interfaces::srv::GetTrainingInfo_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetTrainingInfo_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetTrainingInfo_Event msg_;
};

class Init_GetTrainingInfo_Event_info
{
public:
  Init_GetTrainingInfo_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTrainingInfo_Event_request info(::physical_ai_interfaces::srv::GetTrainingInfo_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetTrainingInfo_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetTrainingInfo_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetTrainingInfo_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_GetTrainingInfo_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_TRAINING_INFO__BUILDER_HPP_
