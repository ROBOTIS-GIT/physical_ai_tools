// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/GetDatasetInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_dataset_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/get_dataset_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDatasetInfo_Request_dataset_path
{
public:
  Init_GetDatasetInfo_Request_dataset_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::physical_ai_interfaces::srv::GetDatasetInfo_Request dataset_path(::physical_ai_interfaces::srv::GetDatasetInfo_Request::_dataset_path_type arg)
  {
    msg_.dataset_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetInfo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetDatasetInfo_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_GetDatasetInfo_Request_dataset_path();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDatasetInfo_Response_message
{
public:
  explicit Init_GetDatasetInfo_Response_message(::physical_ai_interfaces::srv::GetDatasetInfo_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::GetDatasetInfo_Response message(::physical_ai_interfaces::srv::GetDatasetInfo_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetInfo_Response msg_;
};

class Init_GetDatasetInfo_Response_success
{
public:
  explicit Init_GetDatasetInfo_Response_success(::physical_ai_interfaces::srv::GetDatasetInfo_Response & msg)
  : msg_(msg)
  {}
  Init_GetDatasetInfo_Response_message success(::physical_ai_interfaces::srv::GetDatasetInfo_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetDatasetInfo_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetInfo_Response msg_;
};

class Init_GetDatasetInfo_Response_dataset_info
{
public:
  Init_GetDatasetInfo_Response_dataset_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDatasetInfo_Response_success dataset_info(::physical_ai_interfaces::srv::GetDatasetInfo_Response::_dataset_info_type arg)
  {
    msg_.dataset_info = std::move(arg);
    return Init_GetDatasetInfo_Response_success(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetInfo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetDatasetInfo_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_GetDatasetInfo_Response_dataset_info();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDatasetInfo_Event_response
{
public:
  explicit Init_GetDatasetInfo_Event_response(::physical_ai_interfaces::srv::GetDatasetInfo_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::GetDatasetInfo_Event response(::physical_ai_interfaces::srv::GetDatasetInfo_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetInfo_Event msg_;
};

class Init_GetDatasetInfo_Event_request
{
public:
  explicit Init_GetDatasetInfo_Event_request(::physical_ai_interfaces::srv::GetDatasetInfo_Event & msg)
  : msg_(msg)
  {}
  Init_GetDatasetInfo_Event_response request(::physical_ai_interfaces::srv::GetDatasetInfo_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetDatasetInfo_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetInfo_Event msg_;
};

class Init_GetDatasetInfo_Event_info
{
public:
  Init_GetDatasetInfo_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDatasetInfo_Event_request info(::physical_ai_interfaces::srv::GetDatasetInfo_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetDatasetInfo_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetInfo_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetDatasetInfo_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_GetDatasetInfo_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__BUILDER_HPP_
