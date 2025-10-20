// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/GetDatasetList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_dataset_list.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_LIST__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/get_dataset_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDatasetList_Request_user_id
{
public:
  Init_GetDatasetList_Request_user_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::physical_ai_interfaces::srv::GetDatasetList_Request user_id(::physical_ai_interfaces::srv::GetDatasetList_Request::_user_id_type arg)
  {
    msg_.user_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetList_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetDatasetList_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_GetDatasetList_Request_user_id();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDatasetList_Response_message
{
public:
  explicit Init_GetDatasetList_Response_message(::physical_ai_interfaces::srv::GetDatasetList_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::GetDatasetList_Response message(::physical_ai_interfaces::srv::GetDatasetList_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetList_Response msg_;
};

class Init_GetDatasetList_Response_success
{
public:
  explicit Init_GetDatasetList_Response_success(::physical_ai_interfaces::srv::GetDatasetList_Response & msg)
  : msg_(msg)
  {}
  Init_GetDatasetList_Response_message success(::physical_ai_interfaces::srv::GetDatasetList_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetDatasetList_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetList_Response msg_;
};

class Init_GetDatasetList_Response_dataset_list
{
public:
  Init_GetDatasetList_Response_dataset_list()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDatasetList_Response_success dataset_list(::physical_ai_interfaces::srv::GetDatasetList_Response::_dataset_list_type arg)
  {
    msg_.dataset_list = std::move(arg);
    return Init_GetDatasetList_Response_success(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetList_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetDatasetList_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_GetDatasetList_Response_dataset_list();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDatasetList_Event_response
{
public:
  explicit Init_GetDatasetList_Event_response(::physical_ai_interfaces::srv::GetDatasetList_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::GetDatasetList_Event response(::physical_ai_interfaces::srv::GetDatasetList_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetList_Event msg_;
};

class Init_GetDatasetList_Event_request
{
public:
  explicit Init_GetDatasetList_Event_request(::physical_ai_interfaces::srv::GetDatasetList_Event & msg)
  : msg_(msg)
  {}
  Init_GetDatasetList_Event_response request(::physical_ai_interfaces::srv::GetDatasetList_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetDatasetList_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetList_Event msg_;
};

class Init_GetDatasetList_Event_info
{
public:
  Init_GetDatasetList_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDatasetList_Event_request info(::physical_ai_interfaces::srv::GetDatasetList_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetDatasetList_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetDatasetList_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetDatasetList_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_GetDatasetList_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_LIST__BUILDER_HPP_
