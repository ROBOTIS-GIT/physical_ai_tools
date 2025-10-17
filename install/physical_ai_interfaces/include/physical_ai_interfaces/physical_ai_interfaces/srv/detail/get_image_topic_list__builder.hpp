// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/GetImageTopicList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_image_topic_list.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_IMAGE_TOPIC_LIST__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_IMAGE_TOPIC_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/get_image_topic_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetImageTopicList_Request>()
{
  return ::physical_ai_interfaces::srv::GetImageTopicList_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetImageTopicList_Response_message
{
public:
  explicit Init_GetImageTopicList_Response_message(::physical_ai_interfaces::srv::GetImageTopicList_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::GetImageTopicList_Response message(::physical_ai_interfaces::srv::GetImageTopicList_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetImageTopicList_Response msg_;
};

class Init_GetImageTopicList_Response_success
{
public:
  explicit Init_GetImageTopicList_Response_success(::physical_ai_interfaces::srv::GetImageTopicList_Response & msg)
  : msg_(msg)
  {}
  Init_GetImageTopicList_Response_message success(::physical_ai_interfaces::srv::GetImageTopicList_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetImageTopicList_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetImageTopicList_Response msg_;
};

class Init_GetImageTopicList_Response_image_topic_list
{
public:
  Init_GetImageTopicList_Response_image_topic_list()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetImageTopicList_Response_success image_topic_list(::physical_ai_interfaces::srv::GetImageTopicList_Response::_image_topic_list_type arg)
  {
    msg_.image_topic_list = std::move(arg);
    return Init_GetImageTopicList_Response_success(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetImageTopicList_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetImageTopicList_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_GetImageTopicList_Response_image_topic_list();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetImageTopicList_Event_response
{
public:
  explicit Init_GetImageTopicList_Event_response(::physical_ai_interfaces::srv::GetImageTopicList_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::GetImageTopicList_Event response(::physical_ai_interfaces::srv::GetImageTopicList_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetImageTopicList_Event msg_;
};

class Init_GetImageTopicList_Event_request
{
public:
  explicit Init_GetImageTopicList_Event_request(::physical_ai_interfaces::srv::GetImageTopicList_Event & msg)
  : msg_(msg)
  {}
  Init_GetImageTopicList_Event_response request(::physical_ai_interfaces::srv::GetImageTopicList_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetImageTopicList_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetImageTopicList_Event msg_;
};

class Init_GetImageTopicList_Event_info
{
public:
  Init_GetImageTopicList_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetImageTopicList_Event_request info(::physical_ai_interfaces::srv::GetImageTopicList_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetImageTopicList_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::GetImageTopicList_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::GetImageTopicList_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_GetImageTopicList_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_IMAGE_TOPIC_LIST__BUILDER_HPP_
