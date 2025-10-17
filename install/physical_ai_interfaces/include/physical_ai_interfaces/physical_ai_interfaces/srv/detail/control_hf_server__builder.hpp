// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/ControlHfServer.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/control_hf_server.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/control_hf_server__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_ControlHfServer_Request_author
{
public:
  explicit Init_ControlHfServer_Request_author(::physical_ai_interfaces::srv::ControlHfServer_Request & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::ControlHfServer_Request author(::physical_ai_interfaces::srv::ControlHfServer_Request::_author_type arg)
  {
    msg_.author = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Request msg_;
};

class Init_ControlHfServer_Request_repo_type
{
public:
  explicit Init_ControlHfServer_Request_repo_type(::physical_ai_interfaces::srv::ControlHfServer_Request & msg)
  : msg_(msg)
  {}
  Init_ControlHfServer_Request_author repo_type(::physical_ai_interfaces::srv::ControlHfServer_Request::_repo_type_type arg)
  {
    msg_.repo_type = std::move(arg);
    return Init_ControlHfServer_Request_author(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Request msg_;
};

class Init_ControlHfServer_Request_local_dir
{
public:
  explicit Init_ControlHfServer_Request_local_dir(::physical_ai_interfaces::srv::ControlHfServer_Request & msg)
  : msg_(msg)
  {}
  Init_ControlHfServer_Request_repo_type local_dir(::physical_ai_interfaces::srv::ControlHfServer_Request::_local_dir_type arg)
  {
    msg_.local_dir = std::move(arg);
    return Init_ControlHfServer_Request_repo_type(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Request msg_;
};

class Init_ControlHfServer_Request_repo_id
{
public:
  explicit Init_ControlHfServer_Request_repo_id(::physical_ai_interfaces::srv::ControlHfServer_Request & msg)
  : msg_(msg)
  {}
  Init_ControlHfServer_Request_local_dir repo_id(::physical_ai_interfaces::srv::ControlHfServer_Request::_repo_id_type arg)
  {
    msg_.repo_id = std::move(arg);
    return Init_ControlHfServer_Request_local_dir(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Request msg_;
};

class Init_ControlHfServer_Request_mode
{
public:
  Init_ControlHfServer_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlHfServer_Request_repo_id mode(::physical_ai_interfaces::srv::ControlHfServer_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_ControlHfServer_Request_repo_id(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::ControlHfServer_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_ControlHfServer_Request_mode();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_ControlHfServer_Response_message
{
public:
  explicit Init_ControlHfServer_Response_message(::physical_ai_interfaces::srv::ControlHfServer_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::ControlHfServer_Response message(::physical_ai_interfaces::srv::ControlHfServer_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Response msg_;
};

class Init_ControlHfServer_Response_success
{
public:
  Init_ControlHfServer_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlHfServer_Response_message success(::physical_ai_interfaces::srv::ControlHfServer_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ControlHfServer_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::ControlHfServer_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_ControlHfServer_Response_success();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_ControlHfServer_Event_response
{
public:
  explicit Init_ControlHfServer_Event_response(::physical_ai_interfaces::srv::ControlHfServer_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::ControlHfServer_Event response(::physical_ai_interfaces::srv::ControlHfServer_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Event msg_;
};

class Init_ControlHfServer_Event_request
{
public:
  explicit Init_ControlHfServer_Event_request(::physical_ai_interfaces::srv::ControlHfServer_Event & msg)
  : msg_(msg)
  {}
  Init_ControlHfServer_Event_response request(::physical_ai_interfaces::srv::ControlHfServer_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ControlHfServer_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Event msg_;
};

class Init_ControlHfServer_Event_info
{
public:
  Init_ControlHfServer_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlHfServer_Event_request info(::physical_ai_interfaces::srv::ControlHfServer_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ControlHfServer_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::ControlHfServer_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::ControlHfServer_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_ControlHfServer_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__BUILDER_HPP_
