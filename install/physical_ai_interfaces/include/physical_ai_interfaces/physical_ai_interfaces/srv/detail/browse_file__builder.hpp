// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/BrowseFile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/browse_file.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/browse_file__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_BrowseFile_Request_target_folders
{
public:
  explicit Init_BrowseFile_Request_target_folders(::physical_ai_interfaces::srv::BrowseFile_Request & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::BrowseFile_Request target_folders(::physical_ai_interfaces::srv::BrowseFile_Request::_target_folders_type arg)
  {
    msg_.target_folders = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Request msg_;
};

class Init_BrowseFile_Request_target_files
{
public:
  explicit Init_BrowseFile_Request_target_files(::physical_ai_interfaces::srv::BrowseFile_Request & msg)
  : msg_(msg)
  {}
  Init_BrowseFile_Request_target_folders target_files(::physical_ai_interfaces::srv::BrowseFile_Request::_target_files_type arg)
  {
    msg_.target_files = std::move(arg);
    return Init_BrowseFile_Request_target_folders(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Request msg_;
};

class Init_BrowseFile_Request_target_name
{
public:
  explicit Init_BrowseFile_Request_target_name(::physical_ai_interfaces::srv::BrowseFile_Request & msg)
  : msg_(msg)
  {}
  Init_BrowseFile_Request_target_files target_name(::physical_ai_interfaces::srv::BrowseFile_Request::_target_name_type arg)
  {
    msg_.target_name = std::move(arg);
    return Init_BrowseFile_Request_target_files(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Request msg_;
};

class Init_BrowseFile_Request_current_path
{
public:
  explicit Init_BrowseFile_Request_current_path(::physical_ai_interfaces::srv::BrowseFile_Request & msg)
  : msg_(msg)
  {}
  Init_BrowseFile_Request_target_name current_path(::physical_ai_interfaces::srv::BrowseFile_Request::_current_path_type arg)
  {
    msg_.current_path = std::move(arg);
    return Init_BrowseFile_Request_target_name(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Request msg_;
};

class Init_BrowseFile_Request_action
{
public:
  Init_BrowseFile_Request_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BrowseFile_Request_current_path action(::physical_ai_interfaces::srv::BrowseFile_Request::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_BrowseFile_Request_current_path(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::BrowseFile_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_BrowseFile_Request_action();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_BrowseFile_Response_items
{
public:
  explicit Init_BrowseFile_Response_items(::physical_ai_interfaces::srv::BrowseFile_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::BrowseFile_Response items(::physical_ai_interfaces::srv::BrowseFile_Response::_items_type arg)
  {
    msg_.items = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Response msg_;
};

class Init_BrowseFile_Response_selected_path
{
public:
  explicit Init_BrowseFile_Response_selected_path(::physical_ai_interfaces::srv::BrowseFile_Response & msg)
  : msg_(msg)
  {}
  Init_BrowseFile_Response_items selected_path(::physical_ai_interfaces::srv::BrowseFile_Response::_selected_path_type arg)
  {
    msg_.selected_path = std::move(arg);
    return Init_BrowseFile_Response_items(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Response msg_;
};

class Init_BrowseFile_Response_parent_path
{
public:
  explicit Init_BrowseFile_Response_parent_path(::physical_ai_interfaces::srv::BrowseFile_Response & msg)
  : msg_(msg)
  {}
  Init_BrowseFile_Response_selected_path parent_path(::physical_ai_interfaces::srv::BrowseFile_Response::_parent_path_type arg)
  {
    msg_.parent_path = std::move(arg);
    return Init_BrowseFile_Response_selected_path(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Response msg_;
};

class Init_BrowseFile_Response_current_path
{
public:
  explicit Init_BrowseFile_Response_current_path(::physical_ai_interfaces::srv::BrowseFile_Response & msg)
  : msg_(msg)
  {}
  Init_BrowseFile_Response_parent_path current_path(::physical_ai_interfaces::srv::BrowseFile_Response::_current_path_type arg)
  {
    msg_.current_path = std::move(arg);
    return Init_BrowseFile_Response_parent_path(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Response msg_;
};

class Init_BrowseFile_Response_message
{
public:
  explicit Init_BrowseFile_Response_message(::physical_ai_interfaces::srv::BrowseFile_Response & msg)
  : msg_(msg)
  {}
  Init_BrowseFile_Response_current_path message(::physical_ai_interfaces::srv::BrowseFile_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_BrowseFile_Response_current_path(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Response msg_;
};

class Init_BrowseFile_Response_success
{
public:
  Init_BrowseFile_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BrowseFile_Response_message success(::physical_ai_interfaces::srv::BrowseFile_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_BrowseFile_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::BrowseFile_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_BrowseFile_Response_success();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_BrowseFile_Event_response
{
public:
  explicit Init_BrowseFile_Event_response(::physical_ai_interfaces::srv::BrowseFile_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::BrowseFile_Event response(::physical_ai_interfaces::srv::BrowseFile_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Event msg_;
};

class Init_BrowseFile_Event_request
{
public:
  explicit Init_BrowseFile_Event_request(::physical_ai_interfaces::srv::BrowseFile_Event & msg)
  : msg_(msg)
  {}
  Init_BrowseFile_Event_response request(::physical_ai_interfaces::srv::BrowseFile_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_BrowseFile_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Event msg_;
};

class Init_BrowseFile_Event_info
{
public:
  Init_BrowseFile_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BrowseFile_Event_request info(::physical_ai_interfaces::srv::BrowseFile_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_BrowseFile_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::BrowseFile_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::BrowseFile_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_BrowseFile_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__BUILDER_HPP_
