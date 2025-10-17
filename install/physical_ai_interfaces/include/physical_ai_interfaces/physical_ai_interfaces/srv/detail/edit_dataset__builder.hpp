// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:srv/EditDataset.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/edit_dataset.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/srv/detail/edit_dataset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_EditDataset_Request_upload_huggingface
{
public:
  explicit Init_EditDataset_Request_upload_huggingface(::physical_ai_interfaces::srv::EditDataset_Request & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::EditDataset_Request upload_huggingface(::physical_ai_interfaces::srv::EditDataset_Request::_upload_huggingface_type arg)
  {
    msg_.upload_huggingface = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Request msg_;
};

class Init_EditDataset_Request_delete_episode_num
{
public:
  explicit Init_EditDataset_Request_delete_episode_num(::physical_ai_interfaces::srv::EditDataset_Request & msg)
  : msg_(msg)
  {}
  Init_EditDataset_Request_upload_huggingface delete_episode_num(::physical_ai_interfaces::srv::EditDataset_Request::_delete_episode_num_type arg)
  {
    msg_.delete_episode_num = std::move(arg);
    return Init_EditDataset_Request_upload_huggingface(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Request msg_;
};

class Init_EditDataset_Request_output_path
{
public:
  explicit Init_EditDataset_Request_output_path(::physical_ai_interfaces::srv::EditDataset_Request & msg)
  : msg_(msg)
  {}
  Init_EditDataset_Request_delete_episode_num output_path(::physical_ai_interfaces::srv::EditDataset_Request::_output_path_type arg)
  {
    msg_.output_path = std::move(arg);
    return Init_EditDataset_Request_delete_episode_num(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Request msg_;
};

class Init_EditDataset_Request_delete_dataset_path
{
public:
  explicit Init_EditDataset_Request_delete_dataset_path(::physical_ai_interfaces::srv::EditDataset_Request & msg)
  : msg_(msg)
  {}
  Init_EditDataset_Request_output_path delete_dataset_path(::physical_ai_interfaces::srv::EditDataset_Request::_delete_dataset_path_type arg)
  {
    msg_.delete_dataset_path = std::move(arg);
    return Init_EditDataset_Request_output_path(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Request msg_;
};

class Init_EditDataset_Request_merge_dataset_list
{
public:
  explicit Init_EditDataset_Request_merge_dataset_list(::physical_ai_interfaces::srv::EditDataset_Request & msg)
  : msg_(msg)
  {}
  Init_EditDataset_Request_delete_dataset_path merge_dataset_list(::physical_ai_interfaces::srv::EditDataset_Request::_merge_dataset_list_type arg)
  {
    msg_.merge_dataset_list = std::move(arg);
    return Init_EditDataset_Request_delete_dataset_path(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Request msg_;
};

class Init_EditDataset_Request_mode
{
public:
  Init_EditDataset_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EditDataset_Request_merge_dataset_list mode(::physical_ai_interfaces::srv::EditDataset_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_EditDataset_Request_merge_dataset_list(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::EditDataset_Request>()
{
  return physical_ai_interfaces::srv::builder::Init_EditDataset_Request_mode();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_EditDataset_Response_message
{
public:
  explicit Init_EditDataset_Response_message(::physical_ai_interfaces::srv::EditDataset_Response & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::EditDataset_Response message(::physical_ai_interfaces::srv::EditDataset_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Response msg_;
};

class Init_EditDataset_Response_success
{
public:
  Init_EditDataset_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EditDataset_Response_message success(::physical_ai_interfaces::srv::EditDataset_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_EditDataset_Response_message(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::EditDataset_Response>()
{
  return physical_ai_interfaces::srv::builder::Init_EditDataset_Response_success();
}

}  // namespace physical_ai_interfaces


namespace physical_ai_interfaces
{

namespace srv
{

namespace builder
{

class Init_EditDataset_Event_response
{
public:
  explicit Init_EditDataset_Event_response(::physical_ai_interfaces::srv::EditDataset_Event & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::srv::EditDataset_Event response(::physical_ai_interfaces::srv::EditDataset_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Event msg_;
};

class Init_EditDataset_Event_request
{
public:
  explicit Init_EditDataset_Event_request(::physical_ai_interfaces::srv::EditDataset_Event & msg)
  : msg_(msg)
  {}
  Init_EditDataset_Event_response request(::physical_ai_interfaces::srv::EditDataset_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_EditDataset_Event_response(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Event msg_;
};

class Init_EditDataset_Event_info
{
public:
  Init_EditDataset_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EditDataset_Event_request info(::physical_ai_interfaces::srv::EditDataset_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_EditDataset_Event_request(msg_);
  }

private:
  ::physical_ai_interfaces::srv::EditDataset_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::srv::EditDataset_Event>()
{
  return physical_ai_interfaces::srv::builder::Init_EditDataset_Event_info();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__BUILDER_HPP_
