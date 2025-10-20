// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from physical_ai_interfaces:msg/BrowserItem.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/browser_item.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__BUILDER_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "physical_ai_interfaces/msg/detail/browser_item__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace physical_ai_interfaces
{

namespace msg
{

namespace builder
{

class Init_BrowserItem_has_target_file
{
public:
  explicit Init_BrowserItem_has_target_file(::physical_ai_interfaces::msg::BrowserItem & msg)
  : msg_(msg)
  {}
  ::physical_ai_interfaces::msg::BrowserItem has_target_file(::physical_ai_interfaces::msg::BrowserItem::_has_target_file_type arg)
  {
    msg_.has_target_file = std::move(arg);
    return std::move(msg_);
  }

private:
  ::physical_ai_interfaces::msg::BrowserItem msg_;
};

class Init_BrowserItem_modified_time
{
public:
  explicit Init_BrowserItem_modified_time(::physical_ai_interfaces::msg::BrowserItem & msg)
  : msg_(msg)
  {}
  Init_BrowserItem_has_target_file modified_time(::physical_ai_interfaces::msg::BrowserItem::_modified_time_type arg)
  {
    msg_.modified_time = std::move(arg);
    return Init_BrowserItem_has_target_file(msg_);
  }

private:
  ::physical_ai_interfaces::msg::BrowserItem msg_;
};

class Init_BrowserItem_size
{
public:
  explicit Init_BrowserItem_size(::physical_ai_interfaces::msg::BrowserItem & msg)
  : msg_(msg)
  {}
  Init_BrowserItem_modified_time size(::physical_ai_interfaces::msg::BrowserItem::_size_type arg)
  {
    msg_.size = std::move(arg);
    return Init_BrowserItem_modified_time(msg_);
  }

private:
  ::physical_ai_interfaces::msg::BrowserItem msg_;
};

class Init_BrowserItem_is_directory
{
public:
  explicit Init_BrowserItem_is_directory(::physical_ai_interfaces::msg::BrowserItem & msg)
  : msg_(msg)
  {}
  Init_BrowserItem_size is_directory(::physical_ai_interfaces::msg::BrowserItem::_is_directory_type arg)
  {
    msg_.is_directory = std::move(arg);
    return Init_BrowserItem_size(msg_);
  }

private:
  ::physical_ai_interfaces::msg::BrowserItem msg_;
};

class Init_BrowserItem_full_path
{
public:
  explicit Init_BrowserItem_full_path(::physical_ai_interfaces::msg::BrowserItem & msg)
  : msg_(msg)
  {}
  Init_BrowserItem_is_directory full_path(::physical_ai_interfaces::msg::BrowserItem::_full_path_type arg)
  {
    msg_.full_path = std::move(arg);
    return Init_BrowserItem_is_directory(msg_);
  }

private:
  ::physical_ai_interfaces::msg::BrowserItem msg_;
};

class Init_BrowserItem_name
{
public:
  Init_BrowserItem_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BrowserItem_full_path name(::physical_ai_interfaces::msg::BrowserItem::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_BrowserItem_full_path(msg_);
  }

private:
  ::physical_ai_interfaces::msg::BrowserItem msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::physical_ai_interfaces::msg::BrowserItem>()
{
  return physical_ai_interfaces::msg::builder::Init_BrowserItem_name();
}

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__BUILDER_HPP_
