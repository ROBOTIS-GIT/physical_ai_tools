// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:msg/BrowserItem.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/browser_item.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__msg__BrowserItem __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__msg__BrowserItem __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BrowserItem_
{
  using Type = BrowserItem_<ContainerAllocator>;

  explicit BrowserItem_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->full_path = "";
      this->is_directory = false;
      this->size = 0ll;
      this->modified_time = "";
      this->has_target_file = false;
    }
  }

  explicit BrowserItem_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    full_path(_alloc),
    modified_time(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->full_path = "";
      this->is_directory = false;
      this->size = 0ll;
      this->modified_time = "";
      this->has_target_file = false;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _full_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _full_path_type full_path;
  using _is_directory_type =
    bool;
  _is_directory_type is_directory;
  using _size_type =
    int64_t;
  _size_type size;
  using _modified_time_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _modified_time_type modified_time;
  using _has_target_file_type =
    bool;
  _has_target_file_type has_target_file;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__full_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->full_path = _arg;
    return *this;
  }
  Type & set__is_directory(
    const bool & _arg)
  {
    this->is_directory = _arg;
    return *this;
  }
  Type & set__size(
    const int64_t & _arg)
  {
    this->size = _arg;
    return *this;
  }
  Type & set__modified_time(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->modified_time = _arg;
    return *this;
  }
  Type & set__has_target_file(
    const bool & _arg)
  {
    this->has_target_file = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__msg__BrowserItem
    std::shared_ptr<physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__msg__BrowserItem
    std::shared_ptr<physical_ai_interfaces::msg::BrowserItem_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BrowserItem_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->full_path != other.full_path) {
      return false;
    }
    if (this->is_directory != other.is_directory) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    if (this->modified_time != other.modified_time) {
      return false;
    }
    if (this->has_target_file != other.has_target_file) {
      return false;
    }
    return true;
  }
  bool operator!=(const BrowserItem_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BrowserItem_

// alias to use template instance with default allocator
using BrowserItem =
  physical_ai_interfaces::msg::BrowserItem_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__STRUCT_HPP_
