// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:msg/DatasetInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/dataset_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__msg__DatasetInfo __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__msg__DatasetInfo __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DatasetInfo_
{
  using Type = DatasetInfo_<ContainerAllocator>;

  explicit DatasetInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->codebase_version = "";
      this->robot_type = "";
      this->total_episodes = 0;
      this->total_tasks = 0;
      this->fps = 0;
    }
  }

  explicit DatasetInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : codebase_version(_alloc),
    robot_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->codebase_version = "";
      this->robot_type = "";
      this->total_episodes = 0;
      this->total_tasks = 0;
      this->fps = 0;
    }
  }

  // field types and members
  using _codebase_version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _codebase_version_type codebase_version;
  using _robot_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_type_type robot_type;
  using _total_episodes_type =
    uint16_t;
  _total_episodes_type total_episodes;
  using _total_tasks_type =
    uint16_t;
  _total_tasks_type total_tasks;
  using _fps_type =
    uint8_t;
  _fps_type fps;

  // setters for named parameter idiom
  Type & set__codebase_version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->codebase_version = _arg;
    return *this;
  }
  Type & set__robot_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_type = _arg;
    return *this;
  }
  Type & set__total_episodes(
    const uint16_t & _arg)
  {
    this->total_episodes = _arg;
    return *this;
  }
  Type & set__total_tasks(
    const uint16_t & _arg)
  {
    this->total_tasks = _arg;
    return *this;
  }
  Type & set__fps(
    const uint8_t & _arg)
  {
    this->fps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__msg__DatasetInfo
    std::shared_ptr<physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__msg__DatasetInfo
    std::shared_ptr<physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DatasetInfo_ & other) const
  {
    if (this->codebase_version != other.codebase_version) {
      return false;
    }
    if (this->robot_type != other.robot_type) {
      return false;
    }
    if (this->total_episodes != other.total_episodes) {
      return false;
    }
    if (this->total_tasks != other.total_tasks) {
      return false;
    }
    if (this->fps != other.fps) {
      return false;
    }
    return true;
  }
  bool operator!=(const DatasetInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DatasetInfo_

// alias to use template instance with default allocator
using DatasetInfo =
  physical_ai_interfaces::msg::DatasetInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__STRUCT_HPP_
