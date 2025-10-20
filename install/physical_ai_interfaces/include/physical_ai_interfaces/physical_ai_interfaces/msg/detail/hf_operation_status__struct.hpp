// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:msg/HFOperationStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/hf_operation_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__msg__HFOperationStatus __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__msg__HFOperationStatus __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HFOperationStatus_
{
  using Type = HFOperationStatus_<ContainerAllocator>;

  explicit HFOperationStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operation = "";
      this->status = "";
      this->local_path = "";
      this->repo_id = "";
      this->message = "";
      this->progress_current = 0;
      this->progress_total = 0;
      this->progress_percentage = 0.0f;
    }
  }

  explicit HFOperationStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : operation(_alloc),
    status(_alloc),
    local_path(_alloc),
    repo_id(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operation = "";
      this->status = "";
      this->local_path = "";
      this->repo_id = "";
      this->message = "";
      this->progress_current = 0;
      this->progress_total = 0;
      this->progress_percentage = 0.0f;
    }
  }

  // field types and members
  using _operation_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operation_type operation;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _local_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _local_path_type local_path;
  using _repo_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _repo_id_type repo_id;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _progress_current_type =
    uint16_t;
  _progress_current_type progress_current;
  using _progress_total_type =
    uint16_t;
  _progress_total_type progress_total;
  using _progress_percentage_type =
    float;
  _progress_percentage_type progress_percentage;

  // setters for named parameter idiom
  Type & set__operation(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operation = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__local_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->local_path = _arg;
    return *this;
  }
  Type & set__repo_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->repo_id = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__progress_current(
    const uint16_t & _arg)
  {
    this->progress_current = _arg;
    return *this;
  }
  Type & set__progress_total(
    const uint16_t & _arg)
  {
    this->progress_total = _arg;
    return *this;
  }
  Type & set__progress_percentage(
    const float & _arg)
  {
    this->progress_percentage = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__msg__HFOperationStatus
    std::shared_ptr<physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__msg__HFOperationStatus
    std::shared_ptr<physical_ai_interfaces::msg::HFOperationStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HFOperationStatus_ & other) const
  {
    if (this->operation != other.operation) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->local_path != other.local_path) {
      return false;
    }
    if (this->repo_id != other.repo_id) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->progress_current != other.progress_current) {
      return false;
    }
    if (this->progress_total != other.progress_total) {
      return false;
    }
    if (this->progress_percentage != other.progress_percentage) {
      return false;
    }
    return true;
  }
  bool operator!=(const HFOperationStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HFOperationStatus_

// alias to use template instance with default allocator
using HFOperationStatus =
  physical_ai_interfaces::msg::HFOperationStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__STRUCT_HPP_
