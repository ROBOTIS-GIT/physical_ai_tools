// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/training_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'training_info'
#include "physical_ai_interfaces/msg/detail/training_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__msg__TrainingStatus __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__msg__TrainingStatus __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrainingStatus_
{
  using Type = TrainingStatus_<ContainerAllocator>;

  explicit TrainingStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : training_info(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_step = 0ul;
      this->current_loss = 0.0f;
      this->is_training = false;
      this->error = "";
    }
  }

  explicit TrainingStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : training_info(_alloc, _init),
    error(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_step = 0ul;
      this->current_loss = 0.0f;
      this->is_training = false;
      this->error = "";
    }
  }

  // field types and members
  using _training_info_type =
    physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator>;
  _training_info_type training_info;
  using _current_step_type =
    uint32_t;
  _current_step_type current_step;
  using _current_loss_type =
    float;
  _current_loss_type current_loss;
  using _is_training_type =
    bool;
  _is_training_type is_training;
  using _error_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_type error;

  // setters for named parameter idiom
  Type & set__training_info(
    const physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator> & _arg)
  {
    this->training_info = _arg;
    return *this;
  }
  Type & set__current_step(
    const uint32_t & _arg)
  {
    this->current_step = _arg;
    return *this;
  }
  Type & set__current_loss(
    const float & _arg)
  {
    this->current_loss = _arg;
    return *this;
  }
  Type & set__is_training(
    const bool & _arg)
  {
    this->is_training = _arg;
    return *this;
  }
  Type & set__error(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__msg__TrainingStatus
    std::shared_ptr<physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__msg__TrainingStatus
    std::shared_ptr<physical_ai_interfaces::msg::TrainingStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrainingStatus_ & other) const
  {
    if (this->training_info != other.training_info) {
      return false;
    }
    if (this->current_step != other.current_step) {
      return false;
    }
    if (this->current_loss != other.current_loss) {
      return false;
    }
    if (this->is_training != other.is_training) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrainingStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrainingStatus_

// alias to use template instance with default allocator
using TrainingStatus =
  physical_ai_interfaces::msg::TrainingStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__STRUCT_HPP_
