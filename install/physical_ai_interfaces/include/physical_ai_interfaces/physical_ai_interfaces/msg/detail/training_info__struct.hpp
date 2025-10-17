// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:msg/TrainingInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/training_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__msg__TrainingInfo __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__msg__TrainingInfo __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrainingInfo_
{
  using Type = TrainingInfo_<ContainerAllocator>;

  explicit TrainingInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dataset = "";
      this->policy_type = "";
      this->output_folder_name = "";
      this->policy_device = "";
      this->seed = 0ul;
      this->num_workers = 0;
      this->batch_size = 0;
      this->steps = 0ul;
      this->eval_freq = 0ul;
      this->log_freq = 0ul;
      this->save_freq = 0ul;
    }
  }

  explicit TrainingInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dataset(_alloc),
    policy_type(_alloc),
    output_folder_name(_alloc),
    policy_device(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dataset = "";
      this->policy_type = "";
      this->output_folder_name = "";
      this->policy_device = "";
      this->seed = 0ul;
      this->num_workers = 0;
      this->batch_size = 0;
      this->steps = 0ul;
      this->eval_freq = 0ul;
      this->log_freq = 0ul;
      this->save_freq = 0ul;
    }
  }

  // field types and members
  using _dataset_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dataset_type dataset;
  using _policy_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _policy_type_type policy_type;
  using _output_folder_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _output_folder_name_type output_folder_name;
  using _policy_device_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _policy_device_type policy_device;
  using _seed_type =
    uint32_t;
  _seed_type seed;
  using _num_workers_type =
    uint8_t;
  _num_workers_type num_workers;
  using _batch_size_type =
    uint16_t;
  _batch_size_type batch_size;
  using _steps_type =
    uint32_t;
  _steps_type steps;
  using _eval_freq_type =
    uint32_t;
  _eval_freq_type eval_freq;
  using _log_freq_type =
    uint32_t;
  _log_freq_type log_freq;
  using _save_freq_type =
    uint32_t;
  _save_freq_type save_freq;

  // setters for named parameter idiom
  Type & set__dataset(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dataset = _arg;
    return *this;
  }
  Type & set__policy_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->policy_type = _arg;
    return *this;
  }
  Type & set__output_folder_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->output_folder_name = _arg;
    return *this;
  }
  Type & set__policy_device(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->policy_device = _arg;
    return *this;
  }
  Type & set__seed(
    const uint32_t & _arg)
  {
    this->seed = _arg;
    return *this;
  }
  Type & set__num_workers(
    const uint8_t & _arg)
  {
    this->num_workers = _arg;
    return *this;
  }
  Type & set__batch_size(
    const uint16_t & _arg)
  {
    this->batch_size = _arg;
    return *this;
  }
  Type & set__steps(
    const uint32_t & _arg)
  {
    this->steps = _arg;
    return *this;
  }
  Type & set__eval_freq(
    const uint32_t & _arg)
  {
    this->eval_freq = _arg;
    return *this;
  }
  Type & set__log_freq(
    const uint32_t & _arg)
  {
    this->log_freq = _arg;
    return *this;
  }
  Type & set__save_freq(
    const uint32_t & _arg)
  {
    this->save_freq = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__msg__TrainingInfo
    std::shared_ptr<physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__msg__TrainingInfo
    std::shared_ptr<physical_ai_interfaces::msg::TrainingInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrainingInfo_ & other) const
  {
    if (this->dataset != other.dataset) {
      return false;
    }
    if (this->policy_type != other.policy_type) {
      return false;
    }
    if (this->output_folder_name != other.output_folder_name) {
      return false;
    }
    if (this->policy_device != other.policy_device) {
      return false;
    }
    if (this->seed != other.seed) {
      return false;
    }
    if (this->num_workers != other.num_workers) {
      return false;
    }
    if (this->batch_size != other.batch_size) {
      return false;
    }
    if (this->steps != other.steps) {
      return false;
    }
    if (this->eval_freq != other.eval_freq) {
      return false;
    }
    if (this->log_freq != other.log_freq) {
      return false;
    }
    if (this->save_freq != other.save_freq) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrainingInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrainingInfo_

// alias to use template instance with default allocator
using TrainingInfo =
  physical_ai_interfaces::msg::TrainingInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__STRUCT_HPP_
