// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:msg/TaskStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'task_info'
#include "physical_ai_interfaces/msg/detail/task_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__msg__TaskStatus __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__msg__TaskStatus __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskStatus_
{
  using Type = TaskStatus_<ContainerAllocator>;

  explicit TaskStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_info(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_type = "";
      this->phase = 0;
      this->total_time = 0;
      this->proceed_time = 0;
      this->current_episode_number = 0;
      this->current_scenario_number = 0;
      this->current_task_instruction = "";
      this->encoding_progress = 0.0f;
      this->used_storage_size = 0.0f;
      this->total_storage_size = 0.0f;
      this->used_cpu = 0.0f;
      this->used_ram_size = 0.0f;
      this->total_ram_size = 0.0f;
      this->error = "";
    }
  }

  explicit TaskStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_info(_alloc, _init),
    robot_type(_alloc),
    current_task_instruction(_alloc),
    error(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_type = "";
      this->phase = 0;
      this->total_time = 0;
      this->proceed_time = 0;
      this->current_episode_number = 0;
      this->current_scenario_number = 0;
      this->current_task_instruction = "";
      this->encoding_progress = 0.0f;
      this->used_storage_size = 0.0f;
      this->total_storage_size = 0.0f;
      this->used_cpu = 0.0f;
      this->used_ram_size = 0.0f;
      this->total_ram_size = 0.0f;
      this->error = "";
    }
  }

  // field types and members
  using _task_info_type =
    physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator>;
  _task_info_type task_info;
  using _robot_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_type_type robot_type;
  using _phase_type =
    uint8_t;
  _phase_type phase;
  using _total_time_type =
    uint16_t;
  _total_time_type total_time;
  using _proceed_time_type =
    uint16_t;
  _proceed_time_type proceed_time;
  using _current_episode_number_type =
    uint16_t;
  _current_episode_number_type current_episode_number;
  using _current_scenario_number_type =
    uint16_t;
  _current_scenario_number_type current_scenario_number;
  using _current_task_instruction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_task_instruction_type current_task_instruction;
  using _encoding_progress_type =
    float;
  _encoding_progress_type encoding_progress;
  using _used_storage_size_type =
    float;
  _used_storage_size_type used_storage_size;
  using _total_storage_size_type =
    float;
  _total_storage_size_type total_storage_size;
  using _used_cpu_type =
    float;
  _used_cpu_type used_cpu;
  using _used_ram_size_type =
    float;
  _used_ram_size_type used_ram_size;
  using _total_ram_size_type =
    float;
  _total_ram_size_type total_ram_size;
  using _error_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_type error;

  // setters for named parameter idiom
  Type & set__task_info(
    const physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator> & _arg)
  {
    this->task_info = _arg;
    return *this;
  }
  Type & set__robot_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_type = _arg;
    return *this;
  }
  Type & set__phase(
    const uint8_t & _arg)
  {
    this->phase = _arg;
    return *this;
  }
  Type & set__total_time(
    const uint16_t & _arg)
  {
    this->total_time = _arg;
    return *this;
  }
  Type & set__proceed_time(
    const uint16_t & _arg)
  {
    this->proceed_time = _arg;
    return *this;
  }
  Type & set__current_episode_number(
    const uint16_t & _arg)
  {
    this->current_episode_number = _arg;
    return *this;
  }
  Type & set__current_scenario_number(
    const uint16_t & _arg)
  {
    this->current_scenario_number = _arg;
    return *this;
  }
  Type & set__current_task_instruction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_task_instruction = _arg;
    return *this;
  }
  Type & set__encoding_progress(
    const float & _arg)
  {
    this->encoding_progress = _arg;
    return *this;
  }
  Type & set__used_storage_size(
    const float & _arg)
  {
    this->used_storage_size = _arg;
    return *this;
  }
  Type & set__total_storage_size(
    const float & _arg)
  {
    this->total_storage_size = _arg;
    return *this;
  }
  Type & set__used_cpu(
    const float & _arg)
  {
    this->used_cpu = _arg;
    return *this;
  }
  Type & set__used_ram_size(
    const float & _arg)
  {
    this->used_ram_size = _arg;
    return *this;
  }
  Type & set__total_ram_size(
    const float & _arg)
  {
    this->total_ram_size = _arg;
    return *this;
  }
  Type & set__error(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t READY =
    0u;
  static constexpr uint8_t WARMING_UP =
    1u;
  static constexpr uint8_t RESETTING =
    2u;
  static constexpr uint8_t RECORDING =
    3u;
  static constexpr uint8_t SAVING =
    4u;
  static constexpr uint8_t STOPPED =
    5u;
  static constexpr uint8_t INFERENCING =
    6u;

  // pointer types
  using RawPtr =
    physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__msg__TaskStatus
    std::shared_ptr<physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__msg__TaskStatus
    std::shared_ptr<physical_ai_interfaces::msg::TaskStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskStatus_ & other) const
  {
    if (this->task_info != other.task_info) {
      return false;
    }
    if (this->robot_type != other.robot_type) {
      return false;
    }
    if (this->phase != other.phase) {
      return false;
    }
    if (this->total_time != other.total_time) {
      return false;
    }
    if (this->proceed_time != other.proceed_time) {
      return false;
    }
    if (this->current_episode_number != other.current_episode_number) {
      return false;
    }
    if (this->current_scenario_number != other.current_scenario_number) {
      return false;
    }
    if (this->current_task_instruction != other.current_task_instruction) {
      return false;
    }
    if (this->encoding_progress != other.encoding_progress) {
      return false;
    }
    if (this->used_storage_size != other.used_storage_size) {
      return false;
    }
    if (this->total_storage_size != other.total_storage_size) {
      return false;
    }
    if (this->used_cpu != other.used_cpu) {
      return false;
    }
    if (this->used_ram_size != other.used_ram_size) {
      return false;
    }
    if (this->total_ram_size != other.total_ram_size) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskStatus_

// alias to use template instance with default allocator
using TaskStatus =
  physical_ai_interfaces::msg::TaskStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::READY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::WARMING_UP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::RESETTING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::RECORDING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::SAVING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::STOPPED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::INFERENCING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__STRUCT_HPP_
