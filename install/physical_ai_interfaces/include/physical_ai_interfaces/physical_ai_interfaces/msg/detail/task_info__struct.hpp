// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:msg/TaskInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__msg__TaskInfo __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__msg__TaskInfo __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskInfo_
{
  using Type = TaskInfo_<ContainerAllocator>;

  explicit TaskInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_name = "";
      this->task_type = "";
      this->user_id = "";
      this->policy_path = "";
      this->fps = 0;
      this->warmup_time_s = 0;
      this->episode_time_s = 0;
      this->reset_time_s = 0;
      this->num_episodes = 0;
      this->push_to_hub = false;
      this->private_mode = false;
      this->use_optimized_save_mode = false;
      this->record_inference_mode = false;
    }
  }

  explicit TaskInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_name(_alloc),
    task_type(_alloc),
    user_id(_alloc),
    policy_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_name = "";
      this->task_type = "";
      this->user_id = "";
      this->policy_path = "";
      this->fps = 0;
      this->warmup_time_s = 0;
      this->episode_time_s = 0;
      this->reset_time_s = 0;
      this->num_episodes = 0;
      this->push_to_hub = false;
      this->private_mode = false;
      this->use_optimized_save_mode = false;
      this->record_inference_mode = false;
    }
  }

  // field types and members
  using _task_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_name_type task_name;
  using _task_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_type_type task_type;
  using _user_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _user_id_type user_id;
  using _task_instruction_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _task_instruction_type task_instruction;
  using _policy_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _policy_path_type policy_path;
  using _fps_type =
    uint8_t;
  _fps_type fps;
  using _tags_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _tags_type tags;
  using _warmup_time_s_type =
    uint16_t;
  _warmup_time_s_type warmup_time_s;
  using _episode_time_s_type =
    uint16_t;
  _episode_time_s_type episode_time_s;
  using _reset_time_s_type =
    uint16_t;
  _reset_time_s_type reset_time_s;
  using _num_episodes_type =
    uint16_t;
  _num_episodes_type num_episodes;
  using _push_to_hub_type =
    bool;
  _push_to_hub_type push_to_hub;
  using _private_mode_type =
    bool;
  _private_mode_type private_mode;
  using _use_optimized_save_mode_type =
    bool;
  _use_optimized_save_mode_type use_optimized_save_mode;
  using _record_inference_mode_type =
    bool;
  _record_inference_mode_type record_inference_mode;

  // setters for named parameter idiom
  Type & set__task_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_name = _arg;
    return *this;
  }
  Type & set__task_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_type = _arg;
    return *this;
  }
  Type & set__user_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->user_id = _arg;
    return *this;
  }
  Type & set__task_instruction(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->task_instruction = _arg;
    return *this;
  }
  Type & set__policy_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->policy_path = _arg;
    return *this;
  }
  Type & set__fps(
    const uint8_t & _arg)
  {
    this->fps = _arg;
    return *this;
  }
  Type & set__tags(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->tags = _arg;
    return *this;
  }
  Type & set__warmup_time_s(
    const uint16_t & _arg)
  {
    this->warmup_time_s = _arg;
    return *this;
  }
  Type & set__episode_time_s(
    const uint16_t & _arg)
  {
    this->episode_time_s = _arg;
    return *this;
  }
  Type & set__reset_time_s(
    const uint16_t & _arg)
  {
    this->reset_time_s = _arg;
    return *this;
  }
  Type & set__num_episodes(
    const uint16_t & _arg)
  {
    this->num_episodes = _arg;
    return *this;
  }
  Type & set__push_to_hub(
    const bool & _arg)
  {
    this->push_to_hub = _arg;
    return *this;
  }
  Type & set__private_mode(
    const bool & _arg)
  {
    this->private_mode = _arg;
    return *this;
  }
  Type & set__use_optimized_save_mode(
    const bool & _arg)
  {
    this->use_optimized_save_mode = _arg;
    return *this;
  }
  Type & set__record_inference_mode(
    const bool & _arg)
  {
    this->record_inference_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__msg__TaskInfo
    std::shared_ptr<physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__msg__TaskInfo
    std::shared_ptr<physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskInfo_ & other) const
  {
    if (this->task_name != other.task_name) {
      return false;
    }
    if (this->task_type != other.task_type) {
      return false;
    }
    if (this->user_id != other.user_id) {
      return false;
    }
    if (this->task_instruction != other.task_instruction) {
      return false;
    }
    if (this->policy_path != other.policy_path) {
      return false;
    }
    if (this->fps != other.fps) {
      return false;
    }
    if (this->tags != other.tags) {
      return false;
    }
    if (this->warmup_time_s != other.warmup_time_s) {
      return false;
    }
    if (this->episode_time_s != other.episode_time_s) {
      return false;
    }
    if (this->reset_time_s != other.reset_time_s) {
      return false;
    }
    if (this->num_episodes != other.num_episodes) {
      return false;
    }
    if (this->push_to_hub != other.push_to_hub) {
      return false;
    }
    if (this->private_mode != other.private_mode) {
      return false;
    }
    if (this->use_optimized_save_mode != other.use_optimized_save_mode) {
      return false;
    }
    if (this->record_inference_mode != other.record_inference_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskInfo_

// alias to use template instance with default allocator
using TaskInfo =
  physical_ai_interfaces::msg::TaskInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__STRUCT_HPP_
