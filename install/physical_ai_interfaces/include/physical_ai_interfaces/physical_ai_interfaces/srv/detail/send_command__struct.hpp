// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:srv/SendCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/send_command.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__physical_ai_interfaces__srv__SendCommand_Request __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__SendCommand_Request __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendCommand_Request_
{
  using Type = SendCommand_Request_<ContainerAllocator>;

  explicit SendCommand_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_info(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  explicit SendCommand_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_info(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  // field types and members
  using _command_type =
    uint8_t;
  _command_type command;
  using _task_info_type =
    physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator>;
  _task_info_type task_info;

  // setters for named parameter idiom
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__task_info(
    const physical_ai_interfaces::msg::TaskInfo_<ContainerAllocator> & _arg)
  {
    this->task_info = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t IDLE =
    0u;
  static constexpr uint8_t START_RECORD =
    1u;
  static constexpr uint8_t START_INFERENCE =
    2u;
  static constexpr uint8_t STOP =
    3u;
  static constexpr uint8_t MOVE_TO_NEXT =
    4u;
  static constexpr uint8_t RERECORD =
    5u;
  static constexpr uint8_t FINISH =
    6u;
  static constexpr uint8_t SKIP_TASK =
    7u;

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__SendCommand_Request
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__SendCommand_Request
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendCommand_Request_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->task_info != other.task_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendCommand_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendCommand_Request_

// alias to use template instance with default allocator
using SendCommand_Request =
  physical_ai_interfaces::srv::SendCommand_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SendCommand_Request_<ContainerAllocator>::IDLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SendCommand_Request_<ContainerAllocator>::START_RECORD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SendCommand_Request_<ContainerAllocator>::START_INFERENCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SendCommand_Request_<ContainerAllocator>::STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SendCommand_Request_<ContainerAllocator>::MOVE_TO_NEXT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SendCommand_Request_<ContainerAllocator>::RERECORD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SendCommand_Request_<ContainerAllocator>::FINISH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SendCommand_Request_<ContainerAllocator>::SKIP_TASK;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace physical_ai_interfaces


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__SendCommand_Response __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__SendCommand_Response __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendCommand_Response_
{
  using Type = SendCommand_Response_<ContainerAllocator>;

  explicit SendCommand_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SendCommand_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__SendCommand_Response
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__SendCommand_Response
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendCommand_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendCommand_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendCommand_Response_

// alias to use template instance with default allocator
using SendCommand_Response =
  physical_ai_interfaces::srv::SendCommand_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__SendCommand_Event __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__SendCommand_Event __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendCommand_Event_
{
  using Type = SendCommand_Event_<ContainerAllocator>;

  explicit SendCommand_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SendCommand_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::SendCommand_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::SendCommand_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__SendCommand_Event
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__SendCommand_Event
    std::shared_ptr<physical_ai_interfaces::srv::SendCommand_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendCommand_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendCommand_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendCommand_Event_

// alias to use template instance with default allocator
using SendCommand_Event =
  physical_ai_interfaces::srv::SendCommand_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces

namespace physical_ai_interfaces
{

namespace srv
{

struct SendCommand
{
  using Request = physical_ai_interfaces::srv::SendCommand_Request;
  using Response = physical_ai_interfaces::srv::SendCommand_Response;
  using Event = physical_ai_interfaces::srv::SendCommand_Event;
};

}  // namespace srv

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__STRUCT_HPP_
