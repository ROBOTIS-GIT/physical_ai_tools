// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:srv/ControlHfServer.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/control_hf_server.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Request __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Request __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlHfServer_Request_
{
  using Type = ControlHfServer_Request_<ContainerAllocator>;

  explicit ControlHfServer_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = "";
      this->repo_id = "";
      this->local_dir = "";
      this->repo_type = "";
      this->author = "";
    }
  }

  explicit ControlHfServer_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mode(_alloc),
    repo_id(_alloc),
    local_dir(_alloc),
    repo_type(_alloc),
    author(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = "";
      this->repo_id = "";
      this->local_dir = "";
      this->repo_type = "";
      this->author = "";
    }
  }

  // field types and members
  using _mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mode_type mode;
  using _repo_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _repo_id_type repo_id;
  using _local_dir_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _local_dir_type local_dir;
  using _repo_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _repo_type_type repo_type;
  using _author_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _author_type author;

  // setters for named parameter idiom
  Type & set__mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__repo_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->repo_id = _arg;
    return *this;
  }
  Type & set__local_dir(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->local_dir = _arg;
    return *this;
  }
  Type & set__repo_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->repo_type = _arg;
    return *this;
  }
  Type & set__author(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->author = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Request
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Request
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlHfServer_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->repo_id != other.repo_id) {
      return false;
    }
    if (this->local_dir != other.local_dir) {
      return false;
    }
    if (this->repo_type != other.repo_type) {
      return false;
    }
    if (this->author != other.author) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlHfServer_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlHfServer_Request_

// alias to use template instance with default allocator
using ControlHfServer_Request =
  physical_ai_interfaces::srv::ControlHfServer_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Response __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Response __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlHfServer_Response_
{
  using Type = ControlHfServer_Response_<ContainerAllocator>;

  explicit ControlHfServer_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ControlHfServer_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Response
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Response
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlHfServer_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlHfServer_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlHfServer_Response_

// alias to use template instance with default allocator
using ControlHfServer_Response =
  physical_ai_interfaces::srv::ControlHfServer_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Event __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Event __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlHfServer_Event_
{
  using Type = ControlHfServer_Event_<ContainerAllocator>;

  explicit ControlHfServer_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ControlHfServer_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::ControlHfServer_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::ControlHfServer_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Event
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__ControlHfServer_Event
    std::shared_ptr<physical_ai_interfaces::srv::ControlHfServer_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlHfServer_Event_ & other) const
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
  bool operator!=(const ControlHfServer_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlHfServer_Event_

// alias to use template instance with default allocator
using ControlHfServer_Event =
  physical_ai_interfaces::srv::ControlHfServer_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces

namespace physical_ai_interfaces
{

namespace srv
{

struct ControlHfServer
{
  using Request = physical_ai_interfaces::srv::ControlHfServer_Request;
  using Response = physical_ai_interfaces::srv::ControlHfServer_Response;
  using Event = physical_ai_interfaces::srv::ControlHfServer_Event;
};

}  // namespace srv

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__STRUCT_HPP_
