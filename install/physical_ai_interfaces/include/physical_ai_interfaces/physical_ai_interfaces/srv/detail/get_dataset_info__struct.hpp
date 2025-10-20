// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:srv/GetDatasetInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_dataset_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Request __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Request __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetDatasetInfo_Request_
{
  using Type = GetDatasetInfo_Request_<ContainerAllocator>;

  explicit GetDatasetInfo_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dataset_path = "";
    }
  }

  explicit GetDatasetInfo_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dataset_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dataset_path = "";
    }
  }

  // field types and members
  using _dataset_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dataset_path_type dataset_path;

  // setters for named parameter idiom
  Type & set__dataset_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dataset_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Request
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Request
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetDatasetInfo_Request_ & other) const
  {
    if (this->dataset_path != other.dataset_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetDatasetInfo_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetDatasetInfo_Request_

// alias to use template instance with default allocator
using GetDatasetInfo_Request =
  physical_ai_interfaces::srv::GetDatasetInfo_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces


// Include directives for member types
// Member 'dataset_info'
#include "physical_ai_interfaces/msg/detail/dataset_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Response __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Response __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetDatasetInfo_Response_
{
  using Type = GetDatasetInfo_Response_<ContainerAllocator>;

  explicit GetDatasetInfo_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dataset_info(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit GetDatasetInfo_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dataset_info(_alloc, _init),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _dataset_info_type =
    physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator>;
  _dataset_info_type dataset_info;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__dataset_info(
    const physical_ai_interfaces::msg::DatasetInfo_<ContainerAllocator> & _arg)
  {
    this->dataset_info = _arg;
    return *this;
  }
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
    physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Response
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Response
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetDatasetInfo_Response_ & other) const
  {
    if (this->dataset_info != other.dataset_info) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetDatasetInfo_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetDatasetInfo_Response_

// alias to use template instance with default allocator
using GetDatasetInfo_Response =
  physical_ai_interfaces::srv::GetDatasetInfo_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Event __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Event __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetDatasetInfo_Event_
{
  using Type = GetDatasetInfo_Event_<ContainerAllocator>;

  explicit GetDatasetInfo_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetDatasetInfo_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::GetDatasetInfo_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::GetDatasetInfo_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Event
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__GetDatasetInfo_Event
    std::shared_ptr<physical_ai_interfaces::srv::GetDatasetInfo_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetDatasetInfo_Event_ & other) const
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
  bool operator!=(const GetDatasetInfo_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetDatasetInfo_Event_

// alias to use template instance with default allocator
using GetDatasetInfo_Event =
  physical_ai_interfaces::srv::GetDatasetInfo_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces

namespace physical_ai_interfaces
{

namespace srv
{

struct GetDatasetInfo
{
  using Request = physical_ai_interfaces::srv::GetDatasetInfo_Request;
  using Response = physical_ai_interfaces::srv::GetDatasetInfo_Response;
  using Event = physical_ai_interfaces::srv::GetDatasetInfo_Event;
};

}  // namespace srv

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__STRUCT_HPP_
