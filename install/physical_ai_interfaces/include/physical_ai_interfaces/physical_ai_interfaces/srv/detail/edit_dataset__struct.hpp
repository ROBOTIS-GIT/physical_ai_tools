// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from physical_ai_interfaces:srv/EditDataset.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/edit_dataset.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__STRUCT_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__EditDataset_Request __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__EditDataset_Request __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EditDataset_Request_
{
  using Type = EditDataset_Request_<ContainerAllocator>;

  explicit EditDataset_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->delete_dataset_path = "";
      this->output_path = "";
      this->upload_huggingface = false;
    }
  }

  explicit EditDataset_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : delete_dataset_path(_alloc),
    output_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->delete_dataset_path = "";
      this->output_path = "";
      this->upload_huggingface = false;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _merge_dataset_list_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _merge_dataset_list_type merge_dataset_list;
  using _delete_dataset_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _delete_dataset_path_type delete_dataset_path;
  using _output_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _output_path_type output_path;
  using _delete_episode_num_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _delete_episode_num_type delete_episode_num;
  using _upload_huggingface_type =
    bool;
  _upload_huggingface_type upload_huggingface;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__merge_dataset_list(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->merge_dataset_list = _arg;
    return *this;
  }
  Type & set__delete_dataset_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->delete_dataset_path = _arg;
    return *this;
  }
  Type & set__output_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->output_path = _arg;
    return *this;
  }
  Type & set__delete_episode_num(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->delete_episode_num = _arg;
    return *this;
  }
  Type & set__upload_huggingface(
    const bool & _arg)
  {
    this->upload_huggingface = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MERGE =
    0u;
  // guard against 'DELETE' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(DELETE)
#    pragma push_macro("DELETE")
#    undef DELETE
#  endif
#endif
  static constexpr uint8_t DELETE =
    1u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("DELETE")
#endif

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__EditDataset_Request
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__EditDataset_Request
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EditDataset_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->merge_dataset_list != other.merge_dataset_list) {
      return false;
    }
    if (this->delete_dataset_path != other.delete_dataset_path) {
      return false;
    }
    if (this->output_path != other.output_path) {
      return false;
    }
    if (this->delete_episode_num != other.delete_episode_num) {
      return false;
    }
    if (this->upload_huggingface != other.upload_huggingface) {
      return false;
    }
    return true;
  }
  bool operator!=(const EditDataset_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EditDataset_Request_

// alias to use template instance with default allocator
using EditDataset_Request =
  physical_ai_interfaces::srv::EditDataset_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EditDataset_Request_<ContainerAllocator>::MERGE;
#endif  // __cplusplus < 201703L
// guard against 'DELETE' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(DELETE)
#    pragma push_macro("DELETE")
#    undef DELETE
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EditDataset_Request_<ContainerAllocator>::DELETE;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("DELETE")
#endif

}  // namespace srv

}  // namespace physical_ai_interfaces


#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__EditDataset_Response __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__EditDataset_Response __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EditDataset_Response_
{
  using Type = EditDataset_Response_<ContainerAllocator>;

  explicit EditDataset_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit EditDataset_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__EditDataset_Response
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__EditDataset_Response
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EditDataset_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const EditDataset_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EditDataset_Response_

// alias to use template instance with default allocator
using EditDataset_Response =
  physical_ai_interfaces::srv::EditDataset_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__physical_ai_interfaces__srv__EditDataset_Event __attribute__((deprecated))
#else
# define DEPRECATED__physical_ai_interfaces__srv__EditDataset_Event __declspec(deprecated)
#endif

namespace physical_ai_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EditDataset_Event_
{
  using Type = EditDataset_Event_<ContainerAllocator>;

  explicit EditDataset_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit EditDataset_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::EditDataset_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<physical_ai_interfaces::srv::EditDataset_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__physical_ai_interfaces__srv__EditDataset_Event
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__physical_ai_interfaces__srv__EditDataset_Event
    std::shared_ptr<physical_ai_interfaces::srv::EditDataset_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EditDataset_Event_ & other) const
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
  bool operator!=(const EditDataset_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EditDataset_Event_

// alias to use template instance with default allocator
using EditDataset_Event =
  physical_ai_interfaces::srv::EditDataset_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace physical_ai_interfaces

namespace physical_ai_interfaces
{

namespace srv
{

struct EditDataset
{
  using Request = physical_ai_interfaces::srv::EditDataset_Request;
  using Response = physical_ai_interfaces::srv::EditDataset_Response;
  using Event = physical_ai_interfaces::srv::EditDataset_Event;
};

}  // namespace srv

}  // namespace physical_ai_interfaces

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__STRUCT_HPP_
