// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "physical_ai_interfaces/msg/detail/training_status__functions.h"
#include "physical_ai_interfaces/msg/detail/training_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace physical_ai_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TrainingStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) physical_ai_interfaces::msg::TrainingStatus(_init);
}

void TrainingStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<physical_ai_interfaces::msg::TrainingStatus *>(message_memory);
  typed_message->~TrainingStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TrainingStatus_message_member_array[5] = {
  {
    "training_info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<physical_ai_interfaces::msg::TrainingInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces::msg::TrainingStatus, training_info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "current_step",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces::msg::TrainingStatus, current_step),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "current_loss",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces::msg::TrainingStatus, current_loss),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_training",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces::msg::TrainingStatus, is_training),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces::msg::TrainingStatus, error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TrainingStatus_message_members = {
  "physical_ai_interfaces::msg",  // message namespace
  "TrainingStatus",  // message name
  5,  // number of fields
  sizeof(physical_ai_interfaces::msg::TrainingStatus),
  false,  // has_any_key_member_
  TrainingStatus_message_member_array,  // message members
  TrainingStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  TrainingStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TrainingStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TrainingStatus_message_members,
  get_message_typesupport_handle_function,
  &physical_ai_interfaces__msg__TrainingStatus__get_type_hash,
  &physical_ai_interfaces__msg__TrainingStatus__get_type_description,
  &physical_ai_interfaces__msg__TrainingStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace physical_ai_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<physical_ai_interfaces::msg::TrainingStatus>()
{
  return &::physical_ai_interfaces::msg::rosidl_typesupport_introspection_cpp::TrainingStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, physical_ai_interfaces, msg, TrainingStatus)() {
  return &::physical_ai_interfaces::msg::rosidl_typesupport_introspection_cpp::TrainingStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
