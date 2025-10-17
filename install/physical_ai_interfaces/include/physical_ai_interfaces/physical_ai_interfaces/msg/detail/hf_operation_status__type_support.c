// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from physical_ai_interfaces:msg/HFOperationStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "physical_ai_interfaces/msg/detail/hf_operation_status__rosidl_typesupport_introspection_c.h"
#include "physical_ai_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "physical_ai_interfaces/msg/detail/hf_operation_status__functions.h"
#include "physical_ai_interfaces/msg/detail/hf_operation_status__struct.h"


// Include directives for member types
// Member `operation`
// Member `status`
// Member `local_path`
// Member `repo_id`
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  physical_ai_interfaces__msg__HFOperationStatus__init(message_memory);
}

void physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_fini_function(void * message_memory)
{
  physical_ai_interfaces__msg__HFOperationStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_message_member_array[8] = {
  {
    "operation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__HFOperationStatus, operation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__HFOperationStatus, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__HFOperationStatus, local_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "repo_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__HFOperationStatus, repo_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__HFOperationStatus, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "progress_current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__HFOperationStatus, progress_current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "progress_total",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__HFOperationStatus, progress_total),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "progress_percentage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__HFOperationStatus, progress_percentage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_message_members = {
  "physical_ai_interfaces__msg",  // message namespace
  "HFOperationStatus",  // message name
  8,  // number of fields
  sizeof(physical_ai_interfaces__msg__HFOperationStatus),
  false,  // has_any_key_member_
  physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_message_member_array,  // message members
  physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_message_type_support_handle = {
  0,
  &physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_message_members,
  get_message_typesupport_handle_function,
  &physical_ai_interfaces__msg__HFOperationStatus__get_type_hash,
  &physical_ai_interfaces__msg__HFOperationStatus__get_type_description,
  &physical_ai_interfaces__msg__HFOperationStatus__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_physical_ai_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, msg, HFOperationStatus)() {
  if (!physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_message_type_support_handle.typesupport_identifier) {
    physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &physical_ai_interfaces__msg__HFOperationStatus__rosidl_typesupport_introspection_c__HFOperationStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
