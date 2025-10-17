// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from physical_ai_interfaces:msg/BrowserItem.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "physical_ai_interfaces/msg/detail/browser_item__rosidl_typesupport_introspection_c.h"
#include "physical_ai_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "physical_ai_interfaces/msg/detail/browser_item__functions.h"
#include "physical_ai_interfaces/msg/detail/browser_item__struct.h"


// Include directives for member types
// Member `name`
// Member `full_path`
// Member `modified_time`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  physical_ai_interfaces__msg__BrowserItem__init(message_memory);
}

void physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_fini_function(void * message_memory)
{
  physical_ai_interfaces__msg__BrowserItem__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_message_member_array[6] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__BrowserItem, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "full_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__BrowserItem, full_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_directory",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__BrowserItem, is_directory),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__BrowserItem, size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "modified_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__BrowserItem, modified_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_target_file",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__BrowserItem, has_target_file),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_message_members = {
  "physical_ai_interfaces__msg",  // message namespace
  "BrowserItem",  // message name
  6,  // number of fields
  sizeof(physical_ai_interfaces__msg__BrowserItem),
  false,  // has_any_key_member_
  physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_message_member_array,  // message members
  physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_init_function,  // function to initialize message memory (memory has to be allocated)
  physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_message_type_support_handle = {
  0,
  &physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_message_members,
  get_message_typesupport_handle_function,
  &physical_ai_interfaces__msg__BrowserItem__get_type_hash,
  &physical_ai_interfaces__msg__BrowserItem__get_type_description,
  &physical_ai_interfaces__msg__BrowserItem__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_physical_ai_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, msg, BrowserItem)() {
  if (!physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_message_type_support_handle.typesupport_identifier) {
    physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &physical_ai_interfaces__msg__BrowserItem__rosidl_typesupport_introspection_c__BrowserItem_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
