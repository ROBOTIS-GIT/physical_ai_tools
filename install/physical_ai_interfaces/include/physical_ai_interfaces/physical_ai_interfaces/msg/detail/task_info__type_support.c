// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from physical_ai_interfaces:msg/TaskInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "physical_ai_interfaces/msg/detail/task_info__rosidl_typesupport_introspection_c.h"
#include "physical_ai_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "physical_ai_interfaces/msg/detail/task_info__functions.h"
#include "physical_ai_interfaces/msg/detail/task_info__struct.h"


// Include directives for member types
// Member `task_name`
// Member `task_type`
// Member `user_id`
// Member `task_instruction`
// Member `policy_path`
// Member `tags`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  physical_ai_interfaces__msg__TaskInfo__init(message_memory);
}

void physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_fini_function(void * message_memory)
{
  physical_ai_interfaces__msg__TaskInfo__fini(message_memory);
}

size_t physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__size_function__TaskInfo__task_instruction(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_const_function__TaskInfo__task_instruction(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_function__TaskInfo__task_instruction(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__fetch_function__TaskInfo__task_instruction(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_const_function__TaskInfo__task_instruction(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__assign_function__TaskInfo__task_instruction(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_function__TaskInfo__task_instruction(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__resize_function__TaskInfo__task_instruction(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__size_function__TaskInfo__tags(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_const_function__TaskInfo__tags(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_function__TaskInfo__tags(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__fetch_function__TaskInfo__tags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_const_function__TaskInfo__tags(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__assign_function__TaskInfo__tags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_function__TaskInfo__tags(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__resize_function__TaskInfo__tags(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_message_member_array[15] = {
  {
    "task_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, task_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, task_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "user_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, user_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_instruction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, task_instruction),  // bytes offset in struct
    NULL,  // default value
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__size_function__TaskInfo__task_instruction,  // size() function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_const_function__TaskInfo__task_instruction,  // get_const(index) function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_function__TaskInfo__task_instruction,  // get(index) function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__fetch_function__TaskInfo__task_instruction,  // fetch(index, &value) function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__assign_function__TaskInfo__task_instruction,  // assign(index, value) function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__resize_function__TaskInfo__task_instruction  // resize(index) function pointer
  },
  {
    "policy_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, policy_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, fps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, tags),  // bytes offset in struct
    NULL,  // default value
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__size_function__TaskInfo__tags,  // size() function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_const_function__TaskInfo__tags,  // get_const(index) function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__get_function__TaskInfo__tags,  // get(index) function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__fetch_function__TaskInfo__tags,  // fetch(index, &value) function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__assign_function__TaskInfo__tags,  // assign(index, value) function pointer
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__resize_function__TaskInfo__tags  // resize(index) function pointer
  },
  {
    "warmup_time_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, warmup_time_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "episode_time_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, episode_time_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reset_time_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, reset_time_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_episodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, num_episodes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "push_to_hub",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, push_to_hub),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "private_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, private_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "use_optimized_save_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, use_optimized_save_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "record_inference_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__msg__TaskInfo, record_inference_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_message_members = {
  "physical_ai_interfaces__msg",  // message namespace
  "TaskInfo",  // message name
  15,  // number of fields
  sizeof(physical_ai_interfaces__msg__TaskInfo),
  false,  // has_any_key_member_
  physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_message_member_array,  // message members
  physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_message_type_support_handle = {
  0,
  &physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_message_members,
  get_message_typesupport_handle_function,
  &physical_ai_interfaces__msg__TaskInfo__get_type_hash,
  &physical_ai_interfaces__msg__TaskInfo__get_type_description,
  &physical_ai_interfaces__msg__TaskInfo__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_physical_ai_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, msg, TaskInfo)() {
  if (!physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_message_type_support_handle.typesupport_identifier) {
    physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &physical_ai_interfaces__msg__TaskInfo__rosidl_typesupport_introspection_c__TaskInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
