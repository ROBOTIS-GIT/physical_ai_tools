// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from physical_ai_interfaces:srv/GetModelWeightList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "physical_ai_interfaces/srv/detail/get_model_weight_list__rosidl_typesupport_introspection_c.h"
#include "physical_ai_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "physical_ai_interfaces/srv/detail/get_model_weight_list__functions.h"
#include "physical_ai_interfaces/srv/detail/get_model_weight_list__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  physical_ai_interfaces__srv__GetModelWeightList_Request__init(message_memory);
}

void physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_fini_function(void * message_memory)
{
  physical_ai_interfaces__srv__GetModelWeightList_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__srv__GetModelWeightList_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_members = {
  "physical_ai_interfaces__srv",  // message namespace
  "GetModelWeightList_Request",  // message name
  1,  // number of fields
  sizeof(physical_ai_interfaces__srv__GetModelWeightList_Request),
  false,  // has_any_key_member_
  physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_member_array,  // message members
  physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_type_support_handle = {
  0,
  &physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_members,
  get_message_typesupport_handle_function,
  &physical_ai_interfaces__srv__GetModelWeightList_Request__get_type_hash,
  &physical_ai_interfaces__srv__GetModelWeightList_Request__get_type_description,
  &physical_ai_interfaces__srv__GetModelWeightList_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_physical_ai_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Request)() {
  if (!physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_type_support_handle.typesupport_identifier) {
    physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "physical_ai_interfaces/srv/detail/get_model_weight_list__rosidl_typesupport_introspection_c.h"
// already included above
// #include "physical_ai_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "physical_ai_interfaces/srv/detail/get_model_weight_list__functions.h"
// already included above
// #include "physical_ai_interfaces/srv/detail/get_model_weight_list__struct.h"


// Include directives for member types
// Member `model_weight_list`
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  physical_ai_interfaces__srv__GetModelWeightList_Response__init(message_memory);
}

void physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_fini_function(void * message_memory)
{
  physical_ai_interfaces__srv__GetModelWeightList_Response__fini(message_memory);
}

size_t physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__size_function__GetModelWeightList_Response__model_weight_list(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Response__model_weight_list(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Response__model_weight_list(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__fetch_function__GetModelWeightList_Response__model_weight_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Response__model_weight_list(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__assign_function__GetModelWeightList_Response__model_weight_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Response__model_weight_list(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__resize_function__GetModelWeightList_Response__model_weight_list(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_member_array[3] = {
  {
    "model_weight_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__srv__GetModelWeightList_Response, model_weight_list),  // bytes offset in struct
    NULL,  // default value
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__size_function__GetModelWeightList_Response__model_weight_list,  // size() function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Response__model_weight_list,  // get_const(index) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Response__model_weight_list,  // get(index) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__fetch_function__GetModelWeightList_Response__model_weight_list,  // fetch(index, &value) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__assign_function__GetModelWeightList_Response__model_weight_list,  // assign(index, value) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__resize_function__GetModelWeightList_Response__model_weight_list  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__srv__GetModelWeightList_Response, success),  // bytes offset in struct
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
    offsetof(physical_ai_interfaces__srv__GetModelWeightList_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_members = {
  "physical_ai_interfaces__srv",  // message namespace
  "GetModelWeightList_Response",  // message name
  3,  // number of fields
  sizeof(physical_ai_interfaces__srv__GetModelWeightList_Response),
  false,  // has_any_key_member_
  physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_member_array,  // message members
  physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_type_support_handle = {
  0,
  &physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_members,
  get_message_typesupport_handle_function,
  &physical_ai_interfaces__srv__GetModelWeightList_Response__get_type_hash,
  &physical_ai_interfaces__srv__GetModelWeightList_Response__get_type_description,
  &physical_ai_interfaces__srv__GetModelWeightList_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_physical_ai_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Response)() {
  if (!physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_type_support_handle.typesupport_identifier) {
    physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "physical_ai_interfaces/srv/detail/get_model_weight_list__rosidl_typesupport_introspection_c.h"
// already included above
// #include "physical_ai_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "physical_ai_interfaces/srv/detail/get_model_weight_list__functions.h"
// already included above
// #include "physical_ai_interfaces/srv/detail/get_model_weight_list__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "physical_ai_interfaces/srv/get_model_weight_list.h"
// Member `request`
// Member `response`
// already included above
// #include "physical_ai_interfaces/srv/detail/get_model_weight_list__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  physical_ai_interfaces__srv__GetModelWeightList_Event__init(message_memory);
}

void physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_fini_function(void * message_memory)
{
  physical_ai_interfaces__srv__GetModelWeightList_Event__fini(message_memory);
}

size_t physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__size_function__GetModelWeightList_Event__request(
  const void * untyped_member)
{
  const physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence * member =
    (const physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Event__request(
  const void * untyped_member, size_t index)
{
  const physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence * member =
    (const physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Event__request(
  void * untyped_member, size_t index)
{
  physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence * member =
    (physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__fetch_function__GetModelWeightList_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const physical_ai_interfaces__srv__GetModelWeightList_Request * item =
    ((const physical_ai_interfaces__srv__GetModelWeightList_Request *)
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Event__request(untyped_member, index));
  physical_ai_interfaces__srv__GetModelWeightList_Request * value =
    (physical_ai_interfaces__srv__GetModelWeightList_Request *)(untyped_value);
  *value = *item;
}

void physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__assign_function__GetModelWeightList_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  physical_ai_interfaces__srv__GetModelWeightList_Request * item =
    ((physical_ai_interfaces__srv__GetModelWeightList_Request *)
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Event__request(untyped_member, index));
  const physical_ai_interfaces__srv__GetModelWeightList_Request * value =
    (const physical_ai_interfaces__srv__GetModelWeightList_Request *)(untyped_value);
  *item = *value;
}

bool physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__resize_function__GetModelWeightList_Event__request(
  void * untyped_member, size_t size)
{
  physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence * member =
    (physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence *)(untyped_member);
  physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence__fini(member);
  return physical_ai_interfaces__srv__GetModelWeightList_Request__Sequence__init(member, size);
}

size_t physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__size_function__GetModelWeightList_Event__response(
  const void * untyped_member)
{
  const physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence * member =
    (const physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Event__response(
  const void * untyped_member, size_t index)
{
  const physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence * member =
    (const physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Event__response(
  void * untyped_member, size_t index)
{
  physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence * member =
    (physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__fetch_function__GetModelWeightList_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const physical_ai_interfaces__srv__GetModelWeightList_Response * item =
    ((const physical_ai_interfaces__srv__GetModelWeightList_Response *)
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Event__response(untyped_member, index));
  physical_ai_interfaces__srv__GetModelWeightList_Response * value =
    (physical_ai_interfaces__srv__GetModelWeightList_Response *)(untyped_value);
  *value = *item;
}

void physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__assign_function__GetModelWeightList_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  physical_ai_interfaces__srv__GetModelWeightList_Response * item =
    ((physical_ai_interfaces__srv__GetModelWeightList_Response *)
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Event__response(untyped_member, index));
  const physical_ai_interfaces__srv__GetModelWeightList_Response * value =
    (const physical_ai_interfaces__srv__GetModelWeightList_Response *)(untyped_value);
  *item = *value;
}

bool physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__resize_function__GetModelWeightList_Event__response(
  void * untyped_member, size_t size)
{
  physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence * member =
    (physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence *)(untyped_member);
  physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence__fini(member);
  return physical_ai_interfaces__srv__GetModelWeightList_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(physical_ai_interfaces__srv__GetModelWeightList_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(physical_ai_interfaces__srv__GetModelWeightList_Event, request),  // bytes offset in struct
    NULL,  // default value
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__size_function__GetModelWeightList_Event__request,  // size() function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Event__request,  // get_const(index) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Event__request,  // get(index) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__fetch_function__GetModelWeightList_Event__request,  // fetch(index, &value) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__assign_function__GetModelWeightList_Event__request,  // assign(index, value) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__resize_function__GetModelWeightList_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(physical_ai_interfaces__srv__GetModelWeightList_Event, response),  // bytes offset in struct
    NULL,  // default value
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__size_function__GetModelWeightList_Event__response,  // size() function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_const_function__GetModelWeightList_Event__response,  // get_const(index) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__get_function__GetModelWeightList_Event__response,  // get(index) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__fetch_function__GetModelWeightList_Event__response,  // fetch(index, &value) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__assign_function__GetModelWeightList_Event__response,  // assign(index, value) function pointer
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__resize_function__GetModelWeightList_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_members = {
  "physical_ai_interfaces__srv",  // message namespace
  "GetModelWeightList_Event",  // message name
  3,  // number of fields
  sizeof(physical_ai_interfaces__srv__GetModelWeightList_Event),
  false,  // has_any_key_member_
  physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_member_array,  // message members
  physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_type_support_handle = {
  0,
  &physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_members,
  get_message_typesupport_handle_function,
  &physical_ai_interfaces__srv__GetModelWeightList_Event__get_type_hash,
  &physical_ai_interfaces__srv__GetModelWeightList_Event__get_type_description,
  &physical_ai_interfaces__srv__GetModelWeightList_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_physical_ai_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Event)() {
  physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Request)();
  physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Response)();
  if (!physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_type_support_handle.typesupport_identifier) {
    physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "physical_ai_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "physical_ai_interfaces/srv/detail/get_model_weight_list__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_service_members = {
  "physical_ai_interfaces__srv",  // service namespace
  "GetModelWeightList",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_type_support_handle,
  NULL,  // response message
  // physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_type_support_handle
  NULL  // event_message
  // physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_type_support_handle
};


static rosidl_service_type_support_t physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_service_type_support_handle = {
  0,
  &physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_service_members,
  get_service_typesupport_handle_function,
  &physical_ai_interfaces__srv__GetModelWeightList_Request__rosidl_typesupport_introspection_c__GetModelWeightList_Request_message_type_support_handle,
  &physical_ai_interfaces__srv__GetModelWeightList_Response__rosidl_typesupport_introspection_c__GetModelWeightList_Response_message_type_support_handle,
  &physical_ai_interfaces__srv__GetModelWeightList_Event__rosidl_typesupport_introspection_c__GetModelWeightList_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    physical_ai_interfaces,
    srv,
    GetModelWeightList
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    physical_ai_interfaces,
    srv,
    GetModelWeightList
  ),
  &physical_ai_interfaces__srv__GetModelWeightList__get_type_hash,
  &physical_ai_interfaces__srv__GetModelWeightList__get_type_description,
  &physical_ai_interfaces__srv__GetModelWeightList__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_physical_ai_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList)(void) {
  if (!physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_service_type_support_handle.typesupport_identifier) {
    physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, physical_ai_interfaces, srv, GetModelWeightList_Event)()->data;
  }

  return &physical_ai_interfaces__srv__detail__get_model_weight_list__rosidl_typesupport_introspection_c__GetModelWeightList_service_type_support_handle;
}
