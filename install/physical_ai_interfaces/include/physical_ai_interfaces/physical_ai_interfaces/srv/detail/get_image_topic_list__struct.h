// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:srv/GetImageTopicList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_image_topic_list.h"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_IMAGE_TOPIC_LIST__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_IMAGE_TOPIC_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetImageTopicList in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__GetImageTopicList_Request
{
  uint8_t structure_needs_at_least_one_member;
} physical_ai_interfaces__srv__GetImageTopicList_Request;

// Struct for a sequence of physical_ai_interfaces__srv__GetImageTopicList_Request.
typedef struct physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence
{
  physical_ai_interfaces__srv__GetImageTopicList_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'image_topic_list'
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetImageTopicList in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__GetImageTopicList_Response
{
  rosidl_runtime_c__String__Sequence image_topic_list;
  bool success;
  rosidl_runtime_c__String message;
} physical_ai_interfaces__srv__GetImageTopicList_Response;

// Struct for a sequence of physical_ai_interfaces__srv__GetImageTopicList_Response.
typedef struct physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence
{
  physical_ai_interfaces__srv__GetImageTopicList_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  physical_ai_interfaces__srv__GetImageTopicList_Event__request__MAX_SIZE = 1
};
// response
enum
{
  physical_ai_interfaces__srv__GetImageTopicList_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetImageTopicList in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__GetImageTopicList_Event
{
  service_msgs__msg__ServiceEventInfo info;
  physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence request;
  physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence response;
} physical_ai_interfaces__srv__GetImageTopicList_Event;

// Struct for a sequence of physical_ai_interfaces__srv__GetImageTopicList_Event.
typedef struct physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence
{
  physical_ai_interfaces__srv__GetImageTopicList_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_IMAGE_TOPIC_LIST__STRUCT_H_
