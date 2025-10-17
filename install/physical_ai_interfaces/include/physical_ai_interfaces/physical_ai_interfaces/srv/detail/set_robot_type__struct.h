// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:srv/SetRobotType.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/set_robot_type.h"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__SET_ROBOT_TYPE__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__SET_ROBOT_TYPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetRobotType in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SetRobotType_Request
{
  rosidl_runtime_c__String robot_type;
} physical_ai_interfaces__srv__SetRobotType_Request;

// Struct for a sequence of physical_ai_interfaces__srv__SetRobotType_Request.
typedef struct physical_ai_interfaces__srv__SetRobotType_Request__Sequence
{
  physical_ai_interfaces__srv__SetRobotType_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SetRobotType_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetRobotType in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SetRobotType_Response
{
  bool success;
  rosidl_runtime_c__String message;
} physical_ai_interfaces__srv__SetRobotType_Response;

// Struct for a sequence of physical_ai_interfaces__srv__SetRobotType_Response.
typedef struct physical_ai_interfaces__srv__SetRobotType_Response__Sequence
{
  physical_ai_interfaces__srv__SetRobotType_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SetRobotType_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  physical_ai_interfaces__srv__SetRobotType_Event__request__MAX_SIZE = 1
};
// response
enum
{
  physical_ai_interfaces__srv__SetRobotType_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetRobotType in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SetRobotType_Event
{
  service_msgs__msg__ServiceEventInfo info;
  physical_ai_interfaces__srv__SetRobotType_Request__Sequence request;
  physical_ai_interfaces__srv__SetRobotType_Response__Sequence response;
} physical_ai_interfaces__srv__SetRobotType_Event;

// Struct for a sequence of physical_ai_interfaces__srv__SetRobotType_Event.
typedef struct physical_ai_interfaces__srv__SetRobotType_Event__Sequence
{
  physical_ai_interfaces__srv__SetRobotType_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SetRobotType_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__SET_ROBOT_TYPE__STRUCT_H_
