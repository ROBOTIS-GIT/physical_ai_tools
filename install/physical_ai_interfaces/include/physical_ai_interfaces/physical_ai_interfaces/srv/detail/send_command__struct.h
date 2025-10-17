// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:srv/SendCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/send_command.h"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IDLE'.
enum
{
  physical_ai_interfaces__srv__SendCommand_Request__IDLE = 0
};

/// Constant 'START_RECORD'.
enum
{
  physical_ai_interfaces__srv__SendCommand_Request__START_RECORD = 1
};

/// Constant 'START_INFERENCE'.
enum
{
  physical_ai_interfaces__srv__SendCommand_Request__START_INFERENCE = 2
};

/// Constant 'STOP'.
enum
{
  physical_ai_interfaces__srv__SendCommand_Request__STOP = 3
};

/// Constant 'MOVE_TO_NEXT'.
enum
{
  physical_ai_interfaces__srv__SendCommand_Request__MOVE_TO_NEXT = 4
};

/// Constant 'RERECORD'.
enum
{
  physical_ai_interfaces__srv__SendCommand_Request__RERECORD = 5
};

/// Constant 'FINISH'.
enum
{
  physical_ai_interfaces__srv__SendCommand_Request__FINISH = 6
};

/// Constant 'SKIP_TASK'.
enum
{
  physical_ai_interfaces__srv__SendCommand_Request__SKIP_TASK = 7
};

// Include directives for member types
// Member 'task_info'
#include "physical_ai_interfaces/msg/detail/task_info__struct.h"

/// Struct defined in srv/SendCommand in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SendCommand_Request
{
  uint8_t command;
  physical_ai_interfaces__msg__TaskInfo task_info;
} physical_ai_interfaces__srv__SendCommand_Request;

// Struct for a sequence of physical_ai_interfaces__srv__SendCommand_Request.
typedef struct physical_ai_interfaces__srv__SendCommand_Request__Sequence
{
  physical_ai_interfaces__srv__SendCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SendCommand_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SendCommand in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SendCommand_Response
{
  bool success;
  rosidl_runtime_c__String message;
} physical_ai_interfaces__srv__SendCommand_Response;

// Struct for a sequence of physical_ai_interfaces__srv__SendCommand_Response.
typedef struct physical_ai_interfaces__srv__SendCommand_Response__Sequence
{
  physical_ai_interfaces__srv__SendCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SendCommand_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  physical_ai_interfaces__srv__SendCommand_Event__request__MAX_SIZE = 1
};
// response
enum
{
  physical_ai_interfaces__srv__SendCommand_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SendCommand in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SendCommand_Event
{
  service_msgs__msg__ServiceEventInfo info;
  physical_ai_interfaces__srv__SendCommand_Request__Sequence request;
  physical_ai_interfaces__srv__SendCommand_Response__Sequence response;
} physical_ai_interfaces__srv__SendCommand_Event;

// Struct for a sequence of physical_ai_interfaces__srv__SendCommand_Event.
typedef struct physical_ai_interfaces__srv__SendCommand_Event__Sequence
{
  physical_ai_interfaces__srv__SendCommand_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SendCommand_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_COMMAND__STRUCT_H_
