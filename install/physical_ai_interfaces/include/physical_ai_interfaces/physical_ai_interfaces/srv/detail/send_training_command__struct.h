// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:srv/SendTrainingCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/send_training_command.h"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_TRAINING_COMMAND__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_TRAINING_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NONE'.
enum
{
  physical_ai_interfaces__srv__SendTrainingCommand_Request__NONE = 0
};

/// Constant 'START'.
enum
{
  physical_ai_interfaces__srv__SendTrainingCommand_Request__START = 1
};

/// Constant 'FINISH'.
enum
{
  physical_ai_interfaces__srv__SendTrainingCommand_Request__FINISH = 2
};

// Include directives for member types
// Member 'training_info'
#include "physical_ai_interfaces/msg/detail/training_info__struct.h"
// Member 'resume_model_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SendTrainingCommand in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SendTrainingCommand_Request
{
  uint8_t command;
  physical_ai_interfaces__msg__TrainingInfo training_info;
  bool resume;
  rosidl_runtime_c__String resume_model_path;
} physical_ai_interfaces__srv__SendTrainingCommand_Request;

// Struct for a sequence of physical_ai_interfaces__srv__SendTrainingCommand_Request.
typedef struct physical_ai_interfaces__srv__SendTrainingCommand_Request__Sequence
{
  physical_ai_interfaces__srv__SendTrainingCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SendTrainingCommand_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SendTrainingCommand in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SendTrainingCommand_Response
{
  bool success;
  rosidl_runtime_c__String message;
} physical_ai_interfaces__srv__SendTrainingCommand_Response;

// Struct for a sequence of physical_ai_interfaces__srv__SendTrainingCommand_Response.
typedef struct physical_ai_interfaces__srv__SendTrainingCommand_Response__Sequence
{
  physical_ai_interfaces__srv__SendTrainingCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SendTrainingCommand_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  physical_ai_interfaces__srv__SendTrainingCommand_Event__request__MAX_SIZE = 1
};
// response
enum
{
  physical_ai_interfaces__srv__SendTrainingCommand_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SendTrainingCommand in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__SendTrainingCommand_Event
{
  service_msgs__msg__ServiceEventInfo info;
  physical_ai_interfaces__srv__SendTrainingCommand_Request__Sequence request;
  physical_ai_interfaces__srv__SendTrainingCommand_Response__Sequence response;
} physical_ai_interfaces__srv__SendTrainingCommand_Event;

// Struct for a sequence of physical_ai_interfaces__srv__SendTrainingCommand_Event.
typedef struct physical_ai_interfaces__srv__SendTrainingCommand_Event__Sequence
{
  physical_ai_interfaces__srv__SendTrainingCommand_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__SendTrainingCommand_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__SEND_TRAINING_COMMAND__STRUCT_H_
