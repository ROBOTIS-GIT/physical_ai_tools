// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:srv/GetDatasetInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_dataset_info.h"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'dataset_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetDatasetInfo in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__GetDatasetInfo_Request
{
  rosidl_runtime_c__String dataset_path;
} physical_ai_interfaces__srv__GetDatasetInfo_Request;

// Struct for a sequence of physical_ai_interfaces__srv__GetDatasetInfo_Request.
typedef struct physical_ai_interfaces__srv__GetDatasetInfo_Request__Sequence
{
  physical_ai_interfaces__srv__GetDatasetInfo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__GetDatasetInfo_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'dataset_info'
#include "physical_ai_interfaces/msg/detail/dataset_info__struct.h"
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetDatasetInfo in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__GetDatasetInfo_Response
{
  physical_ai_interfaces__msg__DatasetInfo dataset_info;
  bool success;
  rosidl_runtime_c__String message;
} physical_ai_interfaces__srv__GetDatasetInfo_Response;

// Struct for a sequence of physical_ai_interfaces__srv__GetDatasetInfo_Response.
typedef struct physical_ai_interfaces__srv__GetDatasetInfo_Response__Sequence
{
  physical_ai_interfaces__srv__GetDatasetInfo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__GetDatasetInfo_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  physical_ai_interfaces__srv__GetDatasetInfo_Event__request__MAX_SIZE = 1
};
// response
enum
{
  physical_ai_interfaces__srv__GetDatasetInfo_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetDatasetInfo in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__GetDatasetInfo_Event
{
  service_msgs__msg__ServiceEventInfo info;
  physical_ai_interfaces__srv__GetDatasetInfo_Request__Sequence request;
  physical_ai_interfaces__srv__GetDatasetInfo_Response__Sequence response;
} physical_ai_interfaces__srv__GetDatasetInfo_Event;

// Struct for a sequence of physical_ai_interfaces__srv__GetDatasetInfo_Event.
typedef struct physical_ai_interfaces__srv__GetDatasetInfo_Event__Sequence
{
  physical_ai_interfaces__srv__GetDatasetInfo_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__GetDatasetInfo_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_DATASET_INFO__STRUCT_H_
