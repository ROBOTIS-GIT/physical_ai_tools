// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:srv/EditDataset.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/edit_dataset.h"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MERGE'.
enum
{
  physical_ai_interfaces__srv__EditDataset_Request__MERGE = 0
};

/// Constant 'DELETE'.
enum
{
  physical_ai_interfaces__srv__EditDataset_Request__DELETE = 1
};

// Include directives for member types
// Member 'merge_dataset_list'
// Member 'delete_dataset_path'
// Member 'output_path'
#include "rosidl_runtime_c/string.h"
// Member 'delete_episode_num'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/EditDataset in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__EditDataset_Request
{
  uint8_t mode;
  rosidl_runtime_c__String__Sequence merge_dataset_list;
  rosidl_runtime_c__String delete_dataset_path;
  rosidl_runtime_c__String output_path;
  rosidl_runtime_c__uint16__Sequence delete_episode_num;
  bool upload_huggingface;
} physical_ai_interfaces__srv__EditDataset_Request;

// Struct for a sequence of physical_ai_interfaces__srv__EditDataset_Request.
typedef struct physical_ai_interfaces__srv__EditDataset_Request__Sequence
{
  physical_ai_interfaces__srv__EditDataset_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__EditDataset_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/EditDataset in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__EditDataset_Response
{
  bool success;
  rosidl_runtime_c__String message;
} physical_ai_interfaces__srv__EditDataset_Response;

// Struct for a sequence of physical_ai_interfaces__srv__EditDataset_Response.
typedef struct physical_ai_interfaces__srv__EditDataset_Response__Sequence
{
  physical_ai_interfaces__srv__EditDataset_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__EditDataset_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  physical_ai_interfaces__srv__EditDataset_Event__request__MAX_SIZE = 1
};
// response
enum
{
  physical_ai_interfaces__srv__EditDataset_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/EditDataset in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__EditDataset_Event
{
  service_msgs__msg__ServiceEventInfo info;
  physical_ai_interfaces__srv__EditDataset_Request__Sequence request;
  physical_ai_interfaces__srv__EditDataset_Response__Sequence response;
} physical_ai_interfaces__srv__EditDataset_Event;

// Struct for a sequence of physical_ai_interfaces__srv__EditDataset_Event.
typedef struct physical_ai_interfaces__srv__EditDataset_Event__Sequence
{
  physical_ai_interfaces__srv__EditDataset_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__EditDataset_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__STRUCT_H_
