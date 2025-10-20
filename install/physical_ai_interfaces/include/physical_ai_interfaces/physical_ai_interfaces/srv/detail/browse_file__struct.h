// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:srv/BrowseFile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/browse_file.h"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'action'
// Member 'current_path'
// Member 'target_name'
// Member 'target_files'
// Member 'target_folders'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/BrowseFile in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__BrowseFile_Request
{
  /// Request
  /// Action to perform: "browse", "get_path", "go_parent"
  rosidl_runtime_c__String action;
  /// Current path (used depending on action)
  rosidl_runtime_c__String current_path;
  /// Name of file/folder to select (for "browse" action)
  rosidl_runtime_c__String target_name;
  /// Optional: Files to check for in subdirectories (parallel search)
  rosidl_runtime_c__String__Sequence target_files;
  /// Optional: Folders to check for in subdirectories (parallel search)
  rosidl_runtime_c__String__Sequence target_folders;
} physical_ai_interfaces__srv__BrowseFile_Request;

// Struct for a sequence of physical_ai_interfaces__srv__BrowseFile_Request.
typedef struct physical_ai_interfaces__srv__BrowseFile_Request__Sequence
{
  physical_ai_interfaces__srv__BrowseFile_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__BrowseFile_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'current_path'
// Member 'parent_path'
// Member 'selected_path'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'items'
#include "physical_ai_interfaces/msg/detail/browser_item__struct.h"

/// Struct defined in srv/BrowseFile in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__BrowseFile_Response
{
  /// Whether the operation was successful
  bool success;
  /// Status or error message
  rosidl_runtime_c__String message;
  /// Current path
  rosidl_runtime_c__String current_path;
  /// Parent directory path
  rosidl_runtime_c__String parent_path;
  /// Full path of the selected item
  rosidl_runtime_c__String selected_path;
  /// Contents of the current directory
  physical_ai_interfaces__msg__BrowserItem__Sequence items;
} physical_ai_interfaces__srv__BrowseFile_Response;

// Struct for a sequence of physical_ai_interfaces__srv__BrowseFile_Response.
typedef struct physical_ai_interfaces__srv__BrowseFile_Response__Sequence
{
  physical_ai_interfaces__srv__BrowseFile_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__BrowseFile_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  physical_ai_interfaces__srv__BrowseFile_Event__request__MAX_SIZE = 1
};
// response
enum
{
  physical_ai_interfaces__srv__BrowseFile_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/BrowseFile in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__srv__BrowseFile_Event
{
  service_msgs__msg__ServiceEventInfo info;
  physical_ai_interfaces__srv__BrowseFile_Request__Sequence request;
  physical_ai_interfaces__srv__BrowseFile_Response__Sequence response;
} physical_ai_interfaces__srv__BrowseFile_Event;

// Struct for a sequence of physical_ai_interfaces__srv__BrowseFile_Event.
typedef struct physical_ai_interfaces__srv__BrowseFile_Event__Sequence
{
  physical_ai_interfaces__srv__BrowseFile_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__srv__BrowseFile_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__STRUCT_H_
