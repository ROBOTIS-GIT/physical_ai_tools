// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:msg/BrowserItem.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/browser_item.h"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'full_path'
// Member 'modified_time'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BrowserItem in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__msg__BrowserItem
{
  /// File or folder name
  rosidl_runtime_c__String name;
  /// Full path
  rosidl_runtime_c__String full_path;
  /// Whether it is a directory
  bool is_directory;
  /// File size (bytes, -1 for directories)
  int64_t size;
  /// Last modified time (as string)
  rosidl_runtime_c__String modified_time;
  /// Whether directory contains any target files (only for directories)
  bool has_target_file;
} physical_ai_interfaces__msg__BrowserItem;

// Struct for a sequence of physical_ai_interfaces__msg__BrowserItem.
typedef struct physical_ai_interfaces__msg__BrowserItem__Sequence
{
  physical_ai_interfaces__msg__BrowserItem * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__msg__BrowserItem__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__STRUCT_H_
