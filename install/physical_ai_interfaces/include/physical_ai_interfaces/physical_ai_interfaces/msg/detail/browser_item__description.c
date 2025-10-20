// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:msg/BrowserItem.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/msg/detail/browser_item__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__msg__BrowserItem__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdc, 0xeb, 0xf7, 0x22, 0x45, 0xcd, 0x42, 0x0b,
      0x72, 0x44, 0x5a, 0x0f, 0x73, 0x68, 0xbb, 0xc8,
      0x46, 0x9d, 0x80, 0x36, 0x77, 0x91, 0xb4, 0xe7,
      0x8e, 0xd4, 0xbc, 0x0d, 0x68, 0x36, 0x92, 0x18,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char physical_ai_interfaces__msg__BrowserItem__TYPE_NAME[] = "physical_ai_interfaces/msg/BrowserItem";

// Define type names, field names, and default values
static char physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__name[] = "name";
static char physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__full_path[] = "full_path";
static char physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__is_directory[] = "is_directory";
static char physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__size[] = "size";
static char physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__modified_time[] = "modified_time";
static char physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__has_target_file[] = "has_target_file";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__msg__BrowserItem__FIELDS[] = {
  {
    {physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__full_path, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__is_directory, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__size, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__modified_time, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__BrowserItem__FIELD_NAME__has_target_file, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__msg__BrowserItem__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__msg__BrowserItem__TYPE_NAME, 38, 38},
      {physical_ai_interfaces__msg__BrowserItem__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string name           # File or folder name\n"
  "string full_path      # Full path\n"
  "bool is_directory     # Whether it is a directory\n"
  "int64 size            # File size (bytes, -1 for directories)\n"
  "string modified_time  # Last modified time (as string)\n"
  "bool has_target_file  # Whether directory contains any target files (only for directories)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__msg__BrowserItem__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__msg__BrowserItem__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 336, 336},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__msg__BrowserItem__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__msg__BrowserItem__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
