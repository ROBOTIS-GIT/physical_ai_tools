// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:msg/DatasetInfo.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/msg/detail/dataset_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__msg__DatasetInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x04, 0x17, 0x39, 0xe9, 0x3d, 0x2a, 0x32, 0x23,
      0xc8, 0x2b, 0x9f, 0x00, 0xfd, 0x05, 0x3b, 0xaf,
      0x5c, 0xbb, 0x35, 0x8a, 0x5e, 0xbb, 0xa3, 0x8f,
      0xde, 0x4b, 0xa1, 0xe3, 0x22, 0x88, 0x00, 0x94,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char physical_ai_interfaces__msg__DatasetInfo__TYPE_NAME[] = "physical_ai_interfaces/msg/DatasetInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__codebase_version[] = "codebase_version";
static char physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__robot_type[] = "robot_type";
static char physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__total_episodes[] = "total_episodes";
static char physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__total_tasks[] = "total_tasks";
static char physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__fps[] = "fps";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__msg__DatasetInfo__FIELDS[] = {
  {
    {physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__codebase_version, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__robot_type, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__total_episodes, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__total_tasks, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__DatasetInfo__FIELD_NAME__fps, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__msg__DatasetInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__msg__DatasetInfo__TYPE_NAME, 38, 38},
      {physical_ai_interfaces__msg__DatasetInfo__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string codebase_version\n"
  "string robot_type\n"
  "uint16 total_episodes\n"
  "uint16 total_tasks\n"
  "uint8 fps";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__msg__DatasetInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__msg__DatasetInfo__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 93, 93},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__msg__DatasetInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__msg__DatasetInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
