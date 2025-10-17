// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:msg/HFOperationStatus.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/msg/detail/hf_operation_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__msg__HFOperationStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc6, 0x82, 0xe9, 0x9b, 0x53, 0x72, 0x16, 0x15,
      0x7f, 0x90, 0x9a, 0xd0, 0xf4, 0x77, 0x96, 0x63,
      0xb8, 0x99, 0x6a, 0x8b, 0x39, 0x8c, 0x1d, 0x7d,
      0x26, 0xc8, 0xd5, 0x1f, 0xda, 0x3a, 0xd7, 0xb1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char physical_ai_interfaces__msg__HFOperationStatus__TYPE_NAME[] = "physical_ai_interfaces/msg/HFOperationStatus";

// Define type names, field names, and default values
static char physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__operation[] = "operation";
static char physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__status[] = "status";
static char physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__local_path[] = "local_path";
static char physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__repo_id[] = "repo_id";
static char physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__message[] = "message";
static char physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__progress_current[] = "progress_current";
static char physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__progress_total[] = "progress_total";
static char physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__progress_percentage[] = "progress_percentage";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__msg__HFOperationStatus__FIELDS[] = {
  {
    {physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__operation, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__local_path, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__repo_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__progress_current, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__progress_total, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__HFOperationStatus__FIELD_NAME__progress_percentage, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__msg__HFOperationStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__msg__HFOperationStatus__TYPE_NAME, 44, 44},
      {physical_ai_interfaces__msg__HFOperationStatus__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string operation              # upload, download\n"
  "string status                 # Idle, Uploading, Downloading, Success, Failed\n"
  "string local_path             # local path of the dataset or model\n"
  "string repo_id                # repo id of the dataset or model\n"
  "string message                # message of the operation\n"
  "uint16 progress_current       # current progress\n"
  "uint16 progress_total         # total progress\n"
  "float32 progress_percentage   # percentage of the progress";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__msg__HFOperationStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__msg__HFOperationStatus__TYPE_NAME, 44, 44},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 470, 470},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__msg__HFOperationStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__msg__HFOperationStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
