// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:msg/TrainingInfo.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/msg/detail/training_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__msg__TrainingInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaf, 0x10, 0xd5, 0xc8, 0x89, 0x77, 0xa3, 0x8f,
      0xe6, 0x20, 0xd6, 0x46, 0xb1, 0x19, 0x65, 0xa6,
      0x13, 0x3f, 0xc6, 0xb3, 0xac, 0xe8, 0xfa, 0x95,
      0x76, 0x4d, 0xaf, 0x30, 0x63, 0x9c, 0xec, 0x31,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char physical_ai_interfaces__msg__TrainingInfo__TYPE_NAME[] = "physical_ai_interfaces/msg/TrainingInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__dataset[] = "dataset";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__policy_type[] = "policy_type";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__output_folder_name[] = "output_folder_name";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__policy_device[] = "policy_device";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__seed[] = "seed";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__num_workers[] = "num_workers";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__batch_size[] = "batch_size";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__steps[] = "steps";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__eval_freq[] = "eval_freq";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__log_freq[] = "log_freq";
static char physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__save_freq[] = "save_freq";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__msg__TrainingInfo__FIELDS[] = {
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__dataset, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__policy_type, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__output_folder_name, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__policy_device, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__seed, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__num_workers, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__batch_size, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__steps, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__eval_freq, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__log_freq, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingInfo__FIELD_NAME__save_freq, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__msg__TrainingInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__msg__TrainingInfo__TYPE_NAME, 39, 39},
      {physical_ai_interfaces__msg__TrainingInfo__FIELDS, 11, 11},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Must be given\n"
  "string dataset  # ROBOTIS/ffw_bg2_example\n"
  "string policy_type\n"
  "string output_folder_name\n"
  "string policy_device\n"
  "\n"
  "# Options\n"
  "uint32 seed\n"
  "uint8 num_workers\n"
  "uint16 batch_size\n"
  "uint32 steps\n"
  "uint32 eval_freq\n"
  "uint32 log_freq\n"
  "uint32 save_freq";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__msg__TrainingInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__msg__TrainingInfo__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 246, 246},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__msg__TrainingInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__msg__TrainingInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
