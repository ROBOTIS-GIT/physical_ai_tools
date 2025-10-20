// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/msg/detail/training_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__msg__TrainingStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3c, 0x35, 0x69, 0x8f, 0x4f, 0x47, 0xf5, 0xc4,
      0x64, 0x15, 0x74, 0x37, 0x99, 0x70, 0xdf, 0x7c,
      0x68, 0x00, 0x67, 0xd9, 0x6f, 0x9e, 0x12, 0xd8,
      0x0d, 0x9d, 0x93, 0x0a, 0x07, 0xd3, 0xe7, 0x40,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "physical_ai_interfaces/msg/detail/training_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t physical_ai_interfaces__msg__TrainingInfo__EXPECTED_HASH = {1, {
    0xaf, 0x10, 0xd5, 0xc8, 0x89, 0x77, 0xa3, 0x8f,
    0xe6, 0x20, 0xd6, 0x46, 0xb1, 0x19, 0x65, 0xa6,
    0x13, 0x3f, 0xc6, 0xb3, 0xac, 0xe8, 0xfa, 0x95,
    0x76, 0x4d, 0xaf, 0x30, 0x63, 0x9c, 0xec, 0x31,
  }};
#endif

static char physical_ai_interfaces__msg__TrainingStatus__TYPE_NAME[] = "physical_ai_interfaces/msg/TrainingStatus";
static char physical_ai_interfaces__msg__TrainingInfo__TYPE_NAME[] = "physical_ai_interfaces/msg/TrainingInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__training_info[] = "training_info";
static char physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__current_step[] = "current_step";
static char physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__current_loss[] = "current_loss";
static char physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__is_training[] = "is_training";
static char physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__error[] = "error";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__msg__TrainingStatus__FIELDS[] = {
  {
    {physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__training_info, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__msg__TrainingInfo__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__current_step, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__current_loss, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__is_training, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TrainingStatus__FIELD_NAME__error, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__msg__TrainingStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {physical_ai_interfaces__msg__TrainingInfo__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__msg__TrainingStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__msg__TrainingStatus__TYPE_NAME, 41, 41},
      {physical_ai_interfaces__msg__TrainingStatus__FIELDS, 5, 5},
    },
    {physical_ai_interfaces__msg__TrainingStatus__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&physical_ai_interfaces__msg__TrainingInfo__EXPECTED_HASH, physical_ai_interfaces__msg__TrainingInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = physical_ai_interfaces__msg__TrainingInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "########################################\n"
  "# Constants\n"
  "########################################\n"
  "\n"
  "TrainingInfo training_info\n"
  "uint32 current_step\n"
  "float32 current_loss\n"
  "bool is_training\n"
  "string error\n"
  "# diagnostic_msgs/KeyValue[]  metrics  # To be determined";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__msg__TrainingStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__msg__TrainingStatus__TYPE_NAME, 41, 41},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 251, 251},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__msg__TrainingStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__msg__TrainingStatus__get_individual_type_description_source(NULL),
    sources[1] = *physical_ai_interfaces__msg__TrainingInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
