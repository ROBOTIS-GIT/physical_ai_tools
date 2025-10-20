// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:msg/TaskStatus.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/msg/detail/task_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__msg__TaskStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7d, 0xfa, 0xcc, 0x2a, 0x34, 0x49, 0x72, 0xad,
      0xb9, 0x56, 0xb7, 0x6e, 0xc7, 0x17, 0xb4, 0x14,
      0x33, 0xc4, 0x99, 0x0c, 0x08, 0x32, 0x08, 0x68,
      0xf0, 0x4f, 0x9d, 0xa1, 0x53, 0x3a, 0x18, 0x7d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "physical_ai_interfaces/msg/detail/task_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t physical_ai_interfaces__msg__TaskInfo__EXPECTED_HASH = {1, {
    0xad, 0x88, 0x99, 0x37, 0x0a, 0xc1, 0x4a, 0x0a,
    0xb7, 0xa3, 0xd5, 0x7c, 0xce, 0xfa, 0xa4, 0xae,
    0x18, 0xb3, 0x95, 0xdc, 0xf3, 0xea, 0x28, 0x70,
    0xab, 0xc4, 0x28, 0x37, 0xe7, 0x7e, 0xc9, 0xd9,
  }};
#endif

static char physical_ai_interfaces__msg__TaskStatus__TYPE_NAME[] = "physical_ai_interfaces/msg/TaskStatus";
static char physical_ai_interfaces__msg__TaskInfo__TYPE_NAME[] = "physical_ai_interfaces/msg/TaskInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__task_info[] = "task_info";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__robot_type[] = "robot_type";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__phase[] = "phase";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__total_time[] = "total_time";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__proceed_time[] = "proceed_time";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__current_episode_number[] = "current_episode_number";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__current_scenario_number[] = "current_scenario_number";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__current_task_instruction[] = "current_task_instruction";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__encoding_progress[] = "encoding_progress";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__used_storage_size[] = "used_storage_size";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__total_storage_size[] = "total_storage_size";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__used_cpu[] = "used_cpu";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__used_ram_size[] = "used_ram_size";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__total_ram_size[] = "total_ram_size";
static char physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__error[] = "error";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__msg__TaskStatus__FIELDS[] = {
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__task_info, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__msg__TaskInfo__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__robot_type, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__phase, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__total_time, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__proceed_time, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__current_episode_number, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__current_scenario_number, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__current_task_instruction, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__encoding_progress, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__used_storage_size, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__total_storage_size, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__used_cpu, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__used_ram_size, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__total_ram_size, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__TaskStatus__FIELD_NAME__error, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__msg__TaskStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {physical_ai_interfaces__msg__TaskInfo__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__msg__TaskStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__msg__TaskStatus__TYPE_NAME, 37, 37},
      {physical_ai_interfaces__msg__TaskStatus__FIELDS, 15, 15},
    },
    {physical_ai_interfaces__msg__TaskStatus__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&physical_ai_interfaces__msg__TaskInfo__EXPECTED_HASH, physical_ai_interfaces__msg__TaskInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = physical_ai_interfaces__msg__TaskInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "########################################\n"
  "# Constants\n"
  "########################################\n"
  "# phase\n"
  "uint8 READY = 0\n"
  "uint8 WARMING_UP = 1\n"
  "uint8 RESETTING = 2\n"
  "uint8 RECORDING = 3\n"
  "uint8 SAVING = 4\n"
  "uint8 STOPPED = 5\n"
  "uint8 INFERENCING = 6\n"
  "\n"
  "TaskInfo task_info\n"
  "string robot_type\n"
  "uint8 phase                     # (0: READY, 1: WARMING_UP, 2: RESETTING 3: RECORDING, 4: SAVING, 5: STOPPED, 6: INFERENCING)\n"
  "uint16 total_time               # [s]\n"
  "uint16 proceed_time             # [s]\n"
  "uint16 current_episode_number\n"
  "uint16 current_scenario_number\n"
  "string current_task_instruction\n"
  "float32 encoding_progress       # [%]\n"
  "float32 used_storage_size       # [GB]\n"
  "float32 total_storage_size      # [GB]\n"
  "float32 used_cpu                # [%]\n"
  "float32 used_ram_size           # [GB]\n"
  "float32 total_ram_size          # [GB]\n"
  "string error";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__msg__TaskStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__msg__TaskStatus__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 814, 814},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__msg__TaskStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__msg__TaskStatus__get_individual_type_description_source(NULL),
    sources[1] = *physical_ai_interfaces__msg__TaskInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
