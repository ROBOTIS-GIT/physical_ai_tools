// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:srv/EditDataset.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/srv/detail/edit_dataset__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__EditDataset__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x52, 0xad, 0xb7, 0xd7, 0xe0, 0x7a, 0xe4, 0x09,
      0xe3, 0x67, 0x3c, 0xb9, 0x18, 0x6f, 0xda, 0x68,
      0x6d, 0xdf, 0x7a, 0x22, 0x9d, 0x5a, 0xf6, 0xb8,
      0x26, 0x65, 0x8f, 0xee, 0x9e, 0x09, 0x32, 0x3e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__EditDataset_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x40, 0x2e, 0x27, 0xb7, 0x65, 0xe1, 0x9d, 0x6b,
      0x54, 0xc6, 0x04, 0xd1, 0xa5, 0x88, 0xbd, 0x5a,
      0x70, 0x17, 0x9b, 0xdb, 0x51, 0xb1, 0x94, 0x8e,
      0x44, 0x10, 0x41, 0xb8, 0xf4, 0xa2, 0x2d, 0x37,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__EditDataset_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x08, 0x69, 0xcc, 0xaa, 0x1d, 0xe1, 0x9b, 0xbd,
      0x55, 0x59, 0xca, 0x55, 0xd3, 0x42, 0xb5, 0x06,
      0x84, 0xbe, 0x73, 0x11, 0xc9, 0xd0, 0xa8, 0xbf,
      0xb9, 0xee, 0x66, 0xf5, 0x93, 0xef, 0xef, 0xdd,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__EditDataset_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8d, 0x21, 0x81, 0xd4, 0xe9, 0x24, 0x39, 0x12,
      0x34, 0x2d, 0xd7, 0x92, 0x2f, 0x61, 0x74, 0xdf,
      0x95, 0xcd, 0xb6, 0xf4, 0xe3, 0x82, 0x16, 0x95,
      0x45, 0x63, 0x2b, 0x76, 0xb3, 0x02, 0x8c, 0xab,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char physical_ai_interfaces__srv__EditDataset__TYPE_NAME[] = "physical_ai_interfaces/srv/EditDataset";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char physical_ai_interfaces__srv__EditDataset_Event__TYPE_NAME[] = "physical_ai_interfaces/srv/EditDataset_Event";
static char physical_ai_interfaces__srv__EditDataset_Request__TYPE_NAME[] = "physical_ai_interfaces/srv/EditDataset_Request";
static char physical_ai_interfaces__srv__EditDataset_Response__TYPE_NAME[] = "physical_ai_interfaces/srv/EditDataset_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__srv__EditDataset__FIELD_NAME__request_message[] = "request_message";
static char physical_ai_interfaces__srv__EditDataset__FIELD_NAME__response_message[] = "response_message";
static char physical_ai_interfaces__srv__EditDataset__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__EditDataset__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__EditDataset__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__EditDataset_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__EditDataset_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__EditDataset_Event__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__EditDataset__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Event__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__EditDataset__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__EditDataset__TYPE_NAME, 38, 38},
      {physical_ai_interfaces__srv__EditDataset__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__EditDataset__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__srv__EditDataset_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__EditDataset_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = physical_ai_interfaces__srv__EditDataset_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__mode[] = "mode";
static char physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__merge_dataset_list[] = "merge_dataset_list";
static char physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__delete_dataset_path[] = "delete_dataset_path";
static char physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__output_path[] = "output_path";
static char physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__delete_episode_num[] = "delete_episode_num";
static char physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__upload_huggingface[] = "upload_huggingface";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__EditDataset_Request__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__merge_dataset_list, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__delete_dataset_path, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__output_path, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__delete_episode_num, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Request__FIELD_NAME__upload_huggingface, 18, 18},
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
physical_ai_interfaces__srv__EditDataset_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__EditDataset_Request__TYPE_NAME, 46, 46},
      {physical_ai_interfaces__srv__EditDataset_Request__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__EditDataset_Response__FIELD_NAME__success[] = "success";
static char physical_ai_interfaces__srv__EditDataset_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__EditDataset_Response__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__EditDataset_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__EditDataset_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__EditDataset_Response__TYPE_NAME, 47, 47},
      {physical_ai_interfaces__srv__EditDataset_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__EditDataset_Event__FIELD_NAME__info[] = "info";
static char physical_ai_interfaces__srv__EditDataset_Event__FIELD_NAME__request[] = "request";
static char physical_ai_interfaces__srv__EditDataset_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__EditDataset_Event__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__EditDataset_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__EditDataset_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__EditDataset_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__EditDataset_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__EditDataset_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__EditDataset_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__EditDataset_Event__TYPE_NAME, 44, 44},
      {physical_ai_interfaces__srv__EditDataset_Event__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__EditDataset_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__srv__EditDataset_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__EditDataset_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "########################################\n"
  "# Constants\n"
  "########################################\n"
  "# command\n"
  "uint8 MERGE = 0\n"
  "uint8 DELETE = 1\n"
  "\n"
  "uint8 mode\n"
  "string[] merge_dataset_list\n"
  "string delete_dataset_path\n"
  "string output_path\n"
  "uint16[] delete_episode_num\n"
  "bool upload_huggingface\n"
  "---\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__EditDataset__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__EditDataset__TYPE_NAME, 38, 38},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 307, 307},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__EditDataset_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__EditDataset_Request__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__EditDataset_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__EditDataset_Response__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__EditDataset_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__EditDataset_Event__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__EditDataset__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__EditDataset__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__srv__EditDataset_Event__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__EditDataset_Request__get_individual_type_description_source(NULL);
    sources[4] = *physical_ai_interfaces__srv__EditDataset_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__EditDataset_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__EditDataset_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__EditDataset_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__EditDataset_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__EditDataset_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__EditDataset_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__srv__EditDataset_Request__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__EditDataset_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
