// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:srv/GetDatasetInfo.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/srv/detail/get_dataset_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetDatasetInfo__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8e, 0xaf, 0x6e, 0xee, 0x3b, 0xd9, 0xad, 0x23,
      0x9d, 0xa6, 0xb9, 0x10, 0x31, 0xd0, 0x36, 0x35,
      0xd7, 0xa8, 0x28, 0xa9, 0xa7, 0xd3, 0x75, 0xad,
      0xac, 0x17, 0xd5, 0x78, 0x11, 0xe4, 0x79, 0x80,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetDatasetInfo_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x92, 0xe8, 0x57, 0x75, 0x88, 0x2d, 0x03, 0xfe,
      0x73, 0xb8, 0xdc, 0x57, 0xe8, 0xd3, 0x16, 0x40,
      0x92, 0xc0, 0x8a, 0x6d, 0x21, 0x7b, 0xb4, 0xd0,
      0x98, 0x4c, 0x54, 0x6c, 0x22, 0xc7, 0x18, 0x7d,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetDatasetInfo_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x57, 0xbf, 0x97, 0x32, 0x10, 0x24, 0x47, 0x7d,
      0x3f, 0x73, 0x9a, 0x57, 0x90, 0x9f, 0x50, 0x7e,
      0x72, 0x44, 0x89, 0x9a, 0x47, 0x6d, 0x9a, 0x5b,
      0x04, 0xca, 0x34, 0x64, 0x2f, 0x63, 0x7e, 0x86,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetDatasetInfo_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcc, 0x2e, 0xe2, 0xac, 0x6b, 0x41, 0xc4, 0xf8,
      0xa7, 0xd7, 0x54, 0xb7, 0x44, 0x9c, 0x03, 0x16,
      0xd5, 0x09, 0x73, 0xb7, 0xfb, 0xdf, 0x5c, 0x48,
      0xe1, 0x02, 0xef, 0xfe, 0x29, 0x42, 0xd2, 0x36,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "physical_ai_interfaces/msg/detail/dataset_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t physical_ai_interfaces__msg__DatasetInfo__EXPECTED_HASH = {1, {
    0x04, 0x17, 0x39, 0xe9, 0x3d, 0x2a, 0x32, 0x23,
    0xc8, 0x2b, 0x9f, 0x00, 0xfd, 0x05, 0x3b, 0xaf,
    0x5c, 0xbb, 0x35, 0x8a, 0x5e, 0xbb, 0xa3, 0x8f,
    0xde, 0x4b, 0xa1, 0xe3, 0x22, 0x88, 0x00, 0x94,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char physical_ai_interfaces__srv__GetDatasetInfo__TYPE_NAME[] = "physical_ai_interfaces/srv/GetDatasetInfo";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char physical_ai_interfaces__msg__DatasetInfo__TYPE_NAME[] = "physical_ai_interfaces/msg/DatasetInfo";
static char physical_ai_interfaces__srv__GetDatasetInfo_Event__TYPE_NAME[] = "physical_ai_interfaces/srv/GetDatasetInfo_Event";
static char physical_ai_interfaces__srv__GetDatasetInfo_Request__TYPE_NAME[] = "physical_ai_interfaces/srv/GetDatasetInfo_Request";
static char physical_ai_interfaces__srv__GetDatasetInfo_Response__TYPE_NAME[] = "physical_ai_interfaces/srv/GetDatasetInfo_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetDatasetInfo__FIELD_NAME__request_message[] = "request_message";
static char physical_ai_interfaces__srv__GetDatasetInfo__FIELD_NAME__response_message[] = "response_message";
static char physical_ai_interfaces__srv__GetDatasetInfo__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetDatasetInfo__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetDatasetInfo__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetDatasetInfo_Request__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetDatasetInfo_Response__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetDatasetInfo_Event__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__GetDatasetInfo__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__DatasetInfo__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Event__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Request__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Response__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetDatasetInfo__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetDatasetInfo__TYPE_NAME, 41, 41},
      {physical_ai_interfaces__srv__GetDatasetInfo__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__GetDatasetInfo__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&physical_ai_interfaces__msg__DatasetInfo__EXPECTED_HASH, physical_ai_interfaces__msg__DatasetInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__msg__DatasetInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__GetDatasetInfo_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = physical_ai_interfaces__srv__GetDatasetInfo_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = physical_ai_interfaces__srv__GetDatasetInfo_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetDatasetInfo_Request__FIELD_NAME__dataset_path[] = "dataset_path";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetDatasetInfo_Request__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Request__FIELD_NAME__dataset_path, 12, 12},
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
physical_ai_interfaces__srv__GetDatasetInfo_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetDatasetInfo_Request__TYPE_NAME, 49, 49},
      {physical_ai_interfaces__srv__GetDatasetInfo_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetDatasetInfo_Response__FIELD_NAME__dataset_info[] = "dataset_info";
static char physical_ai_interfaces__srv__GetDatasetInfo_Response__FIELD_NAME__success[] = "success";
static char physical_ai_interfaces__srv__GetDatasetInfo_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetDatasetInfo_Response__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Response__FIELD_NAME__dataset_info, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__msg__DatasetInfo__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__GetDatasetInfo_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {physical_ai_interfaces__msg__DatasetInfo__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetDatasetInfo_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetDatasetInfo_Response__TYPE_NAME, 50, 50},
      {physical_ai_interfaces__srv__GetDatasetInfo_Response__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__GetDatasetInfo_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&physical_ai_interfaces__msg__DatasetInfo__EXPECTED_HASH, physical_ai_interfaces__msg__DatasetInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = physical_ai_interfaces__msg__DatasetInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetDatasetInfo_Event__FIELD_NAME__info[] = "info";
static char physical_ai_interfaces__srv__GetDatasetInfo_Event__FIELD_NAME__request[] = "request";
static char physical_ai_interfaces__srv__GetDatasetInfo_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetDatasetInfo_Event__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__GetDatasetInfo_Request__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__GetDatasetInfo_Response__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__GetDatasetInfo_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__DatasetInfo__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Request__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetDatasetInfo_Response__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetDatasetInfo_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetDatasetInfo_Event__TYPE_NAME, 47, 47},
      {physical_ai_interfaces__srv__GetDatasetInfo_Event__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__GetDatasetInfo_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&physical_ai_interfaces__msg__DatasetInfo__EXPECTED_HASH, physical_ai_interfaces__msg__DatasetInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__msg__DatasetInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__GetDatasetInfo_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = physical_ai_interfaces__srv__GetDatasetInfo_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string dataset_path\n"
  "---\n"
  "DatasetInfo dataset_info\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetDatasetInfo__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetDatasetInfo__TYPE_NAME, 41, 41},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 77, 77},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetDatasetInfo_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetDatasetInfo_Request__TYPE_NAME, 49, 49},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetDatasetInfo_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetDatasetInfo_Response__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetDatasetInfo_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetDatasetInfo_Event__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetDatasetInfo__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetDatasetInfo__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__msg__DatasetInfo__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__GetDatasetInfo_Event__get_individual_type_description_source(NULL);
    sources[4] = *physical_ai_interfaces__srv__GetDatasetInfo_Request__get_individual_type_description_source(NULL);
    sources[5] = *physical_ai_interfaces__srv__GetDatasetInfo_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetDatasetInfo_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetDatasetInfo_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetDatasetInfo_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetDatasetInfo_Response__get_individual_type_description_source(NULL),
    sources[1] = *physical_ai_interfaces__msg__DatasetInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetDatasetInfo_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetDatasetInfo_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__msg__DatasetInfo__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__GetDatasetInfo_Request__get_individual_type_description_source(NULL);
    sources[4] = *physical_ai_interfaces__srv__GetDatasetInfo_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
