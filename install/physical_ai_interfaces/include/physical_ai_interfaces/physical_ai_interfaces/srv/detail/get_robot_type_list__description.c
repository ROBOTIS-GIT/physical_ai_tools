// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:srv/GetRobotTypeList.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/srv/detail/get_robot_type_list__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetRobotTypeList__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfb, 0x8c, 0xc9, 0x95, 0x96, 0x3b, 0x3b, 0x78,
      0x57, 0x13, 0x72, 0x18, 0x8b, 0x41, 0xa8, 0xe2,
      0xf1, 0xd0, 0xf4, 0x7b, 0xef, 0xb4, 0xf1, 0x19,
      0x54, 0x44, 0xc1, 0x2f, 0x24, 0x09, 0xb7, 0xef,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetRobotTypeList_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x44, 0x00, 0xbc, 0xc1, 0xd0, 0x70, 0xd8, 0xe2,
      0x67, 0xc1, 0xdd, 0xea, 0x5b, 0x94, 0x30, 0x3c,
      0x02, 0xd0, 0xf6, 0x6f, 0x6b, 0xc2, 0x27, 0xc2,
      0x6f, 0x74, 0xde, 0xee, 0x9d, 0x87, 0x58, 0x95,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetRobotTypeList_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x27, 0xe1, 0x6b, 0xed, 0xac, 0x9e, 0xf4, 0xee,
      0x42, 0xfa, 0x36, 0x3d, 0x82, 0x49, 0xa2, 0x4e,
      0xa1, 0xae, 0x80, 0x8e, 0x02, 0xb0, 0x3b, 0x14,
      0xc2, 0xda, 0x5b, 0xa1, 0x99, 0xef, 0x1c, 0xc9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetRobotTypeList_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0xa7, 0x54, 0xb2, 0xdf, 0xd9, 0xd6, 0x98,
      0xe7, 0xf0, 0x57, 0x0b, 0x01, 0x3b, 0x3c, 0x2b,
      0xfa, 0x5a, 0x99, 0xd6, 0x13, 0x94, 0xe7, 0x3a,
      0x97, 0xa2, 0x1c, 0x46, 0xe7, 0x56, 0x3c, 0x15,
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

static char physical_ai_interfaces__srv__GetRobotTypeList__TYPE_NAME[] = "physical_ai_interfaces/srv/GetRobotTypeList";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char physical_ai_interfaces__srv__GetRobotTypeList_Event__TYPE_NAME[] = "physical_ai_interfaces/srv/GetRobotTypeList_Event";
static char physical_ai_interfaces__srv__GetRobotTypeList_Request__TYPE_NAME[] = "physical_ai_interfaces/srv/GetRobotTypeList_Request";
static char physical_ai_interfaces__srv__GetRobotTypeList_Response__TYPE_NAME[] = "physical_ai_interfaces/srv/GetRobotTypeList_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetRobotTypeList__FIELD_NAME__request_message[] = "request_message";
static char physical_ai_interfaces__srv__GetRobotTypeList__FIELD_NAME__response_message[] = "response_message";
static char physical_ai_interfaces__srv__GetRobotTypeList__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetRobotTypeList__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetRobotTypeList__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetRobotTypeList_Request__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetRobotTypeList_Response__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetRobotTypeList_Event__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__GetRobotTypeList__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Event__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Request__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Response__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetRobotTypeList__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetRobotTypeList__TYPE_NAME, 43, 43},
      {physical_ai_interfaces__srv__GetRobotTypeList__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__GetRobotTypeList__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__srv__GetRobotTypeList_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__GetRobotTypeList_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = physical_ai_interfaces__srv__GetRobotTypeList_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetRobotTypeList_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetRobotTypeList_Request__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
physical_ai_interfaces__srv__GetRobotTypeList_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetRobotTypeList_Request__TYPE_NAME, 51, 51},
      {physical_ai_interfaces__srv__GetRobotTypeList_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetRobotTypeList_Response__FIELD_NAME__robot_types[] = "robot_types";
static char physical_ai_interfaces__srv__GetRobotTypeList_Response__FIELD_NAME__success[] = "success";
static char physical_ai_interfaces__srv__GetRobotTypeList_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetRobotTypeList_Response__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Response__FIELD_NAME__robot_types, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Response__FIELD_NAME__message, 7, 7},
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
physical_ai_interfaces__srv__GetRobotTypeList_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetRobotTypeList_Response__TYPE_NAME, 52, 52},
      {physical_ai_interfaces__srv__GetRobotTypeList_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetRobotTypeList_Event__FIELD_NAME__info[] = "info";
static char physical_ai_interfaces__srv__GetRobotTypeList_Event__FIELD_NAME__request[] = "request";
static char physical_ai_interfaces__srv__GetRobotTypeList_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetRobotTypeList_Event__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__GetRobotTypeList_Request__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__GetRobotTypeList_Response__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__GetRobotTypeList_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Request__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetRobotTypeList_Response__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetRobotTypeList_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetRobotTypeList_Event__TYPE_NAME, 49, 49},
      {physical_ai_interfaces__srv__GetRobotTypeList_Event__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__GetRobotTypeList_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__srv__GetRobotTypeList_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__GetRobotTypeList_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "string[] robot_types\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetRobotTypeList__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetRobotTypeList__TYPE_NAME, 43, 43},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 53, 53},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetRobotTypeList_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetRobotTypeList_Request__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetRobotTypeList_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetRobotTypeList_Response__TYPE_NAME, 52, 52},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetRobotTypeList_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetRobotTypeList_Event__TYPE_NAME, 49, 49},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetRobotTypeList__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__srv__GetRobotTypeList_Event__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__GetRobotTypeList_Request__get_individual_type_description_source(NULL);
    sources[4] = *physical_ai_interfaces__srv__GetRobotTypeList_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetRobotTypeList_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetRobotTypeList_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetRobotTypeList_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__srv__GetRobotTypeList_Request__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__GetRobotTypeList_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
