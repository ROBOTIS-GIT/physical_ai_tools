// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:srv/GetSavedPolicyList.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/srv/detail/get_saved_policy_list__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetSavedPolicyList__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe3, 0x91, 0x0c, 0x08, 0xe1, 0x03, 0xdd, 0x2f,
      0x4f, 0xf0, 0xbf, 0xd0, 0x45, 0x1a, 0xd7, 0x41,
      0x33, 0xc0, 0x28, 0x24, 0xaa, 0x5e, 0x27, 0x65,
      0xfc, 0x88, 0x42, 0x43, 0x87, 0x88, 0xed, 0x05,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x03, 0x01, 0x26, 0xd3, 0x38, 0xd3, 0xdd, 0xb9,
      0x80, 0xd0, 0x18, 0xe6, 0x65, 0xe8, 0xb0, 0xb6,
      0x36, 0x0e, 0x08, 0x10, 0x41, 0xd2, 0xbf, 0x09,
      0x7d, 0x56, 0x9e, 0x9b, 0x7f, 0x54, 0x1a, 0xff,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5f, 0x8a, 0xdb, 0xf6, 0x06, 0xdc, 0xdd, 0x40,
      0x61, 0x93, 0x18, 0xbc, 0x70, 0x06, 0x99, 0x57,
      0x9c, 0xa7, 0xd1, 0x01, 0x8f, 0xdb, 0xff, 0xb5,
      0xd7, 0xb6, 0xd2, 0x69, 0x2e, 0x91, 0x77, 0x81,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetSavedPolicyList_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbd, 0x2d, 0x51, 0x9c, 0xa8, 0x4d, 0x2d, 0xbf,
      0x91, 0x2d, 0xab, 0x24, 0xa7, 0x4c, 0x57, 0xb7,
      0xc5, 0x7a, 0x35, 0x29, 0x98, 0x2b, 0x95, 0x6c,
      0x7b, 0x90, 0x83, 0xc8, 0x94, 0xd6, 0x01, 0xee,
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

static char physical_ai_interfaces__srv__GetSavedPolicyList__TYPE_NAME[] = "physical_ai_interfaces/srv/GetSavedPolicyList";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char physical_ai_interfaces__srv__GetSavedPolicyList_Event__TYPE_NAME[] = "physical_ai_interfaces/srv/GetSavedPolicyList_Event";
static char physical_ai_interfaces__srv__GetSavedPolicyList_Request__TYPE_NAME[] = "physical_ai_interfaces/srv/GetSavedPolicyList_Request";
static char physical_ai_interfaces__srv__GetSavedPolicyList_Response__TYPE_NAME[] = "physical_ai_interfaces/srv/GetSavedPolicyList_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetSavedPolicyList__FIELD_NAME__request_message[] = "request_message";
static char physical_ai_interfaces__srv__GetSavedPolicyList__FIELD_NAME__response_message[] = "response_message";
static char physical_ai_interfaces__srv__GetSavedPolicyList__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetSavedPolicyList__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetSavedPolicyList_Request__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetSavedPolicyList_Response__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__GetSavedPolicyList_Event__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__GetSavedPolicyList__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Event__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Request__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Response__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetSavedPolicyList__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetSavedPolicyList__TYPE_NAME, 45, 45},
      {physical_ai_interfaces__srv__GetSavedPolicyList__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__GetSavedPolicyList__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__srv__GetSavedPolicyList_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetSavedPolicyList_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetSavedPolicyList_Request__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetSavedPolicyList_Request__TYPE_NAME, 53, 53},
      {physical_ai_interfaces__srv__GetSavedPolicyList_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELD_NAME__saved_policy_path[] = "saved_policy_path";
static char physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELD_NAME__saved_policy_type[] = "saved_policy_type";
static char physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELD_NAME__success[] = "success";
static char physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELD_NAME__saved_policy_path, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELD_NAME__saved_policy_type, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELD_NAME__message, 7, 7},
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
physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetSavedPolicyList_Response__TYPE_NAME, 54, 54},
      {physical_ai_interfaces__srv__GetSavedPolicyList_Response__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__GetSavedPolicyList_Event__FIELD_NAME__info[] = "info";
static char physical_ai_interfaces__srv__GetSavedPolicyList_Event__FIELD_NAME__request[] = "request";
static char physical_ai_interfaces__srv__GetSavedPolicyList_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__GetSavedPolicyList_Event__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__GetSavedPolicyList_Request__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__GetSavedPolicyList_Response__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__GetSavedPolicyList_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Request__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Response__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetSavedPolicyList_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__GetSavedPolicyList_Event__TYPE_NAME, 51, 51},
      {physical_ai_interfaces__srv__GetSavedPolicyList_Event__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__GetSavedPolicyList_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "string[] saved_policy_path\n"
  "string[] saved_policy_type\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetSavedPolicyList__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetSavedPolicyList__TYPE_NAME, 45, 45},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 86, 86},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Request__TYPE_NAME, 53, 53},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Response__TYPE_NAME, 54, 54},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetSavedPolicyList_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__GetSavedPolicyList_Event__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetSavedPolicyList__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetSavedPolicyList__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__srv__GetSavedPolicyList_Event__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_individual_type_description_source(NULL);
    sources[4] = *physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetSavedPolicyList_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__GetSavedPolicyList_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__srv__GetSavedPolicyList_Request__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__GetSavedPolicyList_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
