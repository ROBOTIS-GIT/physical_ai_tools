// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from physical_ai_interfaces:srv/BrowseFile.idl
// generated code does not contain a copyright notice

#include "physical_ai_interfaces/srv/detail/browse_file__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__BrowseFile__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3c, 0x9c, 0x3b, 0x3d, 0xdd, 0xa3, 0x5f, 0x55,
      0x95, 0x6c, 0xe3, 0x78, 0x55, 0x55, 0x52, 0xf0,
      0x9f, 0xa5, 0x7e, 0x71, 0xb1, 0xa0, 0xd1, 0x45,
      0x0d, 0x1b, 0x6d, 0x1c, 0x4e, 0xc7, 0x34, 0x51,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__BrowseFile_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x54, 0xa1, 0x76, 0xae, 0x16, 0x46, 0x40, 0xbf,
      0x63, 0x00, 0xc3, 0xb9, 0xaa, 0x5c, 0xb1, 0x70,
      0x8e, 0xb8, 0xa7, 0xf4, 0x61, 0xa1, 0xc7, 0xa2,
      0xab, 0x3f, 0x84, 0xfc, 0xb8, 0xbf, 0x52, 0xbe,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__BrowseFile_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x77, 0x19, 0x0a, 0x4c, 0x61, 0x38, 0xbb, 0x9c,
      0xde, 0xd9, 0x3e, 0x53, 0x7b, 0x8c, 0xc9, 0xe5,
      0x34, 0xaa, 0x1c, 0x28, 0xf5, 0xf7, 0xe0, 0xf7,
      0x1d, 0x13, 0x0f, 0x74, 0x37, 0x3d, 0x5d, 0x16,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__BrowseFile_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x73, 0xca, 0x65, 0xad, 0x7d, 0x3c, 0x83, 0x14,
      0x00, 0x4e, 0x31, 0xf8, 0x70, 0x40, 0x57, 0xe2,
      0xd2, 0xc5, 0xca, 0xeb, 0xc8, 0xcf, 0x32, 0xcd,
      0x28, 0x44, 0x2f, 0xc8, 0xe2, 0xe0, 0xc8, 0xa4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "physical_ai_interfaces/msg/detail/browser_item__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t physical_ai_interfaces__msg__BrowserItem__EXPECTED_HASH = {1, {
    0xdc, 0xeb, 0xf7, 0x22, 0x45, 0xcd, 0x42, 0x0b,
    0x72, 0x44, 0x5a, 0x0f, 0x73, 0x68, 0xbb, 0xc8,
    0x46, 0x9d, 0x80, 0x36, 0x77, 0x91, 0xb4, 0xe7,
    0x8e, 0xd4, 0xbc, 0x0d, 0x68, 0x36, 0x92, 0x18,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char physical_ai_interfaces__srv__BrowseFile__TYPE_NAME[] = "physical_ai_interfaces/srv/BrowseFile";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char physical_ai_interfaces__msg__BrowserItem__TYPE_NAME[] = "physical_ai_interfaces/msg/BrowserItem";
static char physical_ai_interfaces__srv__BrowseFile_Event__TYPE_NAME[] = "physical_ai_interfaces/srv/BrowseFile_Event";
static char physical_ai_interfaces__srv__BrowseFile_Request__TYPE_NAME[] = "physical_ai_interfaces/srv/BrowseFile_Request";
static char physical_ai_interfaces__srv__BrowseFile_Response__TYPE_NAME[] = "physical_ai_interfaces/srv/BrowseFile_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char physical_ai_interfaces__srv__BrowseFile__FIELD_NAME__request_message[] = "request_message";
static char physical_ai_interfaces__srv__BrowseFile__FIELD_NAME__response_message[] = "response_message";
static char physical_ai_interfaces__srv__BrowseFile__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__BrowseFile__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__BrowseFile__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__BrowseFile_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__BrowseFile_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {physical_ai_interfaces__srv__BrowseFile_Event__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__BrowseFile__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__BrowserItem__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Event__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Response__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__BrowseFile__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__BrowseFile__TYPE_NAME, 37, 37},
      {physical_ai_interfaces__srv__BrowseFile__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__BrowseFile__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&physical_ai_interfaces__msg__BrowserItem__EXPECTED_HASH, physical_ai_interfaces__msg__BrowserItem__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__msg__BrowserItem__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__BrowseFile_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = physical_ai_interfaces__srv__BrowseFile_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = physical_ai_interfaces__srv__BrowseFile_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__action[] = "action";
static char physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__current_path[] = "current_path";
static char physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__target_name[] = "target_name";
static char physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__target_files[] = "target_files";
static char physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__target_folders[] = "target_folders";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__BrowseFile_Request__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__action, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__current_path, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__target_name, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__target_files, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Request__FIELD_NAME__target_folders, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__BrowseFile_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__BrowseFile_Request__TYPE_NAME, 45, 45},
      {physical_ai_interfaces__srv__BrowseFile_Request__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__success[] = "success";
static char physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__message[] = "message";
static char physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__current_path[] = "current_path";
static char physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__parent_path[] = "parent_path";
static char physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__selected_path[] = "selected_path";
static char physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__items[] = "items";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__BrowseFile_Response__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__current_path, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__parent_path, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__selected_path, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Response__FIELD_NAME__items, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {physical_ai_interfaces__msg__BrowserItem__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__BrowseFile_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {physical_ai_interfaces__msg__BrowserItem__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__BrowseFile_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__BrowseFile_Response__TYPE_NAME, 46, 46},
      {physical_ai_interfaces__srv__BrowseFile_Response__FIELDS, 6, 6},
    },
    {physical_ai_interfaces__srv__BrowseFile_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&physical_ai_interfaces__msg__BrowserItem__EXPECTED_HASH, physical_ai_interfaces__msg__BrowserItem__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = physical_ai_interfaces__msg__BrowserItem__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char physical_ai_interfaces__srv__BrowseFile_Event__FIELD_NAME__info[] = "info";
static char physical_ai_interfaces__srv__BrowseFile_Event__FIELD_NAME__request[] = "request";
static char physical_ai_interfaces__srv__BrowseFile_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field physical_ai_interfaces__srv__BrowseFile_Event__FIELDS[] = {
  {
    {physical_ai_interfaces__srv__BrowseFile_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__BrowseFile_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {physical_ai_interfaces__srv__BrowseFile_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription physical_ai_interfaces__srv__BrowseFile_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__msg__BrowserItem__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {physical_ai_interfaces__srv__BrowseFile_Response__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__BrowseFile_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {physical_ai_interfaces__srv__BrowseFile_Event__TYPE_NAME, 43, 43},
      {physical_ai_interfaces__srv__BrowseFile_Event__FIELDS, 3, 3},
    },
    {physical_ai_interfaces__srv__BrowseFile_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&physical_ai_interfaces__msg__BrowserItem__EXPECTED_HASH, physical_ai_interfaces__msg__BrowserItem__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = physical_ai_interfaces__msg__BrowserItem__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = physical_ai_interfaces__srv__BrowseFile_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = physical_ai_interfaces__srv__BrowseFile_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# File browser navigation service\n"
  "\n"
  "# Request\n"
  "string action           # Action to perform: \"browse\", \"get_path\", \"go_parent\"\n"
  "string current_path     # Current path (used depending on action)\n"
  "string target_name      # Name of file/folder to select (for \"browse\" action)\n"
  "string[] target_files   # Optional: Files to check for in subdirectories (parallel search)\n"
  "string[] target_folders # Optional: Folders to check for in subdirectories (parallel search)\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool success           # Whether the operation was successful\n"
  "string message         # Status or error message\n"
  "string current_path    # Current path\n"
  "string parent_path     # Parent directory path\n"
  "string selected_path   # Full path of the selected item\n"
  "\n"
  "# Contents of the current directory\n"
  "BrowserItem[] items";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__BrowseFile__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__BrowseFile__TYPE_NAME, 37, 37},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 777, 777},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__BrowseFile_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__BrowseFile_Request__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__BrowseFile_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__BrowseFile_Response__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__BrowseFile_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {physical_ai_interfaces__srv__BrowseFile_Event__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__BrowseFile__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__BrowseFile__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__msg__BrowserItem__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__BrowseFile_Event__get_individual_type_description_source(NULL);
    sources[4] = *physical_ai_interfaces__srv__BrowseFile_Request__get_individual_type_description_source(NULL);
    sources[5] = *physical_ai_interfaces__srv__BrowseFile_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__BrowseFile_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__BrowseFile_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__BrowseFile_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__BrowseFile_Response__get_individual_type_description_source(NULL),
    sources[1] = *physical_ai_interfaces__msg__BrowserItem__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__BrowseFile_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *physical_ai_interfaces__srv__BrowseFile_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *physical_ai_interfaces__msg__BrowserItem__get_individual_type_description_source(NULL);
    sources[3] = *physical_ai_interfaces__srv__BrowseFile_Request__get_individual_type_description_source(NULL);
    sources[4] = *physical_ai_interfaces__srv__BrowseFile_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
