// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:srv/BrowseFile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/browse_file.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/srv/detail/browse_file__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const BrowseFile_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: action
  {
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << ", ";
  }

  // member: current_path
  {
    out << "current_path: ";
    rosidl_generator_traits::value_to_yaml(msg.current_path, out);
    out << ", ";
  }

  // member: target_name
  {
    out << "target_name: ";
    rosidl_generator_traits::value_to_yaml(msg.target_name, out);
    out << ", ";
  }

  // member: target_files
  {
    if (msg.target_files.size() == 0) {
      out << "target_files: []";
    } else {
      out << "target_files: [";
      size_t pending_items = msg.target_files.size();
      for (auto item : msg.target_files) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target_folders
  {
    if (msg.target_folders.size() == 0) {
      out << "target_folders: []";
    } else {
      out << "target_folders: [";
      size_t pending_items = msg.target_folders.size();
      for (auto item : msg.target_folders) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BrowseFile_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << "\n";
  }

  // member: current_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_path: ";
    rosidl_generator_traits::value_to_yaml(msg.current_path, out);
    out << "\n";
  }

  // member: target_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_name: ";
    rosidl_generator_traits::value_to_yaml(msg.target_name, out);
    out << "\n";
  }

  // member: target_files
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_files.size() == 0) {
      out << "target_files: []\n";
    } else {
      out << "target_files:\n";
      for (auto item : msg.target_files) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target_folders
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_folders.size() == 0) {
      out << "target_folders: []\n";
    } else {
      out << "target_folders:\n";
      for (auto item : msg.target_folders) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BrowseFile_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace physical_ai_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use physical_ai_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const physical_ai_interfaces::srv::BrowseFile_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::BrowseFile_Request & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::BrowseFile_Request>()
{
  return "physical_ai_interfaces::srv::BrowseFile_Request";
}

template<>
inline const char * name<physical_ai_interfaces::srv::BrowseFile_Request>()
{
  return "physical_ai_interfaces/srv/BrowseFile_Request";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::BrowseFile_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::BrowseFile_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::srv::BrowseFile_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'items'
#include "physical_ai_interfaces/msg/detail/browser_item__traits.hpp"

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const BrowseFile_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: current_path
  {
    out << "current_path: ";
    rosidl_generator_traits::value_to_yaml(msg.current_path, out);
    out << ", ";
  }

  // member: parent_path
  {
    out << "parent_path: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_path, out);
    out << ", ";
  }

  // member: selected_path
  {
    out << "selected_path: ";
    rosidl_generator_traits::value_to_yaml(msg.selected_path, out);
    out << ", ";
  }

  // member: items
  {
    if (msg.items.size() == 0) {
      out << "items: []";
    } else {
      out << "items: [";
      size_t pending_items = msg.items.size();
      for (auto item : msg.items) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BrowseFile_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: current_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_path: ";
    rosidl_generator_traits::value_to_yaml(msg.current_path, out);
    out << "\n";
  }

  // member: parent_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parent_path: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_path, out);
    out << "\n";
  }

  // member: selected_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selected_path: ";
    rosidl_generator_traits::value_to_yaml(msg.selected_path, out);
    out << "\n";
  }

  // member: items
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.items.size() == 0) {
      out << "items: []\n";
    } else {
      out << "items:\n";
      for (auto item : msg.items) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BrowseFile_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace physical_ai_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use physical_ai_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const physical_ai_interfaces::srv::BrowseFile_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::BrowseFile_Response & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::BrowseFile_Response>()
{
  return "physical_ai_interfaces::srv::BrowseFile_Response";
}

template<>
inline const char * name<physical_ai_interfaces::srv::BrowseFile_Response>()
{
  return "physical_ai_interfaces/srv/BrowseFile_Response";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::BrowseFile_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::BrowseFile_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::srv::BrowseFile_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const BrowseFile_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BrowseFile_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BrowseFile_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace physical_ai_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use physical_ai_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const physical_ai_interfaces::srv::BrowseFile_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::BrowseFile_Event & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::BrowseFile_Event>()
{
  return "physical_ai_interfaces::srv::BrowseFile_Event";
}

template<>
inline const char * name<physical_ai_interfaces::srv::BrowseFile_Event>()
{
  return "physical_ai_interfaces/srv/BrowseFile_Event";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::BrowseFile_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::BrowseFile_Event>
  : std::integral_constant<bool, has_bounded_size<physical_ai_interfaces::srv::BrowseFile_Request>::value && has_bounded_size<physical_ai_interfaces::srv::BrowseFile_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<physical_ai_interfaces::srv::BrowseFile_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<physical_ai_interfaces::srv::BrowseFile>()
{
  return "physical_ai_interfaces::srv::BrowseFile";
}

template<>
inline const char * name<physical_ai_interfaces::srv::BrowseFile>()
{
  return "physical_ai_interfaces/srv/BrowseFile";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::BrowseFile>
  : std::integral_constant<
    bool,
    has_fixed_size<physical_ai_interfaces::srv::BrowseFile_Request>::value &&
    has_fixed_size<physical_ai_interfaces::srv::BrowseFile_Response>::value
  >
{
};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::BrowseFile>
  : std::integral_constant<
    bool,
    has_bounded_size<physical_ai_interfaces::srv::BrowseFile_Request>::value &&
    has_bounded_size<physical_ai_interfaces::srv::BrowseFile_Response>::value
  >
{
};

template<>
struct is_service<physical_ai_interfaces::srv::BrowseFile>
  : std::true_type
{
};

template<>
struct is_service_request<physical_ai_interfaces::srv::BrowseFile_Request>
  : std::true_type
{
};

template<>
struct is_service_response<physical_ai_interfaces::srv::BrowseFile_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__BROWSE_FILE__TRAITS_HPP_
