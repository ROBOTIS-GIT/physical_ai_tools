// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:srv/ControlHfServer.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/control_hf_server.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/srv/detail/control_hf_server__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ControlHfServer_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: repo_id
  {
    out << "repo_id: ";
    rosidl_generator_traits::value_to_yaml(msg.repo_id, out);
    out << ", ";
  }

  // member: local_dir
  {
    out << "local_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.local_dir, out);
    out << ", ";
  }

  // member: repo_type
  {
    out << "repo_type: ";
    rosidl_generator_traits::value_to_yaml(msg.repo_type, out);
    out << ", ";
  }

  // member: author
  {
    out << "author: ";
    rosidl_generator_traits::value_to_yaml(msg.author, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlHfServer_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: repo_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "repo_id: ";
    rosidl_generator_traits::value_to_yaml(msg.repo_id, out);
    out << "\n";
  }

  // member: local_dir
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.local_dir, out);
    out << "\n";
  }

  // member: repo_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "repo_type: ";
    rosidl_generator_traits::value_to_yaml(msg.repo_type, out);
    out << "\n";
  }

  // member: author
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "author: ";
    rosidl_generator_traits::value_to_yaml(msg.author, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlHfServer_Request & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::ControlHfServer_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::ControlHfServer_Request & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::ControlHfServer_Request>()
{
  return "physical_ai_interfaces::srv::ControlHfServer_Request";
}

template<>
inline const char * name<physical_ai_interfaces::srv::ControlHfServer_Request>()
{
  return "physical_ai_interfaces/srv/ControlHfServer_Request";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::ControlHfServer_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::ControlHfServer_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::srv::ControlHfServer_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ControlHfServer_Response & msg,
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlHfServer_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlHfServer_Response & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::ControlHfServer_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::ControlHfServer_Response & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::ControlHfServer_Response>()
{
  return "physical_ai_interfaces::srv::ControlHfServer_Response";
}

template<>
inline const char * name<physical_ai_interfaces::srv::ControlHfServer_Response>()
{
  return "physical_ai_interfaces/srv/ControlHfServer_Response";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::ControlHfServer_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::ControlHfServer_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::srv::ControlHfServer_Response>
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
  const ControlHfServer_Event & msg,
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
  const ControlHfServer_Event & msg,
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

inline std::string to_yaml(const ControlHfServer_Event & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::ControlHfServer_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::ControlHfServer_Event & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::ControlHfServer_Event>()
{
  return "physical_ai_interfaces::srv::ControlHfServer_Event";
}

template<>
inline const char * name<physical_ai_interfaces::srv::ControlHfServer_Event>()
{
  return "physical_ai_interfaces/srv/ControlHfServer_Event";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::ControlHfServer_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::ControlHfServer_Event>
  : std::integral_constant<bool, has_bounded_size<physical_ai_interfaces::srv::ControlHfServer_Request>::value && has_bounded_size<physical_ai_interfaces::srv::ControlHfServer_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<physical_ai_interfaces::srv::ControlHfServer_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<physical_ai_interfaces::srv::ControlHfServer>()
{
  return "physical_ai_interfaces::srv::ControlHfServer";
}

template<>
inline const char * name<physical_ai_interfaces::srv::ControlHfServer>()
{
  return "physical_ai_interfaces/srv/ControlHfServer";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::ControlHfServer>
  : std::integral_constant<
    bool,
    has_fixed_size<physical_ai_interfaces::srv::ControlHfServer_Request>::value &&
    has_fixed_size<physical_ai_interfaces::srv::ControlHfServer_Response>::value
  >
{
};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::ControlHfServer>
  : std::integral_constant<
    bool,
    has_bounded_size<physical_ai_interfaces::srv::ControlHfServer_Request>::value &&
    has_bounded_size<physical_ai_interfaces::srv::ControlHfServer_Response>::value
  >
{
};

template<>
struct is_service<physical_ai_interfaces::srv::ControlHfServer>
  : std::true_type
{
};

template<>
struct is_service_request<physical_ai_interfaces::srv::ControlHfServer_Request>
  : std::true_type
{
};

template<>
struct is_service_response<physical_ai_interfaces::srv::ControlHfServer_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__CONTROL_HF_SERVER__TRAITS_HPP_
