// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:srv/GetPolicyList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_policy_list.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_POLICY_LIST__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_POLICY_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/srv/detail/get_policy_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetPolicyList_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetPolicyList_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetPolicyList_Request & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::GetPolicyList_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::GetPolicyList_Request & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::GetPolicyList_Request>()
{
  return "physical_ai_interfaces::srv::GetPolicyList_Request";
}

template<>
inline const char * name<physical_ai_interfaces::srv::GetPolicyList_Request>()
{
  return "physical_ai_interfaces/srv/GetPolicyList_Request";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::GetPolicyList_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::GetPolicyList_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<physical_ai_interfaces::srv::GetPolicyList_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetPolicyList_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: policy_list
  {
    if (msg.policy_list.size() == 0) {
      out << "policy_list: []";
    } else {
      out << "policy_list: [";
      size_t pending_items = msg.policy_list.size();
      for (auto item : msg.policy_list) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: device_list
  {
    if (msg.device_list.size() == 0) {
      out << "device_list: []";
    } else {
      out << "device_list: [";
      size_t pending_items = msg.device_list.size();
      for (auto item : msg.device_list) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

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
  const GetPolicyList_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: policy_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.policy_list.size() == 0) {
      out << "policy_list: []\n";
    } else {
      out << "policy_list:\n";
      for (auto item : msg.policy_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: device_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.device_list.size() == 0) {
      out << "device_list: []\n";
    } else {
      out << "device_list:\n";
      for (auto item : msg.device_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

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

inline std::string to_yaml(const GetPolicyList_Response & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::GetPolicyList_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::GetPolicyList_Response & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::GetPolicyList_Response>()
{
  return "physical_ai_interfaces::srv::GetPolicyList_Response";
}

template<>
inline const char * name<physical_ai_interfaces::srv::GetPolicyList_Response>()
{
  return "physical_ai_interfaces/srv/GetPolicyList_Response";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::GetPolicyList_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::GetPolicyList_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::srv::GetPolicyList_Response>
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
  const GetPolicyList_Event & msg,
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
  const GetPolicyList_Event & msg,
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

inline std::string to_yaml(const GetPolicyList_Event & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::GetPolicyList_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::GetPolicyList_Event & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::GetPolicyList_Event>()
{
  return "physical_ai_interfaces::srv::GetPolicyList_Event";
}

template<>
inline const char * name<physical_ai_interfaces::srv::GetPolicyList_Event>()
{
  return "physical_ai_interfaces/srv/GetPolicyList_Event";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::GetPolicyList_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::GetPolicyList_Event>
  : std::integral_constant<bool, has_bounded_size<physical_ai_interfaces::srv::GetPolicyList_Request>::value && has_bounded_size<physical_ai_interfaces::srv::GetPolicyList_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<physical_ai_interfaces::srv::GetPolicyList_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<physical_ai_interfaces::srv::GetPolicyList>()
{
  return "physical_ai_interfaces::srv::GetPolicyList";
}

template<>
inline const char * name<physical_ai_interfaces::srv::GetPolicyList>()
{
  return "physical_ai_interfaces/srv/GetPolicyList";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::GetPolicyList>
  : std::integral_constant<
    bool,
    has_fixed_size<physical_ai_interfaces::srv::GetPolicyList_Request>::value &&
    has_fixed_size<physical_ai_interfaces::srv::GetPolicyList_Response>::value
  >
{
};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::GetPolicyList>
  : std::integral_constant<
    bool,
    has_bounded_size<physical_ai_interfaces::srv::GetPolicyList_Request>::value &&
    has_bounded_size<physical_ai_interfaces::srv::GetPolicyList_Response>::value
  >
{
};

template<>
struct is_service<physical_ai_interfaces::srv::GetPolicyList>
  : std::true_type
{
};

template<>
struct is_service_request<physical_ai_interfaces::srv::GetPolicyList_Request>
  : std::true_type
{
};

template<>
struct is_service_response<physical_ai_interfaces::srv::GetPolicyList_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_POLICY_LIST__TRAITS_HPP_
