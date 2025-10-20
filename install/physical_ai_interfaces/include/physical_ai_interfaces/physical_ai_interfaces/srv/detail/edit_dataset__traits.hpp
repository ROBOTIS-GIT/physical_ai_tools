// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:srv/EditDataset.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/edit_dataset.hpp"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/srv/detail/edit_dataset__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const EditDataset_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: merge_dataset_list
  {
    if (msg.merge_dataset_list.size() == 0) {
      out << "merge_dataset_list: []";
    } else {
      out << "merge_dataset_list: [";
      size_t pending_items = msg.merge_dataset_list.size();
      for (auto item : msg.merge_dataset_list) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: delete_dataset_path
  {
    out << "delete_dataset_path: ";
    rosidl_generator_traits::value_to_yaml(msg.delete_dataset_path, out);
    out << ", ";
  }

  // member: output_path
  {
    out << "output_path: ";
    rosidl_generator_traits::value_to_yaml(msg.output_path, out);
    out << ", ";
  }

  // member: delete_episode_num
  {
    if (msg.delete_episode_num.size() == 0) {
      out << "delete_episode_num: []";
    } else {
      out << "delete_episode_num: [";
      size_t pending_items = msg.delete_episode_num.size();
      for (auto item : msg.delete_episode_num) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: upload_huggingface
  {
    out << "upload_huggingface: ";
    rosidl_generator_traits::value_to_yaml(msg.upload_huggingface, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EditDataset_Request & msg,
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

  // member: merge_dataset_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.merge_dataset_list.size() == 0) {
      out << "merge_dataset_list: []\n";
    } else {
      out << "merge_dataset_list:\n";
      for (auto item : msg.merge_dataset_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: delete_dataset_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delete_dataset_path: ";
    rosidl_generator_traits::value_to_yaml(msg.delete_dataset_path, out);
    out << "\n";
  }

  // member: output_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_path: ";
    rosidl_generator_traits::value_to_yaml(msg.output_path, out);
    out << "\n";
  }

  // member: delete_episode_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.delete_episode_num.size() == 0) {
      out << "delete_episode_num: []\n";
    } else {
      out << "delete_episode_num:\n";
      for (auto item : msg.delete_episode_num) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: upload_huggingface
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "upload_huggingface: ";
    rosidl_generator_traits::value_to_yaml(msg.upload_huggingface, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EditDataset_Request & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::EditDataset_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::EditDataset_Request & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::EditDataset_Request>()
{
  return "physical_ai_interfaces::srv::EditDataset_Request";
}

template<>
inline const char * name<physical_ai_interfaces::srv::EditDataset_Request>()
{
  return "physical_ai_interfaces/srv/EditDataset_Request";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::EditDataset_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::EditDataset_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::srv::EditDataset_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace physical_ai_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const EditDataset_Response & msg,
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
  const EditDataset_Response & msg,
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

inline std::string to_yaml(const EditDataset_Response & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::EditDataset_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::EditDataset_Response & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::EditDataset_Response>()
{
  return "physical_ai_interfaces::srv::EditDataset_Response";
}

template<>
inline const char * name<physical_ai_interfaces::srv::EditDataset_Response>()
{
  return "physical_ai_interfaces/srv/EditDataset_Response";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::EditDataset_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::EditDataset_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::srv::EditDataset_Response>
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
  const EditDataset_Event & msg,
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
  const EditDataset_Event & msg,
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

inline std::string to_yaml(const EditDataset_Event & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::srv::EditDataset_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::srv::EditDataset_Event & msg)
{
  return physical_ai_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::srv::EditDataset_Event>()
{
  return "physical_ai_interfaces::srv::EditDataset_Event";
}

template<>
inline const char * name<physical_ai_interfaces::srv::EditDataset_Event>()
{
  return "physical_ai_interfaces/srv/EditDataset_Event";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::EditDataset_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::EditDataset_Event>
  : std::integral_constant<bool, has_bounded_size<physical_ai_interfaces::srv::EditDataset_Request>::value && has_bounded_size<physical_ai_interfaces::srv::EditDataset_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<physical_ai_interfaces::srv::EditDataset_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<physical_ai_interfaces::srv::EditDataset>()
{
  return "physical_ai_interfaces::srv::EditDataset";
}

template<>
inline const char * name<physical_ai_interfaces::srv::EditDataset>()
{
  return "physical_ai_interfaces/srv/EditDataset";
}

template<>
struct has_fixed_size<physical_ai_interfaces::srv::EditDataset>
  : std::integral_constant<
    bool,
    has_fixed_size<physical_ai_interfaces::srv::EditDataset_Request>::value &&
    has_fixed_size<physical_ai_interfaces::srv::EditDataset_Response>::value
  >
{
};

template<>
struct has_bounded_size<physical_ai_interfaces::srv::EditDataset>
  : std::integral_constant<
    bool,
    has_bounded_size<physical_ai_interfaces::srv::EditDataset_Request>::value &&
    has_bounded_size<physical_ai_interfaces::srv::EditDataset_Response>::value
  >
{
};

template<>
struct is_service<physical_ai_interfaces::srv::EditDataset>
  : std::true_type
{
};

template<>
struct is_service_request<physical_ai_interfaces::srv::EditDataset_Request>
  : std::true_type
{
};

template<>
struct is_service_response<physical_ai_interfaces::srv::EditDataset_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__EDIT_DATASET__TRAITS_HPP_
