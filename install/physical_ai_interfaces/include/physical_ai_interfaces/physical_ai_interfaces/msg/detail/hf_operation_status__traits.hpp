// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:msg/HFOperationStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/hf_operation_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/msg/detail/hf_operation_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const HFOperationStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: operation
  {
    out << "operation: ";
    rosidl_generator_traits::value_to_yaml(msg.operation, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: local_path
  {
    out << "local_path: ";
    rosidl_generator_traits::value_to_yaml(msg.local_path, out);
    out << ", ";
  }

  // member: repo_id
  {
    out << "repo_id: ";
    rosidl_generator_traits::value_to_yaml(msg.repo_id, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: progress_current
  {
    out << "progress_current: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_current, out);
    out << ", ";
  }

  // member: progress_total
  {
    out << "progress_total: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_total, out);
    out << ", ";
  }

  // member: progress_percentage
  {
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HFOperationStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: operation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation: ";
    rosidl_generator_traits::value_to_yaml(msg.operation, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: local_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_path: ";
    rosidl_generator_traits::value_to_yaml(msg.local_path, out);
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: progress_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress_current: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_current, out);
    out << "\n";
  }

  // member: progress_total
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress_total: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_total, out);
    out << "\n";
  }

  // member: progress_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HFOperationStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace physical_ai_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use physical_ai_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const physical_ai_interfaces::msg::HFOperationStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::msg::HFOperationStatus & msg)
{
  return physical_ai_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::msg::HFOperationStatus>()
{
  return "physical_ai_interfaces::msg::HFOperationStatus";
}

template<>
inline const char * name<physical_ai_interfaces::msg::HFOperationStatus>()
{
  return "physical_ai_interfaces/msg/HFOperationStatus";
}

template<>
struct has_fixed_size<physical_ai_interfaces::msg::HFOperationStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::msg::HFOperationStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::msg::HFOperationStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__TRAITS_HPP_
