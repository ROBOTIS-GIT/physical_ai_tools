// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:msg/DatasetInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/dataset_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/msg/detail/dataset_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DatasetInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: codebase_version
  {
    out << "codebase_version: ";
    rosidl_generator_traits::value_to_yaml(msg.codebase_version, out);
    out << ", ";
  }

  // member: robot_type
  {
    out << "robot_type: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_type, out);
    out << ", ";
  }

  // member: total_episodes
  {
    out << "total_episodes: ";
    rosidl_generator_traits::value_to_yaml(msg.total_episodes, out);
    out << ", ";
  }

  // member: total_tasks
  {
    out << "total_tasks: ";
    rosidl_generator_traits::value_to_yaml(msg.total_tasks, out);
    out << ", ";
  }

  // member: fps
  {
    out << "fps: ";
    rosidl_generator_traits::value_to_yaml(msg.fps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DatasetInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: codebase_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "codebase_version: ";
    rosidl_generator_traits::value_to_yaml(msg.codebase_version, out);
    out << "\n";
  }

  // member: robot_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_type: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_type, out);
    out << "\n";
  }

  // member: total_episodes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_episodes: ";
    rosidl_generator_traits::value_to_yaml(msg.total_episodes, out);
    out << "\n";
  }

  // member: total_tasks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_tasks: ";
    rosidl_generator_traits::value_to_yaml(msg.total_tasks, out);
    out << "\n";
  }

  // member: fps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fps: ";
    rosidl_generator_traits::value_to_yaml(msg.fps, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DatasetInfo & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::msg::DatasetInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::msg::DatasetInfo & msg)
{
  return physical_ai_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::msg::DatasetInfo>()
{
  return "physical_ai_interfaces::msg::DatasetInfo";
}

template<>
inline const char * name<physical_ai_interfaces::msg::DatasetInfo>()
{
  return "physical_ai_interfaces/msg/DatasetInfo";
}

template<>
struct has_fixed_size<physical_ai_interfaces::msg::DatasetInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::msg::DatasetInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::msg::DatasetInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__TRAITS_HPP_
