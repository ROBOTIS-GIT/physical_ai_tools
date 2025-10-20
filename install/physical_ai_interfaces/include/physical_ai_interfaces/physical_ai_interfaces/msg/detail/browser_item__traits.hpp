// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:msg/BrowserItem.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/browser_item.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/msg/detail/browser_item__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BrowserItem & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: full_path
  {
    out << "full_path: ";
    rosidl_generator_traits::value_to_yaml(msg.full_path, out);
    out << ", ";
  }

  // member: is_directory
  {
    out << "is_directory: ";
    rosidl_generator_traits::value_to_yaml(msg.is_directory, out);
    out << ", ";
  }

  // member: size
  {
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
    out << ", ";
  }

  // member: modified_time
  {
    out << "modified_time: ";
    rosidl_generator_traits::value_to_yaml(msg.modified_time, out);
    out << ", ";
  }

  // member: has_target_file
  {
    out << "has_target_file: ";
    rosidl_generator_traits::value_to_yaml(msg.has_target_file, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BrowserItem & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: full_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "full_path: ";
    rosidl_generator_traits::value_to_yaml(msg.full_path, out);
    out << "\n";
  }

  // member: is_directory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_directory: ";
    rosidl_generator_traits::value_to_yaml(msg.is_directory, out);
    out << "\n";
  }

  // member: size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
    out << "\n";
  }

  // member: modified_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "modified_time: ";
    rosidl_generator_traits::value_to_yaml(msg.modified_time, out);
    out << "\n";
  }

  // member: has_target_file
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_target_file: ";
    rosidl_generator_traits::value_to_yaml(msg.has_target_file, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BrowserItem & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::msg::BrowserItem & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::msg::BrowserItem & msg)
{
  return physical_ai_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::msg::BrowserItem>()
{
  return "physical_ai_interfaces::msg::BrowserItem";
}

template<>
inline const char * name<physical_ai_interfaces::msg::BrowserItem>()
{
  return "physical_ai_interfaces/msg/BrowserItem";
}

template<>
struct has_fixed_size<physical_ai_interfaces::msg::BrowserItem>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::msg::BrowserItem>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::msg::BrowserItem>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__BROWSER_ITEM__TRAITS_HPP_
