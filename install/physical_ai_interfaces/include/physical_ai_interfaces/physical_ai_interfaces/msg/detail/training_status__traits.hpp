// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/training_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/msg/detail/training_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'training_info'
#include "physical_ai_interfaces/msg/detail/training_info__traits.hpp"

namespace physical_ai_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrainingStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: training_info
  {
    out << "training_info: ";
    to_flow_style_yaml(msg.training_info, out);
    out << ", ";
  }

  // member: current_step
  {
    out << "current_step: ";
    rosidl_generator_traits::value_to_yaml(msg.current_step, out);
    out << ", ";
  }

  // member: current_loss
  {
    out << "current_loss: ";
    rosidl_generator_traits::value_to_yaml(msg.current_loss, out);
    out << ", ";
  }

  // member: is_training
  {
    out << "is_training: ";
    rosidl_generator_traits::value_to_yaml(msg.is_training, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrainingStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: training_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "training_info:\n";
    to_block_style_yaml(msg.training_info, out, indentation + 2);
  }

  // member: current_step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_step: ";
    rosidl_generator_traits::value_to_yaml(msg.current_step, out);
    out << "\n";
  }

  // member: current_loss
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_loss: ";
    rosidl_generator_traits::value_to_yaml(msg.current_loss, out);
    out << "\n";
  }

  // member: is_training
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_training: ";
    rosidl_generator_traits::value_to_yaml(msg.is_training, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrainingStatus & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::msg::TrainingStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::msg::TrainingStatus & msg)
{
  return physical_ai_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::msg::TrainingStatus>()
{
  return "physical_ai_interfaces::msg::TrainingStatus";
}

template<>
inline const char * name<physical_ai_interfaces::msg::TrainingStatus>()
{
  return "physical_ai_interfaces/msg/TrainingStatus";
}

template<>
struct has_fixed_size<physical_ai_interfaces::msg::TrainingStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::msg::TrainingStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::msg::TrainingStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__TRAITS_HPP_
