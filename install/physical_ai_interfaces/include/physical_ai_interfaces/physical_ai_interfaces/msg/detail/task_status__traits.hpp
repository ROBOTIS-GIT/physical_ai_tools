// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:msg/TaskStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_status.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/msg/detail/task_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'task_info'
#include "physical_ai_interfaces/msg/detail/task_info__traits.hpp"

namespace physical_ai_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TaskStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_info
  {
    out << "task_info: ";
    to_flow_style_yaml(msg.task_info, out);
    out << ", ";
  }

  // member: robot_type
  {
    out << "robot_type: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_type, out);
    out << ", ";
  }

  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: total_time
  {
    out << "total_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_time, out);
    out << ", ";
  }

  // member: proceed_time
  {
    out << "proceed_time: ";
    rosidl_generator_traits::value_to_yaml(msg.proceed_time, out);
    out << ", ";
  }

  // member: current_episode_number
  {
    out << "current_episode_number: ";
    rosidl_generator_traits::value_to_yaml(msg.current_episode_number, out);
    out << ", ";
  }

  // member: current_scenario_number
  {
    out << "current_scenario_number: ";
    rosidl_generator_traits::value_to_yaml(msg.current_scenario_number, out);
    out << ", ";
  }

  // member: current_task_instruction
  {
    out << "current_task_instruction: ";
    rosidl_generator_traits::value_to_yaml(msg.current_task_instruction, out);
    out << ", ";
  }

  // member: encoding_progress
  {
    out << "encoding_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.encoding_progress, out);
    out << ", ";
  }

  // member: used_storage_size
  {
    out << "used_storage_size: ";
    rosidl_generator_traits::value_to_yaml(msg.used_storage_size, out);
    out << ", ";
  }

  // member: total_storage_size
  {
    out << "total_storage_size: ";
    rosidl_generator_traits::value_to_yaml(msg.total_storage_size, out);
    out << ", ";
  }

  // member: used_cpu
  {
    out << "used_cpu: ";
    rosidl_generator_traits::value_to_yaml(msg.used_cpu, out);
    out << ", ";
  }

  // member: used_ram_size
  {
    out << "used_ram_size: ";
    rosidl_generator_traits::value_to_yaml(msg.used_ram_size, out);
    out << ", ";
  }

  // member: total_ram_size
  {
    out << "total_ram_size: ";
    rosidl_generator_traits::value_to_yaml(msg.total_ram_size, out);
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
  const TaskStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_info:\n";
    to_block_style_yaml(msg.task_info, out, indentation + 2);
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

  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }

  // member: total_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_time, out);
    out << "\n";
  }

  // member: proceed_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "proceed_time: ";
    rosidl_generator_traits::value_to_yaml(msg.proceed_time, out);
    out << "\n";
  }

  // member: current_episode_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_episode_number: ";
    rosidl_generator_traits::value_to_yaml(msg.current_episode_number, out);
    out << "\n";
  }

  // member: current_scenario_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_scenario_number: ";
    rosidl_generator_traits::value_to_yaml(msg.current_scenario_number, out);
    out << "\n";
  }

  // member: current_task_instruction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_task_instruction: ";
    rosidl_generator_traits::value_to_yaml(msg.current_task_instruction, out);
    out << "\n";
  }

  // member: encoding_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "encoding_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.encoding_progress, out);
    out << "\n";
  }

  // member: used_storage_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "used_storage_size: ";
    rosidl_generator_traits::value_to_yaml(msg.used_storage_size, out);
    out << "\n";
  }

  // member: total_storage_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_storage_size: ";
    rosidl_generator_traits::value_to_yaml(msg.total_storage_size, out);
    out << "\n";
  }

  // member: used_cpu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "used_cpu: ";
    rosidl_generator_traits::value_to_yaml(msg.used_cpu, out);
    out << "\n";
  }

  // member: used_ram_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "used_ram_size: ";
    rosidl_generator_traits::value_to_yaml(msg.used_ram_size, out);
    out << "\n";
  }

  // member: total_ram_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_ram_size: ";
    rosidl_generator_traits::value_to_yaml(msg.total_ram_size, out);
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

inline std::string to_yaml(const TaskStatus & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::msg::TaskStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::msg::TaskStatus & msg)
{
  return physical_ai_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::msg::TaskStatus>()
{
  return "physical_ai_interfaces::msg::TaskStatus";
}

template<>
inline const char * name<physical_ai_interfaces::msg::TaskStatus>()
{
  return "physical_ai_interfaces/msg/TaskStatus";
}

template<>
struct has_fixed_size<physical_ai_interfaces::msg::TaskStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::msg::TaskStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::msg::TaskStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__TRAITS_HPP_
