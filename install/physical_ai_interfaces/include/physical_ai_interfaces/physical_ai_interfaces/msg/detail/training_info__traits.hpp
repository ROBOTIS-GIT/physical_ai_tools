// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:msg/TrainingInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/training_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/msg/detail/training_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrainingInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: dataset
  {
    out << "dataset: ";
    rosidl_generator_traits::value_to_yaml(msg.dataset, out);
    out << ", ";
  }

  // member: policy_type
  {
    out << "policy_type: ";
    rosidl_generator_traits::value_to_yaml(msg.policy_type, out);
    out << ", ";
  }

  // member: output_folder_name
  {
    out << "output_folder_name: ";
    rosidl_generator_traits::value_to_yaml(msg.output_folder_name, out);
    out << ", ";
  }

  // member: policy_device
  {
    out << "policy_device: ";
    rosidl_generator_traits::value_to_yaml(msg.policy_device, out);
    out << ", ";
  }

  // member: seed
  {
    out << "seed: ";
    rosidl_generator_traits::value_to_yaml(msg.seed, out);
    out << ", ";
  }

  // member: num_workers
  {
    out << "num_workers: ";
    rosidl_generator_traits::value_to_yaml(msg.num_workers, out);
    out << ", ";
  }

  // member: batch_size
  {
    out << "batch_size: ";
    rosidl_generator_traits::value_to_yaml(msg.batch_size, out);
    out << ", ";
  }

  // member: steps
  {
    out << "steps: ";
    rosidl_generator_traits::value_to_yaml(msg.steps, out);
    out << ", ";
  }

  // member: eval_freq
  {
    out << "eval_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.eval_freq, out);
    out << ", ";
  }

  // member: log_freq
  {
    out << "log_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.log_freq, out);
    out << ", ";
  }

  // member: save_freq
  {
    out << "save_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.save_freq, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrainingInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dataset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dataset: ";
    rosidl_generator_traits::value_to_yaml(msg.dataset, out);
    out << "\n";
  }

  // member: policy_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "policy_type: ";
    rosidl_generator_traits::value_to_yaml(msg.policy_type, out);
    out << "\n";
  }

  // member: output_folder_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_folder_name: ";
    rosidl_generator_traits::value_to_yaml(msg.output_folder_name, out);
    out << "\n";
  }

  // member: policy_device
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "policy_device: ";
    rosidl_generator_traits::value_to_yaml(msg.policy_device, out);
    out << "\n";
  }

  // member: seed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seed: ";
    rosidl_generator_traits::value_to_yaml(msg.seed, out);
    out << "\n";
  }

  // member: num_workers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_workers: ";
    rosidl_generator_traits::value_to_yaml(msg.num_workers, out);
    out << "\n";
  }

  // member: batch_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "batch_size: ";
    rosidl_generator_traits::value_to_yaml(msg.batch_size, out);
    out << "\n";
  }

  // member: steps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steps: ";
    rosidl_generator_traits::value_to_yaml(msg.steps, out);
    out << "\n";
  }

  // member: eval_freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eval_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.eval_freq, out);
    out << "\n";
  }

  // member: log_freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "log_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.log_freq, out);
    out << "\n";
  }

  // member: save_freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.save_freq, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrainingInfo & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::msg::TrainingInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::msg::TrainingInfo & msg)
{
  return physical_ai_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::msg::TrainingInfo>()
{
  return "physical_ai_interfaces::msg::TrainingInfo";
}

template<>
inline const char * name<physical_ai_interfaces::msg::TrainingInfo>()
{
  return "physical_ai_interfaces/msg/TrainingInfo";
}

template<>
struct has_fixed_size<physical_ai_interfaces::msg::TrainingInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::msg::TrainingInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::msg::TrainingInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__TRAITS_HPP_
