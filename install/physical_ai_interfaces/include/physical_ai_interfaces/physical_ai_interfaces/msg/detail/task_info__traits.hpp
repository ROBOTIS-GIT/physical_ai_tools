// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from physical_ai_interfaces:msg/TaskInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_info.hpp"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__TRAITS_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "physical_ai_interfaces/msg/detail/task_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace physical_ai_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TaskInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_name
  {
    out << "task_name: ";
    rosidl_generator_traits::value_to_yaml(msg.task_name, out);
    out << ", ";
  }

  // member: task_type
  {
    out << "task_type: ";
    rosidl_generator_traits::value_to_yaml(msg.task_type, out);
    out << ", ";
  }

  // member: user_id
  {
    out << "user_id: ";
    rosidl_generator_traits::value_to_yaml(msg.user_id, out);
    out << ", ";
  }

  // member: task_instruction
  {
    if (msg.task_instruction.size() == 0) {
      out << "task_instruction: []";
    } else {
      out << "task_instruction: [";
      size_t pending_items = msg.task_instruction.size();
      for (auto item : msg.task_instruction) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: policy_path
  {
    out << "policy_path: ";
    rosidl_generator_traits::value_to_yaml(msg.policy_path, out);
    out << ", ";
  }

  // member: fps
  {
    out << "fps: ";
    rosidl_generator_traits::value_to_yaml(msg.fps, out);
    out << ", ";
  }

  // member: tags
  {
    if (msg.tags.size() == 0) {
      out << "tags: []";
    } else {
      out << "tags: [";
      size_t pending_items = msg.tags.size();
      for (auto item : msg.tags) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: warmup_time_s
  {
    out << "warmup_time_s: ";
    rosidl_generator_traits::value_to_yaml(msg.warmup_time_s, out);
    out << ", ";
  }

  // member: episode_time_s
  {
    out << "episode_time_s: ";
    rosidl_generator_traits::value_to_yaml(msg.episode_time_s, out);
    out << ", ";
  }

  // member: reset_time_s
  {
    out << "reset_time_s: ";
    rosidl_generator_traits::value_to_yaml(msg.reset_time_s, out);
    out << ", ";
  }

  // member: num_episodes
  {
    out << "num_episodes: ";
    rosidl_generator_traits::value_to_yaml(msg.num_episodes, out);
    out << ", ";
  }

  // member: push_to_hub
  {
    out << "push_to_hub: ";
    rosidl_generator_traits::value_to_yaml(msg.push_to_hub, out);
    out << ", ";
  }

  // member: private_mode
  {
    out << "private_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.private_mode, out);
    out << ", ";
  }

  // member: use_optimized_save_mode
  {
    out << "use_optimized_save_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.use_optimized_save_mode, out);
    out << ", ";
  }

  // member: record_inference_mode
  {
    out << "record_inference_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.record_inference_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TaskInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_name: ";
    rosidl_generator_traits::value_to_yaml(msg.task_name, out);
    out << "\n";
  }

  // member: task_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_type: ";
    rosidl_generator_traits::value_to_yaml(msg.task_type, out);
    out << "\n";
  }

  // member: user_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "user_id: ";
    rosidl_generator_traits::value_to_yaml(msg.user_id, out);
    out << "\n";
  }

  // member: task_instruction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.task_instruction.size() == 0) {
      out << "task_instruction: []\n";
    } else {
      out << "task_instruction:\n";
      for (auto item : msg.task_instruction) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: policy_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "policy_path: ";
    rosidl_generator_traits::value_to_yaml(msg.policy_path, out);
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

  // member: tags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tags.size() == 0) {
      out << "tags: []\n";
    } else {
      out << "tags:\n";
      for (auto item : msg.tags) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: warmup_time_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "warmup_time_s: ";
    rosidl_generator_traits::value_to_yaml(msg.warmup_time_s, out);
    out << "\n";
  }

  // member: episode_time_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "episode_time_s: ";
    rosidl_generator_traits::value_to_yaml(msg.episode_time_s, out);
    out << "\n";
  }

  // member: reset_time_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_time_s: ";
    rosidl_generator_traits::value_to_yaml(msg.reset_time_s, out);
    out << "\n";
  }

  // member: num_episodes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_episodes: ";
    rosidl_generator_traits::value_to_yaml(msg.num_episodes, out);
    out << "\n";
  }

  // member: push_to_hub
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "push_to_hub: ";
    rosidl_generator_traits::value_to_yaml(msg.push_to_hub, out);
    out << "\n";
  }

  // member: private_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "private_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.private_mode, out);
    out << "\n";
  }

  // member: use_optimized_save_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_optimized_save_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.use_optimized_save_mode, out);
    out << "\n";
  }

  // member: record_inference_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "record_inference_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.record_inference_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TaskInfo & msg, bool use_flow_style = false)
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
  const physical_ai_interfaces::msg::TaskInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  physical_ai_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use physical_ai_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const physical_ai_interfaces::msg::TaskInfo & msg)
{
  return physical_ai_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<physical_ai_interfaces::msg::TaskInfo>()
{
  return "physical_ai_interfaces::msg::TaskInfo";
}

template<>
inline const char * name<physical_ai_interfaces::msg::TaskInfo>()
{
  return "physical_ai_interfaces/msg/TaskInfo";
}

template<>
struct has_fixed_size<physical_ai_interfaces::msg::TaskInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<physical_ai_interfaces::msg::TaskInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<physical_ai_interfaces::msg::TaskInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__TRAITS_HPP_
