// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef PHYSICAL_AI_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_physical_ai_interfaces __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_physical_ai_interfaces __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_physical_ai_interfaces __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_physical_ai_interfaces __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_physical_ai_interfaces
    #define ROSIDL_GENERATOR_CPP_PUBLIC_physical_ai_interfaces ROSIDL_GENERATOR_CPP_EXPORT_physical_ai_interfaces
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_physical_ai_interfaces ROSIDL_GENERATOR_CPP_IMPORT_physical_ai_interfaces
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_physical_ai_interfaces __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_physical_ai_interfaces
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_physical_ai_interfaces __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_physical_ai_interfaces
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
