// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from physical_ai_interfaces:msg/TaskInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_info.h"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__FUNCTIONS_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "physical_ai_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "physical_ai_interfaces/msg/detail/task_info__struct.h"

/// Initialize msg/TaskInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * physical_ai_interfaces__msg__TaskInfo
 * )) before or use
 * physical_ai_interfaces__msg__TaskInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__msg__TaskInfo__init(physical_ai_interfaces__msg__TaskInfo * msg);

/// Finalize msg/TaskInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__msg__TaskInfo__fini(physical_ai_interfaces__msg__TaskInfo * msg);

/// Create msg/TaskInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * physical_ai_interfaces__msg__TaskInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
physical_ai_interfaces__msg__TaskInfo *
physical_ai_interfaces__msg__TaskInfo__create(void);

/// Destroy msg/TaskInfo message.
/**
 * It calls
 * physical_ai_interfaces__msg__TaskInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__msg__TaskInfo__destroy(physical_ai_interfaces__msg__TaskInfo * msg);

/// Check for msg/TaskInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__msg__TaskInfo__are_equal(const physical_ai_interfaces__msg__TaskInfo * lhs, const physical_ai_interfaces__msg__TaskInfo * rhs);

/// Copy a msg/TaskInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__msg__TaskInfo__copy(
  const physical_ai_interfaces__msg__TaskInfo * input,
  physical_ai_interfaces__msg__TaskInfo * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__msg__TaskInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__msg__TaskInfo__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__msg__TaskInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__msg__TaskInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/TaskInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * physical_ai_interfaces__msg__TaskInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__msg__TaskInfo__Sequence__init(physical_ai_interfaces__msg__TaskInfo__Sequence * array, size_t size);

/// Finalize array of msg/TaskInfo messages.
/**
 * It calls
 * physical_ai_interfaces__msg__TaskInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__msg__TaskInfo__Sequence__fini(physical_ai_interfaces__msg__TaskInfo__Sequence * array);

/// Create array of msg/TaskInfo messages.
/**
 * It allocates the memory for the array and calls
 * physical_ai_interfaces__msg__TaskInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
physical_ai_interfaces__msg__TaskInfo__Sequence *
physical_ai_interfaces__msg__TaskInfo__Sequence__create(size_t size);

/// Destroy array of msg/TaskInfo messages.
/**
 * It calls
 * physical_ai_interfaces__msg__TaskInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__msg__TaskInfo__Sequence__destroy(physical_ai_interfaces__msg__TaskInfo__Sequence * array);

/// Check for msg/TaskInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__msg__TaskInfo__Sequence__are_equal(const physical_ai_interfaces__msg__TaskInfo__Sequence * lhs, const physical_ai_interfaces__msg__TaskInfo__Sequence * rhs);

/// Copy an array of msg/TaskInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__msg__TaskInfo__Sequence__copy(
  const physical_ai_interfaces__msg__TaskInfo__Sequence * input,
  physical_ai_interfaces__msg__TaskInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__FUNCTIONS_H_
