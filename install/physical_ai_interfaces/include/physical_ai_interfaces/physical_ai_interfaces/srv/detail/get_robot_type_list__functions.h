// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from physical_ai_interfaces:srv/GetRobotTypeList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/srv/get_robot_type_list.h"


#ifndef PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_ROBOT_TYPE_LIST__FUNCTIONS_H_
#define PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_ROBOT_TYPE_LIST__FUNCTIONS_H_

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

#include "physical_ai_interfaces/srv/detail/get_robot_type_list__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetRobotTypeList__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetRobotTypeList__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetRobotTypeList__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/GetRobotTypeList message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * physical_ai_interfaces__srv__GetRobotTypeList_Request
 * )) before or use
 * physical_ai_interfaces__srv__GetRobotTypeList_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Request__init(physical_ai_interfaces__srv__GetRobotTypeList_Request * msg);

/// Finalize srv/GetRobotTypeList message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Request__fini(physical_ai_interfaces__srv__GetRobotTypeList_Request * msg);

/// Create srv/GetRobotTypeList message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
physical_ai_interfaces__srv__GetRobotTypeList_Request *
physical_ai_interfaces__srv__GetRobotTypeList_Request__create(void);

/// Destroy srv/GetRobotTypeList message.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Request__destroy(physical_ai_interfaces__srv__GetRobotTypeList_Request * msg);

/// Check for srv/GetRobotTypeList message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Request__are_equal(const physical_ai_interfaces__srv__GetRobotTypeList_Request * lhs, const physical_ai_interfaces__srv__GetRobotTypeList_Request * rhs);

/// Copy a srv/GetRobotTypeList message.
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
physical_ai_interfaces__srv__GetRobotTypeList_Request__copy(
  const physical_ai_interfaces__srv__GetRobotTypeList_Request * input,
  physical_ai_interfaces__srv__GetRobotTypeList_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetRobotTypeList_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetRobotTypeList_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetRobotTypeList_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetRobotTypeList messages.
/**
 * It allocates the memory for the number of elements and calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence__init(physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetRobotTypeList messages.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence__fini(physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence * array);

/// Create array of srv/GetRobotTypeList messages.
/**
 * It allocates the memory for the array and calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetRobotTypeList messages.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence__destroy(physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence * array);

/// Check for srv/GetRobotTypeList message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence__are_equal(const physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence * lhs, const physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence * rhs);

/// Copy an array of srv/GetRobotTypeList messages.
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
physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence__copy(
  const physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence * input,
  physical_ai_interfaces__srv__GetRobotTypeList_Request__Sequence * output);

/// Initialize srv/GetRobotTypeList message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * physical_ai_interfaces__srv__GetRobotTypeList_Response
 * )) before or use
 * physical_ai_interfaces__srv__GetRobotTypeList_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Response__init(physical_ai_interfaces__srv__GetRobotTypeList_Response * msg);

/// Finalize srv/GetRobotTypeList message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Response__fini(physical_ai_interfaces__srv__GetRobotTypeList_Response * msg);

/// Create srv/GetRobotTypeList message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
physical_ai_interfaces__srv__GetRobotTypeList_Response *
physical_ai_interfaces__srv__GetRobotTypeList_Response__create(void);

/// Destroy srv/GetRobotTypeList message.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Response__destroy(physical_ai_interfaces__srv__GetRobotTypeList_Response * msg);

/// Check for srv/GetRobotTypeList message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Response__are_equal(const physical_ai_interfaces__srv__GetRobotTypeList_Response * lhs, const physical_ai_interfaces__srv__GetRobotTypeList_Response * rhs);

/// Copy a srv/GetRobotTypeList message.
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
physical_ai_interfaces__srv__GetRobotTypeList_Response__copy(
  const physical_ai_interfaces__srv__GetRobotTypeList_Response * input,
  physical_ai_interfaces__srv__GetRobotTypeList_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetRobotTypeList_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetRobotTypeList_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetRobotTypeList_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetRobotTypeList messages.
/**
 * It allocates the memory for the number of elements and calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence__init(physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetRobotTypeList messages.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence__fini(physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence * array);

/// Create array of srv/GetRobotTypeList messages.
/**
 * It allocates the memory for the array and calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetRobotTypeList messages.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence__destroy(physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence * array);

/// Check for srv/GetRobotTypeList message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence__are_equal(const physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence * lhs, const physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence * rhs);

/// Copy an array of srv/GetRobotTypeList messages.
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
physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence__copy(
  const physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence * input,
  physical_ai_interfaces__srv__GetRobotTypeList_Response__Sequence * output);

/// Initialize srv/GetRobotTypeList message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * physical_ai_interfaces__srv__GetRobotTypeList_Event
 * )) before or use
 * physical_ai_interfaces__srv__GetRobotTypeList_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Event__init(physical_ai_interfaces__srv__GetRobotTypeList_Event * msg);

/// Finalize srv/GetRobotTypeList message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Event__fini(physical_ai_interfaces__srv__GetRobotTypeList_Event * msg);

/// Create srv/GetRobotTypeList message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
physical_ai_interfaces__srv__GetRobotTypeList_Event *
physical_ai_interfaces__srv__GetRobotTypeList_Event__create(void);

/// Destroy srv/GetRobotTypeList message.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Event__destroy(physical_ai_interfaces__srv__GetRobotTypeList_Event * msg);

/// Check for srv/GetRobotTypeList message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Event__are_equal(const physical_ai_interfaces__srv__GetRobotTypeList_Event * lhs, const physical_ai_interfaces__srv__GetRobotTypeList_Event * rhs);

/// Copy a srv/GetRobotTypeList message.
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
physical_ai_interfaces__srv__GetRobotTypeList_Event__copy(
  const physical_ai_interfaces__srv__GetRobotTypeList_Event * input,
  physical_ai_interfaces__srv__GetRobotTypeList_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_type_hash_t *
physical_ai_interfaces__srv__GetRobotTypeList_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
physical_ai_interfaces__srv__GetRobotTypeList_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource *
physical_ai_interfaces__srv__GetRobotTypeList_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetRobotTypeList messages.
/**
 * It allocates the memory for the number of elements and calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence__init(physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence * array, size_t size);

/// Finalize array of srv/GetRobotTypeList messages.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence__fini(physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence * array);

/// Create array of srv/GetRobotTypeList messages.
/**
 * It allocates the memory for the array and calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence *
physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence__create(size_t size);

/// Destroy array of srv/GetRobotTypeList messages.
/**
 * It calls
 * physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
void
physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence__destroy(physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence * array);

/// Check for srv/GetRobotTypeList message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_physical_ai_interfaces
bool
physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence__are_equal(const physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence * lhs, const physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence * rhs);

/// Copy an array of srv/GetRobotTypeList messages.
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
physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence__copy(
  const physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence * input,
  physical_ai_interfaces__srv__GetRobotTypeList_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__SRV__DETAIL__GET_ROBOT_TYPE_LIST__FUNCTIONS_H_
