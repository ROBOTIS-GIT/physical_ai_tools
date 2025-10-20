// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from physical_ai_interfaces:msg/TaskStatus.idl
// generated code does not contain a copyright notice
#include "physical_ai_interfaces/msg/detail/task_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `task_info`
#include "physical_ai_interfaces/msg/detail/task_info__functions.h"
// Member `robot_type`
// Member `current_task_instruction`
// Member `error`
#include "rosidl_runtime_c/string_functions.h"

bool
physical_ai_interfaces__msg__TaskStatus__init(physical_ai_interfaces__msg__TaskStatus * msg)
{
  if (!msg) {
    return false;
  }
  // task_info
  if (!physical_ai_interfaces__msg__TaskInfo__init(&msg->task_info)) {
    physical_ai_interfaces__msg__TaskStatus__fini(msg);
    return false;
  }
  // robot_type
  if (!rosidl_runtime_c__String__init(&msg->robot_type)) {
    physical_ai_interfaces__msg__TaskStatus__fini(msg);
    return false;
  }
  // phase
  // total_time
  // proceed_time
  // current_episode_number
  // current_scenario_number
  // current_task_instruction
  if (!rosidl_runtime_c__String__init(&msg->current_task_instruction)) {
    physical_ai_interfaces__msg__TaskStatus__fini(msg);
    return false;
  }
  // encoding_progress
  // used_storage_size
  // total_storage_size
  // used_cpu
  // used_ram_size
  // total_ram_size
  // error
  if (!rosidl_runtime_c__String__init(&msg->error)) {
    physical_ai_interfaces__msg__TaskStatus__fini(msg);
    return false;
  }
  return true;
}

void
physical_ai_interfaces__msg__TaskStatus__fini(physical_ai_interfaces__msg__TaskStatus * msg)
{
  if (!msg) {
    return;
  }
  // task_info
  physical_ai_interfaces__msg__TaskInfo__fini(&msg->task_info);
  // robot_type
  rosidl_runtime_c__String__fini(&msg->robot_type);
  // phase
  // total_time
  // proceed_time
  // current_episode_number
  // current_scenario_number
  // current_task_instruction
  rosidl_runtime_c__String__fini(&msg->current_task_instruction);
  // encoding_progress
  // used_storage_size
  // total_storage_size
  // used_cpu
  // used_ram_size
  // total_ram_size
  // error
  rosidl_runtime_c__String__fini(&msg->error);
}

bool
physical_ai_interfaces__msg__TaskStatus__are_equal(const physical_ai_interfaces__msg__TaskStatus * lhs, const physical_ai_interfaces__msg__TaskStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // task_info
  if (!physical_ai_interfaces__msg__TaskInfo__are_equal(
      &(lhs->task_info), &(rhs->task_info)))
  {
    return false;
  }
  // robot_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_type), &(rhs->robot_type)))
  {
    return false;
  }
  // phase
  if (lhs->phase != rhs->phase) {
    return false;
  }
  // total_time
  if (lhs->total_time != rhs->total_time) {
    return false;
  }
  // proceed_time
  if (lhs->proceed_time != rhs->proceed_time) {
    return false;
  }
  // current_episode_number
  if (lhs->current_episode_number != rhs->current_episode_number) {
    return false;
  }
  // current_scenario_number
  if (lhs->current_scenario_number != rhs->current_scenario_number) {
    return false;
  }
  // current_task_instruction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_task_instruction), &(rhs->current_task_instruction)))
  {
    return false;
  }
  // encoding_progress
  if (lhs->encoding_progress != rhs->encoding_progress) {
    return false;
  }
  // used_storage_size
  if (lhs->used_storage_size != rhs->used_storage_size) {
    return false;
  }
  // total_storage_size
  if (lhs->total_storage_size != rhs->total_storage_size) {
    return false;
  }
  // used_cpu
  if (lhs->used_cpu != rhs->used_cpu) {
    return false;
  }
  // used_ram_size
  if (lhs->used_ram_size != rhs->used_ram_size) {
    return false;
  }
  // total_ram_size
  if (lhs->total_ram_size != rhs->total_ram_size) {
    return false;
  }
  // error
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error), &(rhs->error)))
  {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__msg__TaskStatus__copy(
  const physical_ai_interfaces__msg__TaskStatus * input,
  physical_ai_interfaces__msg__TaskStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // task_info
  if (!physical_ai_interfaces__msg__TaskInfo__copy(
      &(input->task_info), &(output->task_info)))
  {
    return false;
  }
  // robot_type
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_type), &(output->robot_type)))
  {
    return false;
  }
  // phase
  output->phase = input->phase;
  // total_time
  output->total_time = input->total_time;
  // proceed_time
  output->proceed_time = input->proceed_time;
  // current_episode_number
  output->current_episode_number = input->current_episode_number;
  // current_scenario_number
  output->current_scenario_number = input->current_scenario_number;
  // current_task_instruction
  if (!rosidl_runtime_c__String__copy(
      &(input->current_task_instruction), &(output->current_task_instruction)))
  {
    return false;
  }
  // encoding_progress
  output->encoding_progress = input->encoding_progress;
  // used_storage_size
  output->used_storage_size = input->used_storage_size;
  // total_storage_size
  output->total_storage_size = input->total_storage_size;
  // used_cpu
  output->used_cpu = input->used_cpu;
  // used_ram_size
  output->used_ram_size = input->used_ram_size;
  // total_ram_size
  output->total_ram_size = input->total_ram_size;
  // error
  if (!rosidl_runtime_c__String__copy(
      &(input->error), &(output->error)))
  {
    return false;
  }
  return true;
}

physical_ai_interfaces__msg__TaskStatus *
physical_ai_interfaces__msg__TaskStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TaskStatus * msg = (physical_ai_interfaces__msg__TaskStatus *)allocator.allocate(sizeof(physical_ai_interfaces__msg__TaskStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__msg__TaskStatus));
  bool success = physical_ai_interfaces__msg__TaskStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__msg__TaskStatus__destroy(physical_ai_interfaces__msg__TaskStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__msg__TaskStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__msg__TaskStatus__Sequence__init(physical_ai_interfaces__msg__TaskStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TaskStatus * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__msg__TaskStatus *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__msg__TaskStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__msg__TaskStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__msg__TaskStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
physical_ai_interfaces__msg__TaskStatus__Sequence__fini(physical_ai_interfaces__msg__TaskStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      physical_ai_interfaces__msg__TaskStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

physical_ai_interfaces__msg__TaskStatus__Sequence *
physical_ai_interfaces__msg__TaskStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TaskStatus__Sequence * array = (physical_ai_interfaces__msg__TaskStatus__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__msg__TaskStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__msg__TaskStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__msg__TaskStatus__Sequence__destroy(physical_ai_interfaces__msg__TaskStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__msg__TaskStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__msg__TaskStatus__Sequence__are_equal(const physical_ai_interfaces__msg__TaskStatus__Sequence * lhs, const physical_ai_interfaces__msg__TaskStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__msg__TaskStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__msg__TaskStatus__Sequence__copy(
  const physical_ai_interfaces__msg__TaskStatus__Sequence * input,
  physical_ai_interfaces__msg__TaskStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__msg__TaskStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__msg__TaskStatus * data =
      (physical_ai_interfaces__msg__TaskStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__msg__TaskStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__msg__TaskStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__msg__TaskStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
