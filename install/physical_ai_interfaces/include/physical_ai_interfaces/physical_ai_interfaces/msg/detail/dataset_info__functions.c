// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from physical_ai_interfaces:msg/DatasetInfo.idl
// generated code does not contain a copyright notice
#include "physical_ai_interfaces/msg/detail/dataset_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `codebase_version`
// Member `robot_type`
#include "rosidl_runtime_c/string_functions.h"

bool
physical_ai_interfaces__msg__DatasetInfo__init(physical_ai_interfaces__msg__DatasetInfo * msg)
{
  if (!msg) {
    return false;
  }
  // codebase_version
  if (!rosidl_runtime_c__String__init(&msg->codebase_version)) {
    physical_ai_interfaces__msg__DatasetInfo__fini(msg);
    return false;
  }
  // robot_type
  if (!rosidl_runtime_c__String__init(&msg->robot_type)) {
    physical_ai_interfaces__msg__DatasetInfo__fini(msg);
    return false;
  }
  // total_episodes
  // total_tasks
  // fps
  return true;
}

void
physical_ai_interfaces__msg__DatasetInfo__fini(physical_ai_interfaces__msg__DatasetInfo * msg)
{
  if (!msg) {
    return;
  }
  // codebase_version
  rosidl_runtime_c__String__fini(&msg->codebase_version);
  // robot_type
  rosidl_runtime_c__String__fini(&msg->robot_type);
  // total_episodes
  // total_tasks
  // fps
}

bool
physical_ai_interfaces__msg__DatasetInfo__are_equal(const physical_ai_interfaces__msg__DatasetInfo * lhs, const physical_ai_interfaces__msg__DatasetInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // codebase_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->codebase_version), &(rhs->codebase_version)))
  {
    return false;
  }
  // robot_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_type), &(rhs->robot_type)))
  {
    return false;
  }
  // total_episodes
  if (lhs->total_episodes != rhs->total_episodes) {
    return false;
  }
  // total_tasks
  if (lhs->total_tasks != rhs->total_tasks) {
    return false;
  }
  // fps
  if (lhs->fps != rhs->fps) {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__msg__DatasetInfo__copy(
  const physical_ai_interfaces__msg__DatasetInfo * input,
  physical_ai_interfaces__msg__DatasetInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // codebase_version
  if (!rosidl_runtime_c__String__copy(
      &(input->codebase_version), &(output->codebase_version)))
  {
    return false;
  }
  // robot_type
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_type), &(output->robot_type)))
  {
    return false;
  }
  // total_episodes
  output->total_episodes = input->total_episodes;
  // total_tasks
  output->total_tasks = input->total_tasks;
  // fps
  output->fps = input->fps;
  return true;
}

physical_ai_interfaces__msg__DatasetInfo *
physical_ai_interfaces__msg__DatasetInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__DatasetInfo * msg = (physical_ai_interfaces__msg__DatasetInfo *)allocator.allocate(sizeof(physical_ai_interfaces__msg__DatasetInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__msg__DatasetInfo));
  bool success = physical_ai_interfaces__msg__DatasetInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__msg__DatasetInfo__destroy(physical_ai_interfaces__msg__DatasetInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__msg__DatasetInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__msg__DatasetInfo__Sequence__init(physical_ai_interfaces__msg__DatasetInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__DatasetInfo * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__msg__DatasetInfo *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__msg__DatasetInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__msg__DatasetInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__msg__DatasetInfo__fini(&data[i - 1]);
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
physical_ai_interfaces__msg__DatasetInfo__Sequence__fini(physical_ai_interfaces__msg__DatasetInfo__Sequence * array)
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
      physical_ai_interfaces__msg__DatasetInfo__fini(&array->data[i]);
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

physical_ai_interfaces__msg__DatasetInfo__Sequence *
physical_ai_interfaces__msg__DatasetInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__DatasetInfo__Sequence * array = (physical_ai_interfaces__msg__DatasetInfo__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__msg__DatasetInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__msg__DatasetInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__msg__DatasetInfo__Sequence__destroy(physical_ai_interfaces__msg__DatasetInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__msg__DatasetInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__msg__DatasetInfo__Sequence__are_equal(const physical_ai_interfaces__msg__DatasetInfo__Sequence * lhs, const physical_ai_interfaces__msg__DatasetInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__msg__DatasetInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__msg__DatasetInfo__Sequence__copy(
  const physical_ai_interfaces__msg__DatasetInfo__Sequence * input,
  physical_ai_interfaces__msg__DatasetInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__msg__DatasetInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__msg__DatasetInfo * data =
      (physical_ai_interfaces__msg__DatasetInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__msg__DatasetInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__msg__DatasetInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__msg__DatasetInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
