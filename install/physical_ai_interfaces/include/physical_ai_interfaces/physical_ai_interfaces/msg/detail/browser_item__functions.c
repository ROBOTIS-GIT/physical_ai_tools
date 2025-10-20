// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from physical_ai_interfaces:msg/BrowserItem.idl
// generated code does not contain a copyright notice
#include "physical_ai_interfaces/msg/detail/browser_item__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
// Member `full_path`
// Member `modified_time`
#include "rosidl_runtime_c/string_functions.h"

bool
physical_ai_interfaces__msg__BrowserItem__init(physical_ai_interfaces__msg__BrowserItem * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    physical_ai_interfaces__msg__BrowserItem__fini(msg);
    return false;
  }
  // full_path
  if (!rosidl_runtime_c__String__init(&msg->full_path)) {
    physical_ai_interfaces__msg__BrowserItem__fini(msg);
    return false;
  }
  // is_directory
  // size
  // modified_time
  if (!rosidl_runtime_c__String__init(&msg->modified_time)) {
    physical_ai_interfaces__msg__BrowserItem__fini(msg);
    return false;
  }
  // has_target_file
  return true;
}

void
physical_ai_interfaces__msg__BrowserItem__fini(physical_ai_interfaces__msg__BrowserItem * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // full_path
  rosidl_runtime_c__String__fini(&msg->full_path);
  // is_directory
  // size
  // modified_time
  rosidl_runtime_c__String__fini(&msg->modified_time);
  // has_target_file
}

bool
physical_ai_interfaces__msg__BrowserItem__are_equal(const physical_ai_interfaces__msg__BrowserItem * lhs, const physical_ai_interfaces__msg__BrowserItem * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // full_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->full_path), &(rhs->full_path)))
  {
    return false;
  }
  // is_directory
  if (lhs->is_directory != rhs->is_directory) {
    return false;
  }
  // size
  if (lhs->size != rhs->size) {
    return false;
  }
  // modified_time
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->modified_time), &(rhs->modified_time)))
  {
    return false;
  }
  // has_target_file
  if (lhs->has_target_file != rhs->has_target_file) {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__msg__BrowserItem__copy(
  const physical_ai_interfaces__msg__BrowserItem * input,
  physical_ai_interfaces__msg__BrowserItem * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // full_path
  if (!rosidl_runtime_c__String__copy(
      &(input->full_path), &(output->full_path)))
  {
    return false;
  }
  // is_directory
  output->is_directory = input->is_directory;
  // size
  output->size = input->size;
  // modified_time
  if (!rosidl_runtime_c__String__copy(
      &(input->modified_time), &(output->modified_time)))
  {
    return false;
  }
  // has_target_file
  output->has_target_file = input->has_target_file;
  return true;
}

physical_ai_interfaces__msg__BrowserItem *
physical_ai_interfaces__msg__BrowserItem__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__BrowserItem * msg = (physical_ai_interfaces__msg__BrowserItem *)allocator.allocate(sizeof(physical_ai_interfaces__msg__BrowserItem), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__msg__BrowserItem));
  bool success = physical_ai_interfaces__msg__BrowserItem__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__msg__BrowserItem__destroy(physical_ai_interfaces__msg__BrowserItem * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__msg__BrowserItem__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__msg__BrowserItem__Sequence__init(physical_ai_interfaces__msg__BrowserItem__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__BrowserItem * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__msg__BrowserItem *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__msg__BrowserItem), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__msg__BrowserItem__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__msg__BrowserItem__fini(&data[i - 1]);
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
physical_ai_interfaces__msg__BrowserItem__Sequence__fini(physical_ai_interfaces__msg__BrowserItem__Sequence * array)
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
      physical_ai_interfaces__msg__BrowserItem__fini(&array->data[i]);
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

physical_ai_interfaces__msg__BrowserItem__Sequence *
physical_ai_interfaces__msg__BrowserItem__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__BrowserItem__Sequence * array = (physical_ai_interfaces__msg__BrowserItem__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__msg__BrowserItem__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__msg__BrowserItem__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__msg__BrowserItem__Sequence__destroy(physical_ai_interfaces__msg__BrowserItem__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__msg__BrowserItem__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__msg__BrowserItem__Sequence__are_equal(const physical_ai_interfaces__msg__BrowserItem__Sequence * lhs, const physical_ai_interfaces__msg__BrowserItem__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__msg__BrowserItem__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__msg__BrowserItem__Sequence__copy(
  const physical_ai_interfaces__msg__BrowserItem__Sequence * input,
  physical_ai_interfaces__msg__BrowserItem__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__msg__BrowserItem);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__msg__BrowserItem * data =
      (physical_ai_interfaces__msg__BrowserItem *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__msg__BrowserItem__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__msg__BrowserItem__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__msg__BrowserItem__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
