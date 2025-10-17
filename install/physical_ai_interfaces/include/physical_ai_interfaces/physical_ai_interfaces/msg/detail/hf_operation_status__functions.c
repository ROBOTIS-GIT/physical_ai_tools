// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from physical_ai_interfaces:msg/HFOperationStatus.idl
// generated code does not contain a copyright notice
#include "physical_ai_interfaces/msg/detail/hf_operation_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `operation`
// Member `status`
// Member `local_path`
// Member `repo_id`
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
physical_ai_interfaces__msg__HFOperationStatus__init(physical_ai_interfaces__msg__HFOperationStatus * msg)
{
  if (!msg) {
    return false;
  }
  // operation
  if (!rosidl_runtime_c__String__init(&msg->operation)) {
    physical_ai_interfaces__msg__HFOperationStatus__fini(msg);
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    physical_ai_interfaces__msg__HFOperationStatus__fini(msg);
    return false;
  }
  // local_path
  if (!rosidl_runtime_c__String__init(&msg->local_path)) {
    physical_ai_interfaces__msg__HFOperationStatus__fini(msg);
    return false;
  }
  // repo_id
  if (!rosidl_runtime_c__String__init(&msg->repo_id)) {
    physical_ai_interfaces__msg__HFOperationStatus__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    physical_ai_interfaces__msg__HFOperationStatus__fini(msg);
    return false;
  }
  // progress_current
  // progress_total
  // progress_percentage
  return true;
}

void
physical_ai_interfaces__msg__HFOperationStatus__fini(physical_ai_interfaces__msg__HFOperationStatus * msg)
{
  if (!msg) {
    return;
  }
  // operation
  rosidl_runtime_c__String__fini(&msg->operation);
  // status
  rosidl_runtime_c__String__fini(&msg->status);
  // local_path
  rosidl_runtime_c__String__fini(&msg->local_path);
  // repo_id
  rosidl_runtime_c__String__fini(&msg->repo_id);
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // progress_current
  // progress_total
  // progress_percentage
}

bool
physical_ai_interfaces__msg__HFOperationStatus__are_equal(const physical_ai_interfaces__msg__HFOperationStatus * lhs, const physical_ai_interfaces__msg__HFOperationStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // operation
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->operation), &(rhs->operation)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  // local_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->local_path), &(rhs->local_path)))
  {
    return false;
  }
  // repo_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->repo_id), &(rhs->repo_id)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // progress_current
  if (lhs->progress_current != rhs->progress_current) {
    return false;
  }
  // progress_total
  if (lhs->progress_total != rhs->progress_total) {
    return false;
  }
  // progress_percentage
  if (lhs->progress_percentage != rhs->progress_percentage) {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__msg__HFOperationStatus__copy(
  const physical_ai_interfaces__msg__HFOperationStatus * input,
  physical_ai_interfaces__msg__HFOperationStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // operation
  if (!rosidl_runtime_c__String__copy(
      &(input->operation), &(output->operation)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  // local_path
  if (!rosidl_runtime_c__String__copy(
      &(input->local_path), &(output->local_path)))
  {
    return false;
  }
  // repo_id
  if (!rosidl_runtime_c__String__copy(
      &(input->repo_id), &(output->repo_id)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // progress_current
  output->progress_current = input->progress_current;
  // progress_total
  output->progress_total = input->progress_total;
  // progress_percentage
  output->progress_percentage = input->progress_percentage;
  return true;
}

physical_ai_interfaces__msg__HFOperationStatus *
physical_ai_interfaces__msg__HFOperationStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__HFOperationStatus * msg = (physical_ai_interfaces__msg__HFOperationStatus *)allocator.allocate(sizeof(physical_ai_interfaces__msg__HFOperationStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__msg__HFOperationStatus));
  bool success = physical_ai_interfaces__msg__HFOperationStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__msg__HFOperationStatus__destroy(physical_ai_interfaces__msg__HFOperationStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__msg__HFOperationStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__msg__HFOperationStatus__Sequence__init(physical_ai_interfaces__msg__HFOperationStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__HFOperationStatus * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__msg__HFOperationStatus *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__msg__HFOperationStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__msg__HFOperationStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__msg__HFOperationStatus__fini(&data[i - 1]);
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
physical_ai_interfaces__msg__HFOperationStatus__Sequence__fini(physical_ai_interfaces__msg__HFOperationStatus__Sequence * array)
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
      physical_ai_interfaces__msg__HFOperationStatus__fini(&array->data[i]);
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

physical_ai_interfaces__msg__HFOperationStatus__Sequence *
physical_ai_interfaces__msg__HFOperationStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__HFOperationStatus__Sequence * array = (physical_ai_interfaces__msg__HFOperationStatus__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__msg__HFOperationStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__msg__HFOperationStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__msg__HFOperationStatus__Sequence__destroy(physical_ai_interfaces__msg__HFOperationStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__msg__HFOperationStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__msg__HFOperationStatus__Sequence__are_equal(const physical_ai_interfaces__msg__HFOperationStatus__Sequence * lhs, const physical_ai_interfaces__msg__HFOperationStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__msg__HFOperationStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__msg__HFOperationStatus__Sequence__copy(
  const physical_ai_interfaces__msg__HFOperationStatus__Sequence * input,
  physical_ai_interfaces__msg__HFOperationStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__msg__HFOperationStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__msg__HFOperationStatus * data =
      (physical_ai_interfaces__msg__HFOperationStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__msg__HFOperationStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__msg__HFOperationStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__msg__HFOperationStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
