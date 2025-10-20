// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice
#include "physical_ai_interfaces/msg/detail/training_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `training_info`
#include "physical_ai_interfaces/msg/detail/training_info__functions.h"
// Member `error`
#include "rosidl_runtime_c/string_functions.h"

bool
physical_ai_interfaces__msg__TrainingStatus__init(physical_ai_interfaces__msg__TrainingStatus * msg)
{
  if (!msg) {
    return false;
  }
  // training_info
  if (!physical_ai_interfaces__msg__TrainingInfo__init(&msg->training_info)) {
    physical_ai_interfaces__msg__TrainingStatus__fini(msg);
    return false;
  }
  // current_step
  // current_loss
  // is_training
  // error
  if (!rosidl_runtime_c__String__init(&msg->error)) {
    physical_ai_interfaces__msg__TrainingStatus__fini(msg);
    return false;
  }
  return true;
}

void
physical_ai_interfaces__msg__TrainingStatus__fini(physical_ai_interfaces__msg__TrainingStatus * msg)
{
  if (!msg) {
    return;
  }
  // training_info
  physical_ai_interfaces__msg__TrainingInfo__fini(&msg->training_info);
  // current_step
  // current_loss
  // is_training
  // error
  rosidl_runtime_c__String__fini(&msg->error);
}

bool
physical_ai_interfaces__msg__TrainingStatus__are_equal(const physical_ai_interfaces__msg__TrainingStatus * lhs, const physical_ai_interfaces__msg__TrainingStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // training_info
  if (!physical_ai_interfaces__msg__TrainingInfo__are_equal(
      &(lhs->training_info), &(rhs->training_info)))
  {
    return false;
  }
  // current_step
  if (lhs->current_step != rhs->current_step) {
    return false;
  }
  // current_loss
  if (lhs->current_loss != rhs->current_loss) {
    return false;
  }
  // is_training
  if (lhs->is_training != rhs->is_training) {
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
physical_ai_interfaces__msg__TrainingStatus__copy(
  const physical_ai_interfaces__msg__TrainingStatus * input,
  physical_ai_interfaces__msg__TrainingStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // training_info
  if (!physical_ai_interfaces__msg__TrainingInfo__copy(
      &(input->training_info), &(output->training_info)))
  {
    return false;
  }
  // current_step
  output->current_step = input->current_step;
  // current_loss
  output->current_loss = input->current_loss;
  // is_training
  output->is_training = input->is_training;
  // error
  if (!rosidl_runtime_c__String__copy(
      &(input->error), &(output->error)))
  {
    return false;
  }
  return true;
}

physical_ai_interfaces__msg__TrainingStatus *
physical_ai_interfaces__msg__TrainingStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TrainingStatus * msg = (physical_ai_interfaces__msg__TrainingStatus *)allocator.allocate(sizeof(physical_ai_interfaces__msg__TrainingStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__msg__TrainingStatus));
  bool success = physical_ai_interfaces__msg__TrainingStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__msg__TrainingStatus__destroy(physical_ai_interfaces__msg__TrainingStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__msg__TrainingStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__msg__TrainingStatus__Sequence__init(physical_ai_interfaces__msg__TrainingStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TrainingStatus * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__msg__TrainingStatus *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__msg__TrainingStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__msg__TrainingStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__msg__TrainingStatus__fini(&data[i - 1]);
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
physical_ai_interfaces__msg__TrainingStatus__Sequence__fini(physical_ai_interfaces__msg__TrainingStatus__Sequence * array)
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
      physical_ai_interfaces__msg__TrainingStatus__fini(&array->data[i]);
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

physical_ai_interfaces__msg__TrainingStatus__Sequence *
physical_ai_interfaces__msg__TrainingStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TrainingStatus__Sequence * array = (physical_ai_interfaces__msg__TrainingStatus__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__msg__TrainingStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__msg__TrainingStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__msg__TrainingStatus__Sequence__destroy(physical_ai_interfaces__msg__TrainingStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__msg__TrainingStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__msg__TrainingStatus__Sequence__are_equal(const physical_ai_interfaces__msg__TrainingStatus__Sequence * lhs, const physical_ai_interfaces__msg__TrainingStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__msg__TrainingStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__msg__TrainingStatus__Sequence__copy(
  const physical_ai_interfaces__msg__TrainingStatus__Sequence * input,
  physical_ai_interfaces__msg__TrainingStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__msg__TrainingStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__msg__TrainingStatus * data =
      (physical_ai_interfaces__msg__TrainingStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__msg__TrainingStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__msg__TrainingStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__msg__TrainingStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
