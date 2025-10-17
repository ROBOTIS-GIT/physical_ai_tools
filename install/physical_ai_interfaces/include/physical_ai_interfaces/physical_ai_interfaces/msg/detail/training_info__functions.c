// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from physical_ai_interfaces:msg/TrainingInfo.idl
// generated code does not contain a copyright notice
#include "physical_ai_interfaces/msg/detail/training_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `dataset`
// Member `policy_type`
// Member `output_folder_name`
// Member `policy_device`
#include "rosidl_runtime_c/string_functions.h"

bool
physical_ai_interfaces__msg__TrainingInfo__init(physical_ai_interfaces__msg__TrainingInfo * msg)
{
  if (!msg) {
    return false;
  }
  // dataset
  if (!rosidl_runtime_c__String__init(&msg->dataset)) {
    physical_ai_interfaces__msg__TrainingInfo__fini(msg);
    return false;
  }
  // policy_type
  if (!rosidl_runtime_c__String__init(&msg->policy_type)) {
    physical_ai_interfaces__msg__TrainingInfo__fini(msg);
    return false;
  }
  // output_folder_name
  if (!rosidl_runtime_c__String__init(&msg->output_folder_name)) {
    physical_ai_interfaces__msg__TrainingInfo__fini(msg);
    return false;
  }
  // policy_device
  if (!rosidl_runtime_c__String__init(&msg->policy_device)) {
    physical_ai_interfaces__msg__TrainingInfo__fini(msg);
    return false;
  }
  // seed
  // num_workers
  // batch_size
  // steps
  // eval_freq
  // log_freq
  // save_freq
  return true;
}

void
physical_ai_interfaces__msg__TrainingInfo__fini(physical_ai_interfaces__msg__TrainingInfo * msg)
{
  if (!msg) {
    return;
  }
  // dataset
  rosidl_runtime_c__String__fini(&msg->dataset);
  // policy_type
  rosidl_runtime_c__String__fini(&msg->policy_type);
  // output_folder_name
  rosidl_runtime_c__String__fini(&msg->output_folder_name);
  // policy_device
  rosidl_runtime_c__String__fini(&msg->policy_device);
  // seed
  // num_workers
  // batch_size
  // steps
  // eval_freq
  // log_freq
  // save_freq
}

bool
physical_ai_interfaces__msg__TrainingInfo__are_equal(const physical_ai_interfaces__msg__TrainingInfo * lhs, const physical_ai_interfaces__msg__TrainingInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dataset
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->dataset), &(rhs->dataset)))
  {
    return false;
  }
  // policy_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->policy_type), &(rhs->policy_type)))
  {
    return false;
  }
  // output_folder_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->output_folder_name), &(rhs->output_folder_name)))
  {
    return false;
  }
  // policy_device
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->policy_device), &(rhs->policy_device)))
  {
    return false;
  }
  // seed
  if (lhs->seed != rhs->seed) {
    return false;
  }
  // num_workers
  if (lhs->num_workers != rhs->num_workers) {
    return false;
  }
  // batch_size
  if (lhs->batch_size != rhs->batch_size) {
    return false;
  }
  // steps
  if (lhs->steps != rhs->steps) {
    return false;
  }
  // eval_freq
  if (lhs->eval_freq != rhs->eval_freq) {
    return false;
  }
  // log_freq
  if (lhs->log_freq != rhs->log_freq) {
    return false;
  }
  // save_freq
  if (lhs->save_freq != rhs->save_freq) {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__msg__TrainingInfo__copy(
  const physical_ai_interfaces__msg__TrainingInfo * input,
  physical_ai_interfaces__msg__TrainingInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // dataset
  if (!rosidl_runtime_c__String__copy(
      &(input->dataset), &(output->dataset)))
  {
    return false;
  }
  // policy_type
  if (!rosidl_runtime_c__String__copy(
      &(input->policy_type), &(output->policy_type)))
  {
    return false;
  }
  // output_folder_name
  if (!rosidl_runtime_c__String__copy(
      &(input->output_folder_name), &(output->output_folder_name)))
  {
    return false;
  }
  // policy_device
  if (!rosidl_runtime_c__String__copy(
      &(input->policy_device), &(output->policy_device)))
  {
    return false;
  }
  // seed
  output->seed = input->seed;
  // num_workers
  output->num_workers = input->num_workers;
  // batch_size
  output->batch_size = input->batch_size;
  // steps
  output->steps = input->steps;
  // eval_freq
  output->eval_freq = input->eval_freq;
  // log_freq
  output->log_freq = input->log_freq;
  // save_freq
  output->save_freq = input->save_freq;
  return true;
}

physical_ai_interfaces__msg__TrainingInfo *
physical_ai_interfaces__msg__TrainingInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TrainingInfo * msg = (physical_ai_interfaces__msg__TrainingInfo *)allocator.allocate(sizeof(physical_ai_interfaces__msg__TrainingInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__msg__TrainingInfo));
  bool success = physical_ai_interfaces__msg__TrainingInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__msg__TrainingInfo__destroy(physical_ai_interfaces__msg__TrainingInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__msg__TrainingInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__msg__TrainingInfo__Sequence__init(physical_ai_interfaces__msg__TrainingInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TrainingInfo * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__msg__TrainingInfo *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__msg__TrainingInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__msg__TrainingInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__msg__TrainingInfo__fini(&data[i - 1]);
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
physical_ai_interfaces__msg__TrainingInfo__Sequence__fini(physical_ai_interfaces__msg__TrainingInfo__Sequence * array)
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
      physical_ai_interfaces__msg__TrainingInfo__fini(&array->data[i]);
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

physical_ai_interfaces__msg__TrainingInfo__Sequence *
physical_ai_interfaces__msg__TrainingInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TrainingInfo__Sequence * array = (physical_ai_interfaces__msg__TrainingInfo__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__msg__TrainingInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__msg__TrainingInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__msg__TrainingInfo__Sequence__destroy(physical_ai_interfaces__msg__TrainingInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__msg__TrainingInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__msg__TrainingInfo__Sequence__are_equal(const physical_ai_interfaces__msg__TrainingInfo__Sequence * lhs, const physical_ai_interfaces__msg__TrainingInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__msg__TrainingInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__msg__TrainingInfo__Sequence__copy(
  const physical_ai_interfaces__msg__TrainingInfo__Sequence * input,
  physical_ai_interfaces__msg__TrainingInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__msg__TrainingInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__msg__TrainingInfo * data =
      (physical_ai_interfaces__msg__TrainingInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__msg__TrainingInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__msg__TrainingInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__msg__TrainingInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
