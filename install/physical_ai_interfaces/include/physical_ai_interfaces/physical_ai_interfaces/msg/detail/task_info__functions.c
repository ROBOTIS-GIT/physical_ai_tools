// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from physical_ai_interfaces:msg/TaskInfo.idl
// generated code does not contain a copyright notice
#include "physical_ai_interfaces/msg/detail/task_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `task_name`
// Member `task_type`
// Member `user_id`
// Member `task_instruction`
// Member `policy_path`
// Member `tags`
#include "rosidl_runtime_c/string_functions.h"

bool
physical_ai_interfaces__msg__TaskInfo__init(physical_ai_interfaces__msg__TaskInfo * msg)
{
  if (!msg) {
    return false;
  }
  // task_name
  if (!rosidl_runtime_c__String__init(&msg->task_name)) {
    physical_ai_interfaces__msg__TaskInfo__fini(msg);
    return false;
  }
  // task_type
  if (!rosidl_runtime_c__String__init(&msg->task_type)) {
    physical_ai_interfaces__msg__TaskInfo__fini(msg);
    return false;
  }
  // user_id
  if (!rosidl_runtime_c__String__init(&msg->user_id)) {
    physical_ai_interfaces__msg__TaskInfo__fini(msg);
    return false;
  }
  // task_instruction
  if (!rosidl_runtime_c__String__Sequence__init(&msg->task_instruction, 0)) {
    physical_ai_interfaces__msg__TaskInfo__fini(msg);
    return false;
  }
  // policy_path
  if (!rosidl_runtime_c__String__init(&msg->policy_path)) {
    physical_ai_interfaces__msg__TaskInfo__fini(msg);
    return false;
  }
  // fps
  // tags
  if (!rosidl_runtime_c__String__Sequence__init(&msg->tags, 0)) {
    physical_ai_interfaces__msg__TaskInfo__fini(msg);
    return false;
  }
  // warmup_time_s
  // episode_time_s
  // reset_time_s
  // num_episodes
  // push_to_hub
  // private_mode
  // use_optimized_save_mode
  // record_inference_mode
  return true;
}

void
physical_ai_interfaces__msg__TaskInfo__fini(physical_ai_interfaces__msg__TaskInfo * msg)
{
  if (!msg) {
    return;
  }
  // task_name
  rosidl_runtime_c__String__fini(&msg->task_name);
  // task_type
  rosidl_runtime_c__String__fini(&msg->task_type);
  // user_id
  rosidl_runtime_c__String__fini(&msg->user_id);
  // task_instruction
  rosidl_runtime_c__String__Sequence__fini(&msg->task_instruction);
  // policy_path
  rosidl_runtime_c__String__fini(&msg->policy_path);
  // fps
  // tags
  rosidl_runtime_c__String__Sequence__fini(&msg->tags);
  // warmup_time_s
  // episode_time_s
  // reset_time_s
  // num_episodes
  // push_to_hub
  // private_mode
  // use_optimized_save_mode
  // record_inference_mode
}

bool
physical_ai_interfaces__msg__TaskInfo__are_equal(const physical_ai_interfaces__msg__TaskInfo * lhs, const physical_ai_interfaces__msg__TaskInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // task_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_name), &(rhs->task_name)))
  {
    return false;
  }
  // task_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_type), &(rhs->task_type)))
  {
    return false;
  }
  // user_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->user_id), &(rhs->user_id)))
  {
    return false;
  }
  // task_instruction
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->task_instruction), &(rhs->task_instruction)))
  {
    return false;
  }
  // policy_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->policy_path), &(rhs->policy_path)))
  {
    return false;
  }
  // fps
  if (lhs->fps != rhs->fps) {
    return false;
  }
  // tags
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->tags), &(rhs->tags)))
  {
    return false;
  }
  // warmup_time_s
  if (lhs->warmup_time_s != rhs->warmup_time_s) {
    return false;
  }
  // episode_time_s
  if (lhs->episode_time_s != rhs->episode_time_s) {
    return false;
  }
  // reset_time_s
  if (lhs->reset_time_s != rhs->reset_time_s) {
    return false;
  }
  // num_episodes
  if (lhs->num_episodes != rhs->num_episodes) {
    return false;
  }
  // push_to_hub
  if (lhs->push_to_hub != rhs->push_to_hub) {
    return false;
  }
  // private_mode
  if (lhs->private_mode != rhs->private_mode) {
    return false;
  }
  // use_optimized_save_mode
  if (lhs->use_optimized_save_mode != rhs->use_optimized_save_mode) {
    return false;
  }
  // record_inference_mode
  if (lhs->record_inference_mode != rhs->record_inference_mode) {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__msg__TaskInfo__copy(
  const physical_ai_interfaces__msg__TaskInfo * input,
  physical_ai_interfaces__msg__TaskInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // task_name
  if (!rosidl_runtime_c__String__copy(
      &(input->task_name), &(output->task_name)))
  {
    return false;
  }
  // task_type
  if (!rosidl_runtime_c__String__copy(
      &(input->task_type), &(output->task_type)))
  {
    return false;
  }
  // user_id
  if (!rosidl_runtime_c__String__copy(
      &(input->user_id), &(output->user_id)))
  {
    return false;
  }
  // task_instruction
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->task_instruction), &(output->task_instruction)))
  {
    return false;
  }
  // policy_path
  if (!rosidl_runtime_c__String__copy(
      &(input->policy_path), &(output->policy_path)))
  {
    return false;
  }
  // fps
  output->fps = input->fps;
  // tags
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->tags), &(output->tags)))
  {
    return false;
  }
  // warmup_time_s
  output->warmup_time_s = input->warmup_time_s;
  // episode_time_s
  output->episode_time_s = input->episode_time_s;
  // reset_time_s
  output->reset_time_s = input->reset_time_s;
  // num_episodes
  output->num_episodes = input->num_episodes;
  // push_to_hub
  output->push_to_hub = input->push_to_hub;
  // private_mode
  output->private_mode = input->private_mode;
  // use_optimized_save_mode
  output->use_optimized_save_mode = input->use_optimized_save_mode;
  // record_inference_mode
  output->record_inference_mode = input->record_inference_mode;
  return true;
}

physical_ai_interfaces__msg__TaskInfo *
physical_ai_interfaces__msg__TaskInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TaskInfo * msg = (physical_ai_interfaces__msg__TaskInfo *)allocator.allocate(sizeof(physical_ai_interfaces__msg__TaskInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__msg__TaskInfo));
  bool success = physical_ai_interfaces__msg__TaskInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__msg__TaskInfo__destroy(physical_ai_interfaces__msg__TaskInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__msg__TaskInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__msg__TaskInfo__Sequence__init(physical_ai_interfaces__msg__TaskInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TaskInfo * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__msg__TaskInfo *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__msg__TaskInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__msg__TaskInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__msg__TaskInfo__fini(&data[i - 1]);
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
physical_ai_interfaces__msg__TaskInfo__Sequence__fini(physical_ai_interfaces__msg__TaskInfo__Sequence * array)
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
      physical_ai_interfaces__msg__TaskInfo__fini(&array->data[i]);
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

physical_ai_interfaces__msg__TaskInfo__Sequence *
physical_ai_interfaces__msg__TaskInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__msg__TaskInfo__Sequence * array = (physical_ai_interfaces__msg__TaskInfo__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__msg__TaskInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__msg__TaskInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__msg__TaskInfo__Sequence__destroy(physical_ai_interfaces__msg__TaskInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__msg__TaskInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__msg__TaskInfo__Sequence__are_equal(const physical_ai_interfaces__msg__TaskInfo__Sequence * lhs, const physical_ai_interfaces__msg__TaskInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__msg__TaskInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__msg__TaskInfo__Sequence__copy(
  const physical_ai_interfaces__msg__TaskInfo__Sequence * input,
  physical_ai_interfaces__msg__TaskInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__msg__TaskInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__msg__TaskInfo * data =
      (physical_ai_interfaces__msg__TaskInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__msg__TaskInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__msg__TaskInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__msg__TaskInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
