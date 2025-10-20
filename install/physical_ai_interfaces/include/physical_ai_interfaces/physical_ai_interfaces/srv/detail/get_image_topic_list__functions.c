// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from physical_ai_interfaces:srv/GetImageTopicList.idl
// generated code does not contain a copyright notice
#include "physical_ai_interfaces/srv/detail/get_image_topic_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
physical_ai_interfaces__srv__GetImageTopicList_Request__init(physical_ai_interfaces__srv__GetImageTopicList_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Request__fini(physical_ai_interfaces__srv__GetImageTopicList_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Request__are_equal(const physical_ai_interfaces__srv__GetImageTopicList_Request * lhs, const physical_ai_interfaces__srv__GetImageTopicList_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Request__copy(
  const physical_ai_interfaces__srv__GetImageTopicList_Request * input,
  physical_ai_interfaces__srv__GetImageTopicList_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

physical_ai_interfaces__srv__GetImageTopicList_Request *
physical_ai_interfaces__srv__GetImageTopicList_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Request * msg = (physical_ai_interfaces__srv__GetImageTopicList_Request *)allocator.allocate(sizeof(physical_ai_interfaces__srv__GetImageTopicList_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__srv__GetImageTopicList_Request));
  bool success = physical_ai_interfaces__srv__GetImageTopicList_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Request__destroy(physical_ai_interfaces__srv__GetImageTopicList_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__srv__GetImageTopicList_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__init(physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Request * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__srv__GetImageTopicList_Request *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__srv__GetImageTopicList_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__srv__GetImageTopicList_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__srv__GetImageTopicList_Request__fini(&data[i - 1]);
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
physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__fini(physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence * array)
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
      physical_ai_interfaces__srv__GetImageTopicList_Request__fini(&array->data[i]);
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

physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence *
physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence * array = (physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__destroy(physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__are_equal(const physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence * lhs, const physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__srv__GetImageTopicList_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__copy(
  const physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence * input,
  physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__srv__GetImageTopicList_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__srv__GetImageTopicList_Request * data =
      (physical_ai_interfaces__srv__GetImageTopicList_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__srv__GetImageTopicList_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__srv__GetImageTopicList_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__srv__GetImageTopicList_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `image_topic_list`
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
physical_ai_interfaces__srv__GetImageTopicList_Response__init(physical_ai_interfaces__srv__GetImageTopicList_Response * msg)
{
  if (!msg) {
    return false;
  }
  // image_topic_list
  if (!rosidl_runtime_c__String__Sequence__init(&msg->image_topic_list, 0)) {
    physical_ai_interfaces__srv__GetImageTopicList_Response__fini(msg);
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    physical_ai_interfaces__srv__GetImageTopicList_Response__fini(msg);
    return false;
  }
  return true;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Response__fini(physical_ai_interfaces__srv__GetImageTopicList_Response * msg)
{
  if (!msg) {
    return;
  }
  // image_topic_list
  rosidl_runtime_c__String__Sequence__fini(&msg->image_topic_list);
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Response__are_equal(const physical_ai_interfaces__srv__GetImageTopicList_Response * lhs, const physical_ai_interfaces__srv__GetImageTopicList_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // image_topic_list
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->image_topic_list), &(rhs->image_topic_list)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Response__copy(
  const physical_ai_interfaces__srv__GetImageTopicList_Response * input,
  physical_ai_interfaces__srv__GetImageTopicList_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // image_topic_list
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->image_topic_list), &(output->image_topic_list)))
  {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

physical_ai_interfaces__srv__GetImageTopicList_Response *
physical_ai_interfaces__srv__GetImageTopicList_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Response * msg = (physical_ai_interfaces__srv__GetImageTopicList_Response *)allocator.allocate(sizeof(physical_ai_interfaces__srv__GetImageTopicList_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__srv__GetImageTopicList_Response));
  bool success = physical_ai_interfaces__srv__GetImageTopicList_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Response__destroy(physical_ai_interfaces__srv__GetImageTopicList_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__srv__GetImageTopicList_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__init(physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Response * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__srv__GetImageTopicList_Response *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__srv__GetImageTopicList_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__srv__GetImageTopicList_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__srv__GetImageTopicList_Response__fini(&data[i - 1]);
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
physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__fini(physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence * array)
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
      physical_ai_interfaces__srv__GetImageTopicList_Response__fini(&array->data[i]);
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

physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence *
physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence * array = (physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__destroy(physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__are_equal(const physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence * lhs, const physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__srv__GetImageTopicList_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__copy(
  const physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence * input,
  physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__srv__GetImageTopicList_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__srv__GetImageTopicList_Response * data =
      (physical_ai_interfaces__srv__GetImageTopicList_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__srv__GetImageTopicList_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__srv__GetImageTopicList_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__srv__GetImageTopicList_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "physical_ai_interfaces/srv/detail/get_image_topic_list__functions.h"

bool
physical_ai_interfaces__srv__GetImageTopicList_Event__init(physical_ai_interfaces__srv__GetImageTopicList_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    physical_ai_interfaces__srv__GetImageTopicList_Event__fini(msg);
    return false;
  }
  // request
  if (!physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__init(&msg->request, 0)) {
    physical_ai_interfaces__srv__GetImageTopicList_Event__fini(msg);
    return false;
  }
  // response
  if (!physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__init(&msg->response, 0)) {
    physical_ai_interfaces__srv__GetImageTopicList_Event__fini(msg);
    return false;
  }
  return true;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Event__fini(physical_ai_interfaces__srv__GetImageTopicList_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__fini(&msg->request);
  // response
  physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__fini(&msg->response);
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Event__are_equal(const physical_ai_interfaces__srv__GetImageTopicList_Event * lhs, const physical_ai_interfaces__srv__GetImageTopicList_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Event__copy(
  const physical_ai_interfaces__srv__GetImageTopicList_Event * input,
  physical_ai_interfaces__srv__GetImageTopicList_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!physical_ai_interfaces__srv__GetImageTopicList_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!physical_ai_interfaces__srv__GetImageTopicList_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

physical_ai_interfaces__srv__GetImageTopicList_Event *
physical_ai_interfaces__srv__GetImageTopicList_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Event * msg = (physical_ai_interfaces__srv__GetImageTopicList_Event *)allocator.allocate(sizeof(physical_ai_interfaces__srv__GetImageTopicList_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(physical_ai_interfaces__srv__GetImageTopicList_Event));
  bool success = physical_ai_interfaces__srv__GetImageTopicList_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Event__destroy(physical_ai_interfaces__srv__GetImageTopicList_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    physical_ai_interfaces__srv__GetImageTopicList_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence__init(physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Event * data = NULL;

  if (size) {
    data = (physical_ai_interfaces__srv__GetImageTopicList_Event *)allocator.zero_allocate(size, sizeof(physical_ai_interfaces__srv__GetImageTopicList_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = physical_ai_interfaces__srv__GetImageTopicList_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        physical_ai_interfaces__srv__GetImageTopicList_Event__fini(&data[i - 1]);
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
physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence__fini(physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence * array)
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
      physical_ai_interfaces__srv__GetImageTopicList_Event__fini(&array->data[i]);
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

physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence *
physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence * array = (physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence *)allocator.allocate(sizeof(physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence__destroy(physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence__are_equal(const physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence * lhs, const physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!physical_ai_interfaces__srv__GetImageTopicList_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence__copy(
  const physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence * input,
  physical_ai_interfaces__srv__GetImageTopicList_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(physical_ai_interfaces__srv__GetImageTopicList_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    physical_ai_interfaces__srv__GetImageTopicList_Event * data =
      (physical_ai_interfaces__srv__GetImageTopicList_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!physical_ai_interfaces__srv__GetImageTopicList_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          physical_ai_interfaces__srv__GetImageTopicList_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!physical_ai_interfaces__srv__GetImageTopicList_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
