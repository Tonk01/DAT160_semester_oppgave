// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bug2_interface:srv/GoToPoint.idl
// generated code does not contain a copyright notice
#include "bug2_interface/srv/detail/go_to_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `target_position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
bug2_interface__srv__GoToPoint_Request__init(bug2_interface__srv__GoToPoint_Request * msg)
{
  if (!msg) {
    return false;
  }
  // move_switch
  // target_position
  if (!geometry_msgs__msg__Point__init(&msg->target_position)) {
    bug2_interface__srv__GoToPoint_Request__fini(msg);
    return false;
  }
  return true;
}

void
bug2_interface__srv__GoToPoint_Request__fini(bug2_interface__srv__GoToPoint_Request * msg)
{
  if (!msg) {
    return;
  }
  // move_switch
  // target_position
  geometry_msgs__msg__Point__fini(&msg->target_position);
}

bool
bug2_interface__srv__GoToPoint_Request__are_equal(const bug2_interface__srv__GoToPoint_Request * lhs, const bug2_interface__srv__GoToPoint_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // move_switch
  if (lhs->move_switch != rhs->move_switch) {
    return false;
  }
  // target_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->target_position), &(rhs->target_position)))
  {
    return false;
  }
  return true;
}

bool
bug2_interface__srv__GoToPoint_Request__copy(
  const bug2_interface__srv__GoToPoint_Request * input,
  bug2_interface__srv__GoToPoint_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // move_switch
  output->move_switch = input->move_switch;
  // target_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->target_position), &(output->target_position)))
  {
    return false;
  }
  return true;
}

bug2_interface__srv__GoToPoint_Request *
bug2_interface__srv__GoToPoint_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bug2_interface__srv__GoToPoint_Request * msg = (bug2_interface__srv__GoToPoint_Request *)allocator.allocate(sizeof(bug2_interface__srv__GoToPoint_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bug2_interface__srv__GoToPoint_Request));
  bool success = bug2_interface__srv__GoToPoint_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bug2_interface__srv__GoToPoint_Request__destroy(bug2_interface__srv__GoToPoint_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bug2_interface__srv__GoToPoint_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bug2_interface__srv__GoToPoint_Request__Sequence__init(bug2_interface__srv__GoToPoint_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bug2_interface__srv__GoToPoint_Request * data = NULL;

  if (size) {
    data = (bug2_interface__srv__GoToPoint_Request *)allocator.zero_allocate(size, sizeof(bug2_interface__srv__GoToPoint_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bug2_interface__srv__GoToPoint_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bug2_interface__srv__GoToPoint_Request__fini(&data[i - 1]);
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
bug2_interface__srv__GoToPoint_Request__Sequence__fini(bug2_interface__srv__GoToPoint_Request__Sequence * array)
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
      bug2_interface__srv__GoToPoint_Request__fini(&array->data[i]);
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

bug2_interface__srv__GoToPoint_Request__Sequence *
bug2_interface__srv__GoToPoint_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bug2_interface__srv__GoToPoint_Request__Sequence * array = (bug2_interface__srv__GoToPoint_Request__Sequence *)allocator.allocate(sizeof(bug2_interface__srv__GoToPoint_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bug2_interface__srv__GoToPoint_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bug2_interface__srv__GoToPoint_Request__Sequence__destroy(bug2_interface__srv__GoToPoint_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bug2_interface__srv__GoToPoint_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bug2_interface__srv__GoToPoint_Request__Sequence__are_equal(const bug2_interface__srv__GoToPoint_Request__Sequence * lhs, const bug2_interface__srv__GoToPoint_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bug2_interface__srv__GoToPoint_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bug2_interface__srv__GoToPoint_Request__Sequence__copy(
  const bug2_interface__srv__GoToPoint_Request__Sequence * input,
  bug2_interface__srv__GoToPoint_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bug2_interface__srv__GoToPoint_Request);
    bug2_interface__srv__GoToPoint_Request * data =
      (bug2_interface__srv__GoToPoint_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bug2_interface__srv__GoToPoint_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          bug2_interface__srv__GoToPoint_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bug2_interface__srv__GoToPoint_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
bug2_interface__srv__GoToPoint_Response__init(bug2_interface__srv__GoToPoint_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
bug2_interface__srv__GoToPoint_Response__fini(bug2_interface__srv__GoToPoint_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
bug2_interface__srv__GoToPoint_Response__are_equal(const bug2_interface__srv__GoToPoint_Response * lhs, const bug2_interface__srv__GoToPoint_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
bug2_interface__srv__GoToPoint_Response__copy(
  const bug2_interface__srv__GoToPoint_Response * input,
  bug2_interface__srv__GoToPoint_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

bug2_interface__srv__GoToPoint_Response *
bug2_interface__srv__GoToPoint_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bug2_interface__srv__GoToPoint_Response * msg = (bug2_interface__srv__GoToPoint_Response *)allocator.allocate(sizeof(bug2_interface__srv__GoToPoint_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bug2_interface__srv__GoToPoint_Response));
  bool success = bug2_interface__srv__GoToPoint_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bug2_interface__srv__GoToPoint_Response__destroy(bug2_interface__srv__GoToPoint_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bug2_interface__srv__GoToPoint_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bug2_interface__srv__GoToPoint_Response__Sequence__init(bug2_interface__srv__GoToPoint_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bug2_interface__srv__GoToPoint_Response * data = NULL;

  if (size) {
    data = (bug2_interface__srv__GoToPoint_Response *)allocator.zero_allocate(size, sizeof(bug2_interface__srv__GoToPoint_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bug2_interface__srv__GoToPoint_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bug2_interface__srv__GoToPoint_Response__fini(&data[i - 1]);
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
bug2_interface__srv__GoToPoint_Response__Sequence__fini(bug2_interface__srv__GoToPoint_Response__Sequence * array)
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
      bug2_interface__srv__GoToPoint_Response__fini(&array->data[i]);
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

bug2_interface__srv__GoToPoint_Response__Sequence *
bug2_interface__srv__GoToPoint_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bug2_interface__srv__GoToPoint_Response__Sequence * array = (bug2_interface__srv__GoToPoint_Response__Sequence *)allocator.allocate(sizeof(bug2_interface__srv__GoToPoint_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bug2_interface__srv__GoToPoint_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bug2_interface__srv__GoToPoint_Response__Sequence__destroy(bug2_interface__srv__GoToPoint_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bug2_interface__srv__GoToPoint_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bug2_interface__srv__GoToPoint_Response__Sequence__are_equal(const bug2_interface__srv__GoToPoint_Response__Sequence * lhs, const bug2_interface__srv__GoToPoint_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bug2_interface__srv__GoToPoint_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bug2_interface__srv__GoToPoint_Response__Sequence__copy(
  const bug2_interface__srv__GoToPoint_Response__Sequence * input,
  bug2_interface__srv__GoToPoint_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bug2_interface__srv__GoToPoint_Response);
    bug2_interface__srv__GoToPoint_Response * data =
      (bug2_interface__srv__GoToPoint_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bug2_interface__srv__GoToPoint_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          bug2_interface__srv__GoToPoint_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bug2_interface__srv__GoToPoint_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
