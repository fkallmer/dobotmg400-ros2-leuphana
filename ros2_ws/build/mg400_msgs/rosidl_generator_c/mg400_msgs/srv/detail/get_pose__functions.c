// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mg400_msgs:srv/GetPose.idl
// generated code does not contain a copyright notice
#include "mg400_msgs/srv/detail/get_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
mg400_msgs__srv__GetPose_Request__init(mg400_msgs__srv__GetPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
mg400_msgs__srv__GetPose_Request__fini(mg400_msgs__srv__GetPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
mg400_msgs__srv__GetPose_Request__are_equal(const mg400_msgs__srv__GetPose_Request * lhs, const mg400_msgs__srv__GetPose_Request * rhs)
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
mg400_msgs__srv__GetPose_Request__copy(
  const mg400_msgs__srv__GetPose_Request * input,
  mg400_msgs__srv__GetPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

mg400_msgs__srv__GetPose_Request *
mg400_msgs__srv__GetPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__srv__GetPose_Request * msg = (mg400_msgs__srv__GetPose_Request *)allocator.allocate(sizeof(mg400_msgs__srv__GetPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__srv__GetPose_Request));
  bool success = mg400_msgs__srv__GetPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__srv__GetPose_Request__destroy(mg400_msgs__srv__GetPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__srv__GetPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__srv__GetPose_Request__Sequence__init(mg400_msgs__srv__GetPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__srv__GetPose_Request * data = NULL;

  if (size) {
    data = (mg400_msgs__srv__GetPose_Request *)allocator.zero_allocate(size, sizeof(mg400_msgs__srv__GetPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__srv__GetPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__srv__GetPose_Request__fini(&data[i - 1]);
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
mg400_msgs__srv__GetPose_Request__Sequence__fini(mg400_msgs__srv__GetPose_Request__Sequence * array)
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
      mg400_msgs__srv__GetPose_Request__fini(&array->data[i]);
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

mg400_msgs__srv__GetPose_Request__Sequence *
mg400_msgs__srv__GetPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__srv__GetPose_Request__Sequence * array = (mg400_msgs__srv__GetPose_Request__Sequence *)allocator.allocate(sizeof(mg400_msgs__srv__GetPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__srv__GetPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__srv__GetPose_Request__Sequence__destroy(mg400_msgs__srv__GetPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__srv__GetPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__srv__GetPose_Request__Sequence__are_equal(const mg400_msgs__srv__GetPose_Request__Sequence * lhs, const mg400_msgs__srv__GetPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__srv__GetPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__srv__GetPose_Request__Sequence__copy(
  const mg400_msgs__srv__GetPose_Request__Sequence * input,
  mg400_msgs__srv__GetPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__srv__GetPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__srv__GetPose_Request * data =
      (mg400_msgs__srv__GetPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__srv__GetPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__srv__GetPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__srv__GetPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
mg400_msgs__srv__GetPose_Response__init(mg400_msgs__srv__GetPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // error_id
  // pose1
  // pose2
  // pose3
  // pose4
  // pose5
  // pose6
  return true;
}

void
mg400_msgs__srv__GetPose_Response__fini(mg400_msgs__srv__GetPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // error_id
  // pose1
  // pose2
  // pose3
  // pose4
  // pose5
  // pose6
}

bool
mg400_msgs__srv__GetPose_Response__are_equal(const mg400_msgs__srv__GetPose_Response * lhs, const mg400_msgs__srv__GetPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // error_id
  if (lhs->error_id != rhs->error_id) {
    return false;
  }
  // pose1
  if (lhs->pose1 != rhs->pose1) {
    return false;
  }
  // pose2
  if (lhs->pose2 != rhs->pose2) {
    return false;
  }
  // pose3
  if (lhs->pose3 != rhs->pose3) {
    return false;
  }
  // pose4
  if (lhs->pose4 != rhs->pose4) {
    return false;
  }
  // pose5
  if (lhs->pose5 != rhs->pose5) {
    return false;
  }
  // pose6
  if (lhs->pose6 != rhs->pose6) {
    return false;
  }
  return true;
}

bool
mg400_msgs__srv__GetPose_Response__copy(
  const mg400_msgs__srv__GetPose_Response * input,
  mg400_msgs__srv__GetPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // error_id
  output->error_id = input->error_id;
  // pose1
  output->pose1 = input->pose1;
  // pose2
  output->pose2 = input->pose2;
  // pose3
  output->pose3 = input->pose3;
  // pose4
  output->pose4 = input->pose4;
  // pose5
  output->pose5 = input->pose5;
  // pose6
  output->pose6 = input->pose6;
  return true;
}

mg400_msgs__srv__GetPose_Response *
mg400_msgs__srv__GetPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__srv__GetPose_Response * msg = (mg400_msgs__srv__GetPose_Response *)allocator.allocate(sizeof(mg400_msgs__srv__GetPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__srv__GetPose_Response));
  bool success = mg400_msgs__srv__GetPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__srv__GetPose_Response__destroy(mg400_msgs__srv__GetPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__srv__GetPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__srv__GetPose_Response__Sequence__init(mg400_msgs__srv__GetPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__srv__GetPose_Response * data = NULL;

  if (size) {
    data = (mg400_msgs__srv__GetPose_Response *)allocator.zero_allocate(size, sizeof(mg400_msgs__srv__GetPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__srv__GetPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__srv__GetPose_Response__fini(&data[i - 1]);
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
mg400_msgs__srv__GetPose_Response__Sequence__fini(mg400_msgs__srv__GetPose_Response__Sequence * array)
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
      mg400_msgs__srv__GetPose_Response__fini(&array->data[i]);
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

mg400_msgs__srv__GetPose_Response__Sequence *
mg400_msgs__srv__GetPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__srv__GetPose_Response__Sequence * array = (mg400_msgs__srv__GetPose_Response__Sequence *)allocator.allocate(sizeof(mg400_msgs__srv__GetPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__srv__GetPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__srv__GetPose_Response__Sequence__destroy(mg400_msgs__srv__GetPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__srv__GetPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__srv__GetPose_Response__Sequence__are_equal(const mg400_msgs__srv__GetPose_Response__Sequence * lhs, const mg400_msgs__srv__GetPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__srv__GetPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__srv__GetPose_Response__Sequence__copy(
  const mg400_msgs__srv__GetPose_Response__Sequence * input,
  mg400_msgs__srv__GetPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__srv__GetPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__srv__GetPose_Response * data =
      (mg400_msgs__srv__GetPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__srv__GetPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__srv__GetPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__srv__GetPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
