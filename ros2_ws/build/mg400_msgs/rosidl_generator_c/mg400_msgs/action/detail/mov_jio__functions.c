// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mg400_msgs:action/MovJIO.idl
// generated code does not contain a copyright notice
#include "mg400_msgs/action/detail/mov_jio__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `mode`
#include "mg400_msgs/msg/detail/distance_mode__functions.h"
// Member `index`
#include "mg400_msgs/msg/detail/do_index__functions.h"
// Member `status`
#include "mg400_msgs/msg/detail/do_status__functions.h"

bool
mg400_msgs__action__MovJIO_Goal__init(mg400_msgs__action__MovJIO_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pose)) {
    mg400_msgs__action__MovJIO_Goal__fini(msg);
    return false;
  }
  // mode
  if (!mg400_msgs__msg__DistanceMode__init(&msg->mode)) {
    mg400_msgs__action__MovJIO_Goal__fini(msg);
    return false;
  }
  // distance
  // index
  if (!mg400_msgs__msg__DOIndex__init(&msg->index)) {
    mg400_msgs__action__MovJIO_Goal__fini(msg);
    return false;
  }
  // status
  if (!mg400_msgs__msg__DOStatus__init(&msg->status)) {
    mg400_msgs__action__MovJIO_Goal__fini(msg);
    return false;
  }
  // set_speed_j
  // speed_j
  // set_acc_j
  // acc_j
  // set_cp
  // cp
  return true;
}

void
mg400_msgs__action__MovJIO_Goal__fini(mg400_msgs__action__MovJIO_Goal * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pose);
  // mode
  mg400_msgs__msg__DistanceMode__fini(&msg->mode);
  // distance
  // index
  mg400_msgs__msg__DOIndex__fini(&msg->index);
  // status
  mg400_msgs__msg__DOStatus__fini(&msg->status);
  // set_speed_j
  // speed_j
  // set_acc_j
  // acc_j
  // set_cp
  // cp
}

bool
mg400_msgs__action__MovJIO_Goal__are_equal(const mg400_msgs__action__MovJIO_Goal * lhs, const mg400_msgs__action__MovJIO_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // mode
  if (!mg400_msgs__msg__DistanceMode__are_equal(
      &(lhs->mode), &(rhs->mode)))
  {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // index
  if (!mg400_msgs__msg__DOIndex__are_equal(
      &(lhs->index), &(rhs->index)))
  {
    return false;
  }
  // status
  if (!mg400_msgs__msg__DOStatus__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  // set_speed_j
  if (lhs->set_speed_j != rhs->set_speed_j) {
    return false;
  }
  // speed_j
  if (lhs->speed_j != rhs->speed_j) {
    return false;
  }
  // set_acc_j
  if (lhs->set_acc_j != rhs->set_acc_j) {
    return false;
  }
  // acc_j
  if (lhs->acc_j != rhs->acc_j) {
    return false;
  }
  // set_cp
  if (lhs->set_cp != rhs->set_cp) {
    return false;
  }
  // cp
  if (lhs->cp != rhs->cp) {
    return false;
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_Goal__copy(
  const mg400_msgs__action__MovJIO_Goal * input,
  mg400_msgs__action__MovJIO_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // mode
  if (!mg400_msgs__msg__DistanceMode__copy(
      &(input->mode), &(output->mode)))
  {
    return false;
  }
  // distance
  output->distance = input->distance;
  // index
  if (!mg400_msgs__msg__DOIndex__copy(
      &(input->index), &(output->index)))
  {
    return false;
  }
  // status
  if (!mg400_msgs__msg__DOStatus__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  // set_speed_j
  output->set_speed_j = input->set_speed_j;
  // speed_j
  output->speed_j = input->speed_j;
  // set_acc_j
  output->set_acc_j = input->set_acc_j;
  // acc_j
  output->acc_j = input->acc_j;
  // set_cp
  output->set_cp = input->set_cp;
  // cp
  output->cp = input->cp;
  return true;
}

mg400_msgs__action__MovJIO_Goal *
mg400_msgs__action__MovJIO_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Goal * msg = (mg400_msgs__action__MovJIO_Goal *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__action__MovJIO_Goal));
  bool success = mg400_msgs__action__MovJIO_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__action__MovJIO_Goal__destroy(mg400_msgs__action__MovJIO_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__action__MovJIO_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__action__MovJIO_Goal__Sequence__init(mg400_msgs__action__MovJIO_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Goal * data = NULL;

  if (size) {
    data = (mg400_msgs__action__MovJIO_Goal *)allocator.zero_allocate(size, sizeof(mg400_msgs__action__MovJIO_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__action__MovJIO_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__action__MovJIO_Goal__fini(&data[i - 1]);
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
mg400_msgs__action__MovJIO_Goal__Sequence__fini(mg400_msgs__action__MovJIO_Goal__Sequence * array)
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
      mg400_msgs__action__MovJIO_Goal__fini(&array->data[i]);
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

mg400_msgs__action__MovJIO_Goal__Sequence *
mg400_msgs__action__MovJIO_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Goal__Sequence * array = (mg400_msgs__action__MovJIO_Goal__Sequence *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__action__MovJIO_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__action__MovJIO_Goal__Sequence__destroy(mg400_msgs__action__MovJIO_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__action__MovJIO_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__action__MovJIO_Goal__Sequence__are_equal(const mg400_msgs__action__MovJIO_Goal__Sequence * lhs, const mg400_msgs__action__MovJIO_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__action__MovJIO_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_Goal__Sequence__copy(
  const mg400_msgs__action__MovJIO_Goal__Sequence * input,
  mg400_msgs__action__MovJIO_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__action__MovJIO_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__action__MovJIO_Goal * data =
      (mg400_msgs__action__MovJIO_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__action__MovJIO_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__action__MovJIO_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__action__MovJIO_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `error_id`
#include "mg400_msgs/msg/detail/error_id__functions.h"

bool
mg400_msgs__action__MovJIO_Result__init(mg400_msgs__action__MovJIO_Result * msg)
{
  if (!msg) {
    return false;
  }
  // result
  // error_id
  if (!mg400_msgs__msg__ErrorID__init(&msg->error_id)) {
    mg400_msgs__action__MovJIO_Result__fini(msg);
    return false;
  }
  return true;
}

void
mg400_msgs__action__MovJIO_Result__fini(mg400_msgs__action__MovJIO_Result * msg)
{
  if (!msg) {
    return;
  }
  // result
  // error_id
  mg400_msgs__msg__ErrorID__fini(&msg->error_id);
}

bool
mg400_msgs__action__MovJIO_Result__are_equal(const mg400_msgs__action__MovJIO_Result * lhs, const mg400_msgs__action__MovJIO_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  // error_id
  if (!mg400_msgs__msg__ErrorID__are_equal(
      &(lhs->error_id), &(rhs->error_id)))
  {
    return false;
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_Result__copy(
  const mg400_msgs__action__MovJIO_Result * input,
  mg400_msgs__action__MovJIO_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  // error_id
  if (!mg400_msgs__msg__ErrorID__copy(
      &(input->error_id), &(output->error_id)))
  {
    return false;
  }
  return true;
}

mg400_msgs__action__MovJIO_Result *
mg400_msgs__action__MovJIO_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Result * msg = (mg400_msgs__action__MovJIO_Result *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__action__MovJIO_Result));
  bool success = mg400_msgs__action__MovJIO_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__action__MovJIO_Result__destroy(mg400_msgs__action__MovJIO_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__action__MovJIO_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__action__MovJIO_Result__Sequence__init(mg400_msgs__action__MovJIO_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Result * data = NULL;

  if (size) {
    data = (mg400_msgs__action__MovJIO_Result *)allocator.zero_allocate(size, sizeof(mg400_msgs__action__MovJIO_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__action__MovJIO_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__action__MovJIO_Result__fini(&data[i - 1]);
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
mg400_msgs__action__MovJIO_Result__Sequence__fini(mg400_msgs__action__MovJIO_Result__Sequence * array)
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
      mg400_msgs__action__MovJIO_Result__fini(&array->data[i]);
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

mg400_msgs__action__MovJIO_Result__Sequence *
mg400_msgs__action__MovJIO_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Result__Sequence * array = (mg400_msgs__action__MovJIO_Result__Sequence *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__action__MovJIO_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__action__MovJIO_Result__Sequence__destroy(mg400_msgs__action__MovJIO_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__action__MovJIO_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__action__MovJIO_Result__Sequence__are_equal(const mg400_msgs__action__MovJIO_Result__Sequence * lhs, const mg400_msgs__action__MovJIO_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__action__MovJIO_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_Result__Sequence__copy(
  const mg400_msgs__action__MovJIO_Result__Sequence * input,
  mg400_msgs__action__MovJIO_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__action__MovJIO_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__action__MovJIO_Result * data =
      (mg400_msgs__action__MovJIO_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__action__MovJIO_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__action__MovJIO_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__action__MovJIO_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
mg400_msgs__action__MovJIO_Feedback__init(mg400_msgs__action__MovJIO_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    mg400_msgs__action__MovJIO_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
mg400_msgs__action__MovJIO_Feedback__fini(mg400_msgs__action__MovJIO_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->current_pose);
}

bool
mg400_msgs__action__MovJIO_Feedback__are_equal(const mg400_msgs__action__MovJIO_Feedback * lhs, const mg400_msgs__action__MovJIO_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->current_pose), &(rhs->current_pose)))
  {
    return false;
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_Feedback__copy(
  const mg400_msgs__action__MovJIO_Feedback * input,
  mg400_msgs__action__MovJIO_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->current_pose), &(output->current_pose)))
  {
    return false;
  }
  return true;
}

mg400_msgs__action__MovJIO_Feedback *
mg400_msgs__action__MovJIO_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Feedback * msg = (mg400_msgs__action__MovJIO_Feedback *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__action__MovJIO_Feedback));
  bool success = mg400_msgs__action__MovJIO_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__action__MovJIO_Feedback__destroy(mg400_msgs__action__MovJIO_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__action__MovJIO_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__action__MovJIO_Feedback__Sequence__init(mg400_msgs__action__MovJIO_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Feedback * data = NULL;

  if (size) {
    data = (mg400_msgs__action__MovJIO_Feedback *)allocator.zero_allocate(size, sizeof(mg400_msgs__action__MovJIO_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__action__MovJIO_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__action__MovJIO_Feedback__fini(&data[i - 1]);
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
mg400_msgs__action__MovJIO_Feedback__Sequence__fini(mg400_msgs__action__MovJIO_Feedback__Sequence * array)
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
      mg400_msgs__action__MovJIO_Feedback__fini(&array->data[i]);
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

mg400_msgs__action__MovJIO_Feedback__Sequence *
mg400_msgs__action__MovJIO_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_Feedback__Sequence * array = (mg400_msgs__action__MovJIO_Feedback__Sequence *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__action__MovJIO_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__action__MovJIO_Feedback__Sequence__destroy(mg400_msgs__action__MovJIO_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__action__MovJIO_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__action__MovJIO_Feedback__Sequence__are_equal(const mg400_msgs__action__MovJIO_Feedback__Sequence * lhs, const mg400_msgs__action__MovJIO_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__action__MovJIO_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_Feedback__Sequence__copy(
  const mg400_msgs__action__MovJIO_Feedback__Sequence * input,
  mg400_msgs__action__MovJIO_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__action__MovJIO_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__action__MovJIO_Feedback * data =
      (mg400_msgs__action__MovJIO_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__action__MovJIO_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__action__MovJIO_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__action__MovJIO_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "mg400_msgs/action/detail/mov_jio__functions.h"

bool
mg400_msgs__action__MovJIO_SendGoal_Request__init(mg400_msgs__action__MovJIO_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mg400_msgs__action__MovJIO_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!mg400_msgs__action__MovJIO_Goal__init(&msg->goal)) {
    mg400_msgs__action__MovJIO_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
mg400_msgs__action__MovJIO_SendGoal_Request__fini(mg400_msgs__action__MovJIO_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  mg400_msgs__action__MovJIO_Goal__fini(&msg->goal);
}

bool
mg400_msgs__action__MovJIO_SendGoal_Request__are_equal(const mg400_msgs__action__MovJIO_SendGoal_Request * lhs, const mg400_msgs__action__MovJIO_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!mg400_msgs__action__MovJIO_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_SendGoal_Request__copy(
  const mg400_msgs__action__MovJIO_SendGoal_Request * input,
  mg400_msgs__action__MovJIO_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!mg400_msgs__action__MovJIO_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

mg400_msgs__action__MovJIO_SendGoal_Request *
mg400_msgs__action__MovJIO_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_SendGoal_Request * msg = (mg400_msgs__action__MovJIO_SendGoal_Request *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__action__MovJIO_SendGoal_Request));
  bool success = mg400_msgs__action__MovJIO_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__action__MovJIO_SendGoal_Request__destroy(mg400_msgs__action__MovJIO_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__action__MovJIO_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__action__MovJIO_SendGoal_Request__Sequence__init(mg400_msgs__action__MovJIO_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_SendGoal_Request * data = NULL;

  if (size) {
    data = (mg400_msgs__action__MovJIO_SendGoal_Request *)allocator.zero_allocate(size, sizeof(mg400_msgs__action__MovJIO_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__action__MovJIO_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__action__MovJIO_SendGoal_Request__fini(&data[i - 1]);
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
mg400_msgs__action__MovJIO_SendGoal_Request__Sequence__fini(mg400_msgs__action__MovJIO_SendGoal_Request__Sequence * array)
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
      mg400_msgs__action__MovJIO_SendGoal_Request__fini(&array->data[i]);
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

mg400_msgs__action__MovJIO_SendGoal_Request__Sequence *
mg400_msgs__action__MovJIO_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_SendGoal_Request__Sequence * array = (mg400_msgs__action__MovJIO_SendGoal_Request__Sequence *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__action__MovJIO_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__action__MovJIO_SendGoal_Request__Sequence__destroy(mg400_msgs__action__MovJIO_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__action__MovJIO_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__action__MovJIO_SendGoal_Request__Sequence__are_equal(const mg400_msgs__action__MovJIO_SendGoal_Request__Sequence * lhs, const mg400_msgs__action__MovJIO_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__action__MovJIO_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_SendGoal_Request__Sequence__copy(
  const mg400_msgs__action__MovJIO_SendGoal_Request__Sequence * input,
  mg400_msgs__action__MovJIO_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__action__MovJIO_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__action__MovJIO_SendGoal_Request * data =
      (mg400_msgs__action__MovJIO_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__action__MovJIO_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__action__MovJIO_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__action__MovJIO_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
mg400_msgs__action__MovJIO_SendGoal_Response__init(mg400_msgs__action__MovJIO_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    mg400_msgs__action__MovJIO_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
mg400_msgs__action__MovJIO_SendGoal_Response__fini(mg400_msgs__action__MovJIO_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
mg400_msgs__action__MovJIO_SendGoal_Response__are_equal(const mg400_msgs__action__MovJIO_SendGoal_Response * lhs, const mg400_msgs__action__MovJIO_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_SendGoal_Response__copy(
  const mg400_msgs__action__MovJIO_SendGoal_Response * input,
  mg400_msgs__action__MovJIO_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

mg400_msgs__action__MovJIO_SendGoal_Response *
mg400_msgs__action__MovJIO_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_SendGoal_Response * msg = (mg400_msgs__action__MovJIO_SendGoal_Response *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__action__MovJIO_SendGoal_Response));
  bool success = mg400_msgs__action__MovJIO_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__action__MovJIO_SendGoal_Response__destroy(mg400_msgs__action__MovJIO_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__action__MovJIO_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__action__MovJIO_SendGoal_Response__Sequence__init(mg400_msgs__action__MovJIO_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_SendGoal_Response * data = NULL;

  if (size) {
    data = (mg400_msgs__action__MovJIO_SendGoal_Response *)allocator.zero_allocate(size, sizeof(mg400_msgs__action__MovJIO_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__action__MovJIO_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__action__MovJIO_SendGoal_Response__fini(&data[i - 1]);
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
mg400_msgs__action__MovJIO_SendGoal_Response__Sequence__fini(mg400_msgs__action__MovJIO_SendGoal_Response__Sequence * array)
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
      mg400_msgs__action__MovJIO_SendGoal_Response__fini(&array->data[i]);
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

mg400_msgs__action__MovJIO_SendGoal_Response__Sequence *
mg400_msgs__action__MovJIO_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_SendGoal_Response__Sequence * array = (mg400_msgs__action__MovJIO_SendGoal_Response__Sequence *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__action__MovJIO_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__action__MovJIO_SendGoal_Response__Sequence__destroy(mg400_msgs__action__MovJIO_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__action__MovJIO_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__action__MovJIO_SendGoal_Response__Sequence__are_equal(const mg400_msgs__action__MovJIO_SendGoal_Response__Sequence * lhs, const mg400_msgs__action__MovJIO_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__action__MovJIO_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_SendGoal_Response__Sequence__copy(
  const mg400_msgs__action__MovJIO_SendGoal_Response__Sequence * input,
  mg400_msgs__action__MovJIO_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__action__MovJIO_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__action__MovJIO_SendGoal_Response * data =
      (mg400_msgs__action__MovJIO_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__action__MovJIO_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__action__MovJIO_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__action__MovJIO_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
mg400_msgs__action__MovJIO_GetResult_Request__init(mg400_msgs__action__MovJIO_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mg400_msgs__action__MovJIO_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
mg400_msgs__action__MovJIO_GetResult_Request__fini(mg400_msgs__action__MovJIO_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
mg400_msgs__action__MovJIO_GetResult_Request__are_equal(const mg400_msgs__action__MovJIO_GetResult_Request * lhs, const mg400_msgs__action__MovJIO_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_GetResult_Request__copy(
  const mg400_msgs__action__MovJIO_GetResult_Request * input,
  mg400_msgs__action__MovJIO_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

mg400_msgs__action__MovJIO_GetResult_Request *
mg400_msgs__action__MovJIO_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_GetResult_Request * msg = (mg400_msgs__action__MovJIO_GetResult_Request *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__action__MovJIO_GetResult_Request));
  bool success = mg400_msgs__action__MovJIO_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__action__MovJIO_GetResult_Request__destroy(mg400_msgs__action__MovJIO_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__action__MovJIO_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__action__MovJIO_GetResult_Request__Sequence__init(mg400_msgs__action__MovJIO_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_GetResult_Request * data = NULL;

  if (size) {
    data = (mg400_msgs__action__MovJIO_GetResult_Request *)allocator.zero_allocate(size, sizeof(mg400_msgs__action__MovJIO_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__action__MovJIO_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__action__MovJIO_GetResult_Request__fini(&data[i - 1]);
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
mg400_msgs__action__MovJIO_GetResult_Request__Sequence__fini(mg400_msgs__action__MovJIO_GetResult_Request__Sequence * array)
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
      mg400_msgs__action__MovJIO_GetResult_Request__fini(&array->data[i]);
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

mg400_msgs__action__MovJIO_GetResult_Request__Sequence *
mg400_msgs__action__MovJIO_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_GetResult_Request__Sequence * array = (mg400_msgs__action__MovJIO_GetResult_Request__Sequence *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__action__MovJIO_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__action__MovJIO_GetResult_Request__Sequence__destroy(mg400_msgs__action__MovJIO_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__action__MovJIO_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__action__MovJIO_GetResult_Request__Sequence__are_equal(const mg400_msgs__action__MovJIO_GetResult_Request__Sequence * lhs, const mg400_msgs__action__MovJIO_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__action__MovJIO_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_GetResult_Request__Sequence__copy(
  const mg400_msgs__action__MovJIO_GetResult_Request__Sequence * input,
  mg400_msgs__action__MovJIO_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__action__MovJIO_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__action__MovJIO_GetResult_Request * data =
      (mg400_msgs__action__MovJIO_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__action__MovJIO_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__action__MovJIO_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__action__MovJIO_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "mg400_msgs/action/detail/mov_jio__functions.h"

bool
mg400_msgs__action__MovJIO_GetResult_Response__init(mg400_msgs__action__MovJIO_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!mg400_msgs__action__MovJIO_Result__init(&msg->result)) {
    mg400_msgs__action__MovJIO_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
mg400_msgs__action__MovJIO_GetResult_Response__fini(mg400_msgs__action__MovJIO_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  mg400_msgs__action__MovJIO_Result__fini(&msg->result);
}

bool
mg400_msgs__action__MovJIO_GetResult_Response__are_equal(const mg400_msgs__action__MovJIO_GetResult_Response * lhs, const mg400_msgs__action__MovJIO_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!mg400_msgs__action__MovJIO_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_GetResult_Response__copy(
  const mg400_msgs__action__MovJIO_GetResult_Response * input,
  mg400_msgs__action__MovJIO_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!mg400_msgs__action__MovJIO_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

mg400_msgs__action__MovJIO_GetResult_Response *
mg400_msgs__action__MovJIO_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_GetResult_Response * msg = (mg400_msgs__action__MovJIO_GetResult_Response *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__action__MovJIO_GetResult_Response));
  bool success = mg400_msgs__action__MovJIO_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__action__MovJIO_GetResult_Response__destroy(mg400_msgs__action__MovJIO_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__action__MovJIO_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__action__MovJIO_GetResult_Response__Sequence__init(mg400_msgs__action__MovJIO_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_GetResult_Response * data = NULL;

  if (size) {
    data = (mg400_msgs__action__MovJIO_GetResult_Response *)allocator.zero_allocate(size, sizeof(mg400_msgs__action__MovJIO_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__action__MovJIO_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__action__MovJIO_GetResult_Response__fini(&data[i - 1]);
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
mg400_msgs__action__MovJIO_GetResult_Response__Sequence__fini(mg400_msgs__action__MovJIO_GetResult_Response__Sequence * array)
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
      mg400_msgs__action__MovJIO_GetResult_Response__fini(&array->data[i]);
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

mg400_msgs__action__MovJIO_GetResult_Response__Sequence *
mg400_msgs__action__MovJIO_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_GetResult_Response__Sequence * array = (mg400_msgs__action__MovJIO_GetResult_Response__Sequence *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__action__MovJIO_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__action__MovJIO_GetResult_Response__Sequence__destroy(mg400_msgs__action__MovJIO_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__action__MovJIO_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__action__MovJIO_GetResult_Response__Sequence__are_equal(const mg400_msgs__action__MovJIO_GetResult_Response__Sequence * lhs, const mg400_msgs__action__MovJIO_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__action__MovJIO_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_GetResult_Response__Sequence__copy(
  const mg400_msgs__action__MovJIO_GetResult_Response__Sequence * input,
  mg400_msgs__action__MovJIO_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__action__MovJIO_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__action__MovJIO_GetResult_Response * data =
      (mg400_msgs__action__MovJIO_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__action__MovJIO_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__action__MovJIO_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__action__MovJIO_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "mg400_msgs/action/detail/mov_jio__functions.h"

bool
mg400_msgs__action__MovJIO_FeedbackMessage__init(mg400_msgs__action__MovJIO_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mg400_msgs__action__MovJIO_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!mg400_msgs__action__MovJIO_Feedback__init(&msg->feedback)) {
    mg400_msgs__action__MovJIO_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
mg400_msgs__action__MovJIO_FeedbackMessage__fini(mg400_msgs__action__MovJIO_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  mg400_msgs__action__MovJIO_Feedback__fini(&msg->feedback);
}

bool
mg400_msgs__action__MovJIO_FeedbackMessage__are_equal(const mg400_msgs__action__MovJIO_FeedbackMessage * lhs, const mg400_msgs__action__MovJIO_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!mg400_msgs__action__MovJIO_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_FeedbackMessage__copy(
  const mg400_msgs__action__MovJIO_FeedbackMessage * input,
  mg400_msgs__action__MovJIO_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!mg400_msgs__action__MovJIO_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

mg400_msgs__action__MovJIO_FeedbackMessage *
mg400_msgs__action__MovJIO_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_FeedbackMessage * msg = (mg400_msgs__action__MovJIO_FeedbackMessage *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mg400_msgs__action__MovJIO_FeedbackMessage));
  bool success = mg400_msgs__action__MovJIO_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mg400_msgs__action__MovJIO_FeedbackMessage__destroy(mg400_msgs__action__MovJIO_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mg400_msgs__action__MovJIO_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mg400_msgs__action__MovJIO_FeedbackMessage__Sequence__init(mg400_msgs__action__MovJIO_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_FeedbackMessage * data = NULL;

  if (size) {
    data = (mg400_msgs__action__MovJIO_FeedbackMessage *)allocator.zero_allocate(size, sizeof(mg400_msgs__action__MovJIO_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mg400_msgs__action__MovJIO_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mg400_msgs__action__MovJIO_FeedbackMessage__fini(&data[i - 1]);
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
mg400_msgs__action__MovJIO_FeedbackMessage__Sequence__fini(mg400_msgs__action__MovJIO_FeedbackMessage__Sequence * array)
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
      mg400_msgs__action__MovJIO_FeedbackMessage__fini(&array->data[i]);
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

mg400_msgs__action__MovJIO_FeedbackMessage__Sequence *
mg400_msgs__action__MovJIO_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mg400_msgs__action__MovJIO_FeedbackMessage__Sequence * array = (mg400_msgs__action__MovJIO_FeedbackMessage__Sequence *)allocator.allocate(sizeof(mg400_msgs__action__MovJIO_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mg400_msgs__action__MovJIO_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mg400_msgs__action__MovJIO_FeedbackMessage__Sequence__destroy(mg400_msgs__action__MovJIO_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mg400_msgs__action__MovJIO_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mg400_msgs__action__MovJIO_FeedbackMessage__Sequence__are_equal(const mg400_msgs__action__MovJIO_FeedbackMessage__Sequence * lhs, const mg400_msgs__action__MovJIO_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mg400_msgs__action__MovJIO_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mg400_msgs__action__MovJIO_FeedbackMessage__Sequence__copy(
  const mg400_msgs__action__MovJIO_FeedbackMessage__Sequence * input,
  mg400_msgs__action__MovJIO_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mg400_msgs__action__MovJIO_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mg400_msgs__action__MovJIO_FeedbackMessage * data =
      (mg400_msgs__action__MovJIO_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mg400_msgs__action__MovJIO_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mg400_msgs__action__MovJIO_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mg400_msgs__action__MovJIO_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
