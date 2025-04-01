// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mg400_msgs:srv/AccJ.idl
// generated code does not contain a copyright notice

#ifndef MG400_MSGS__SRV__DETAIL__ACC_J__STRUCT_H_
#define MG400_MSGS__SRV__DETAIL__ACC_J__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/AccJ in the package mg400_msgs.
typedef struct mg400_msgs__srv__AccJ_Request
{
  uint8_t r;
} mg400_msgs__srv__AccJ_Request;

// Struct for a sequence of mg400_msgs__srv__AccJ_Request.
typedef struct mg400_msgs__srv__AccJ_Request__Sequence
{
  mg400_msgs__srv__AccJ_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mg400_msgs__srv__AccJ_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/AccJ in the package mg400_msgs.
typedef struct mg400_msgs__srv__AccJ_Response
{
  bool result;
  int32_t error_id;
} mg400_msgs__srv__AccJ_Response;

// Struct for a sequence of mg400_msgs__srv__AccJ_Response.
typedef struct mg400_msgs__srv__AccJ_Response__Sequence
{
  mg400_msgs__srv__AccJ_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mg400_msgs__srv__AccJ_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MG400_MSGS__SRV__DETAIL__ACC_J__STRUCT_H_
