// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mg400_msgs:srv/RobotMode.idl
// generated code does not contain a copyright notice

#ifndef MG400_MSGS__SRV__DETAIL__ROBOT_MODE__STRUCT_H_
#define MG400_MSGS__SRV__DETAIL__ROBOT_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RobotMode in the package mg400_msgs.
typedef struct mg400_msgs__srv__RobotMode_Request
{
  uint8_t structure_needs_at_least_one_member;
} mg400_msgs__srv__RobotMode_Request;

// Struct for a sequence of mg400_msgs__srv__RobotMode_Request.
typedef struct mg400_msgs__srv__RobotMode_Request__Sequence
{
  mg400_msgs__srv__RobotMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mg400_msgs__srv__RobotMode_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'robot_mode'
#include "mg400_msgs/msg/detail/robot_mode__struct.h"

/// Struct defined in srv/RobotMode in the package mg400_msgs.
typedef struct mg400_msgs__srv__RobotMode_Response
{
  mg400_msgs__msg__RobotMode robot_mode;
  int32_t error_id;
} mg400_msgs__srv__RobotMode_Response;

// Struct for a sequence of mg400_msgs__srv__RobotMode_Response.
typedef struct mg400_msgs__srv__RobotMode_Response__Sequence
{
  mg400_msgs__srv__RobotMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mg400_msgs__srv__RobotMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MG400_MSGS__SRV__DETAIL__ROBOT_MODE__STRUCT_H_
