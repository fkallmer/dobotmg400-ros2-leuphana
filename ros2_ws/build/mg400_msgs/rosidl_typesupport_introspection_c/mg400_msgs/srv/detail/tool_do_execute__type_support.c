// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mg400_msgs:srv/ToolDOExecute.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mg400_msgs/srv/detail/tool_do_execute__rosidl_typesupport_introspection_c.h"
#include "mg400_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mg400_msgs/srv/detail/tool_do_execute__functions.h"
#include "mg400_msgs/srv/detail/tool_do_execute__struct.h"


// Include directives for member types
// Member `index`
#include "mg400_msgs/msg/tool_do_index.h"
// Member `index`
#include "mg400_msgs/msg/detail/tool_do_index__rosidl_typesupport_introspection_c.h"
// Member `status`
#include "mg400_msgs/msg/do_status.h"
// Member `status`
#include "mg400_msgs/msg/detail/do_status__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mg400_msgs__srv__ToolDOExecute_Request__init(message_memory);
}

void mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_fini_function(void * message_memory)
{
  mg400_msgs__srv__ToolDOExecute_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_member_array[2] = {
  {
    "index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mg400_msgs__srv__ToolDOExecute_Request, index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mg400_msgs__srv__ToolDOExecute_Request, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_members = {
  "mg400_msgs__srv",  // message namespace
  "ToolDOExecute_Request",  // message name
  2,  // number of fields
  sizeof(mg400_msgs__srv__ToolDOExecute_Request),
  mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_member_array,  // message members
  mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_type_support_handle = {
  0,
  &mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mg400_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, srv, ToolDOExecute_Request)() {
  mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, msg, ToolDOIndex)();
  mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, msg, DOStatus)();
  if (!mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_type_support_handle.typesupport_identifier) {
    mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mg400_msgs__srv__ToolDOExecute_Request__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mg400_msgs/srv/detail/tool_do_execute__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mg400_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mg400_msgs/srv/detail/tool_do_execute__functions.h"
// already included above
// #include "mg400_msgs/srv/detail/tool_do_execute__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mg400_msgs__srv__ToolDOExecute_Response__init(message_memory);
}

void mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_fini_function(void * message_memory)
{
  mg400_msgs__srv__ToolDOExecute_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_member_array[2] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mg400_msgs__srv__ToolDOExecute_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mg400_msgs__srv__ToolDOExecute_Response, error_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_members = {
  "mg400_msgs__srv",  // message namespace
  "ToolDOExecute_Response",  // message name
  2,  // number of fields
  sizeof(mg400_msgs__srv__ToolDOExecute_Response),
  mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_member_array,  // message members
  mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_type_support_handle = {
  0,
  &mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mg400_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, srv, ToolDOExecute_Response)() {
  if (!mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_type_support_handle.typesupport_identifier) {
    mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mg400_msgs__srv__ToolDOExecute_Response__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mg400_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mg400_msgs/srv/detail/tool_do_execute__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_service_members = {
  "mg400_msgs__srv",  // service namespace
  "ToolDOExecute",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_Request_message_type_support_handle,
  NULL  // response message
  // mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_Response_message_type_support_handle
};

static rosidl_service_type_support_t mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_service_type_support_handle = {
  0,
  &mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, srv, ToolDOExecute_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, srv, ToolDOExecute_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mg400_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, srv, ToolDOExecute)() {
  if (!mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_service_type_support_handle.typesupport_identifier) {
    mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, srv, ToolDOExecute_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mg400_msgs, srv, ToolDOExecute_Response)()->data;
  }

  return &mg400_msgs__srv__detail__tool_do_execute__rosidl_typesupport_introspection_c__ToolDOExecute_service_type_support_handle;
}
