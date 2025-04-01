// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from mg400_msgs:srv/GetPose.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mg400_msgs/srv/detail/get_pose__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mg400_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPose_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPose_Request_type_support_ids_t;

static const _GetPose_Request_type_support_ids_t _GetPose_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetPose_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPose_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPose_Request_type_support_symbol_names_t _GetPose_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mg400_msgs, srv, GetPose_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mg400_msgs, srv, GetPose_Request)),
  }
};

typedef struct _GetPose_Request_type_support_data_t
{
  void * data[2];
} _GetPose_Request_type_support_data_t;

static _GetPose_Request_type_support_data_t _GetPose_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPose_Request_message_typesupport_map = {
  2,
  "mg400_msgs",
  &_GetPose_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetPose_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetPose_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetPose_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPose_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mg400_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mg400_msgs::srv::GetPose_Request>()
{
  return &::mg400_msgs::srv::rosidl_typesupport_cpp::GetPose_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mg400_msgs, srv, GetPose_Request)() {
  return get_message_type_support_handle<mg400_msgs::srv::GetPose_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mg400_msgs/srv/detail/get_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mg400_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPose_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPose_Response_type_support_ids_t;

static const _GetPose_Response_type_support_ids_t _GetPose_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetPose_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPose_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPose_Response_type_support_symbol_names_t _GetPose_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mg400_msgs, srv, GetPose_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mg400_msgs, srv, GetPose_Response)),
  }
};

typedef struct _GetPose_Response_type_support_data_t
{
  void * data[2];
} _GetPose_Response_type_support_data_t;

static _GetPose_Response_type_support_data_t _GetPose_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPose_Response_message_typesupport_map = {
  2,
  "mg400_msgs",
  &_GetPose_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetPose_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetPose_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetPose_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPose_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mg400_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mg400_msgs::srv::GetPose_Response>()
{
  return &::mg400_msgs::srv::rosidl_typesupport_cpp::GetPose_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mg400_msgs, srv, GetPose_Response)() {
  return get_message_type_support_handle<mg400_msgs::srv::GetPose_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mg400_msgs/srv/detail/get_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mg400_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPose_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPose_type_support_ids_t;

static const _GetPose_type_support_ids_t _GetPose_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetPose_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPose_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPose_type_support_symbol_names_t _GetPose_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mg400_msgs, srv, GetPose)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mg400_msgs, srv, GetPose)),
  }
};

typedef struct _GetPose_type_support_data_t
{
  void * data[2];
} _GetPose_type_support_data_t;

static _GetPose_type_support_data_t _GetPose_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPose_service_typesupport_map = {
  2,
  "mg400_msgs",
  &_GetPose_service_typesupport_ids.typesupport_identifier[0],
  &_GetPose_service_typesupport_symbol_names.symbol_name[0],
  &_GetPose_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetPose_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPose_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mg400_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mg400_msgs::srv::GetPose>()
{
  return &::mg400_msgs::srv::rosidl_typesupport_cpp::GetPose_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, mg400_msgs, srv, GetPose)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<mg400_msgs::srv::GetPose>();
}

#ifdef __cplusplus
}
#endif
