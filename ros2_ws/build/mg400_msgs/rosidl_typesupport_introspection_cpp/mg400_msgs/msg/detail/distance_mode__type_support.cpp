// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mg400_msgs:msg/DistanceMode.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mg400_msgs/msg/detail/distance_mode__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mg400_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DistanceMode_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mg400_msgs::msg::DistanceMode(_init);
}

void DistanceMode_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mg400_msgs::msg::DistanceMode *>(message_memory);
  typed_message->~DistanceMode();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DistanceMode_message_member_array[1] = {
  {
    "mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mg400_msgs::msg::DistanceMode, mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DistanceMode_message_members = {
  "mg400_msgs::msg",  // message namespace
  "DistanceMode",  // message name
  1,  // number of fields
  sizeof(mg400_msgs::msg::DistanceMode),
  DistanceMode_message_member_array,  // message members
  DistanceMode_init_function,  // function to initialize message memory (memory has to be allocated)
  DistanceMode_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DistanceMode_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DistanceMode_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mg400_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mg400_msgs::msg::DistanceMode>()
{
  return &::mg400_msgs::msg::rosidl_typesupport_introspection_cpp::DistanceMode_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mg400_msgs, msg, DistanceMode)() {
  return &::mg400_msgs::msg::rosidl_typesupport_introspection_cpp::DistanceMode_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
