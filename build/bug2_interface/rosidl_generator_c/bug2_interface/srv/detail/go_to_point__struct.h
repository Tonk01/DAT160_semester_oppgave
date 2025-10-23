// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bug2_interface:srv/GoToPoint.idl
// generated code does not contain a copyright notice

#ifndef BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__STRUCT_H_
#define BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in srv/GoToPoint in the package bug2_interface.
typedef struct bug2_interface__srv__GoToPoint_Request
{
  bool move_switch;
  geometry_msgs__msg__Point target_position;
} bug2_interface__srv__GoToPoint_Request;

// Struct for a sequence of bug2_interface__srv__GoToPoint_Request.
typedef struct bug2_interface__srv__GoToPoint_Request__Sequence
{
  bug2_interface__srv__GoToPoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bug2_interface__srv__GoToPoint_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/GoToPoint in the package bug2_interface.
typedef struct bug2_interface__srv__GoToPoint_Response
{
  bool success;
} bug2_interface__srv__GoToPoint_Response;

// Struct for a sequence of bug2_interface__srv__GoToPoint_Response.
typedef struct bug2_interface__srv__GoToPoint_Response__Sequence
{
  bug2_interface__srv__GoToPoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bug2_interface__srv__GoToPoint_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__STRUCT_H_
