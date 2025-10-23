// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bug2_interface:srv/GoToPoint.idl
// generated code does not contain a copyright notice

#ifndef BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__TRAITS_HPP_
#define BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__TRAITS_HPP_

#include "bug2_interface/srv/detail/go_to_point__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bug2_interface::srv::GoToPoint_Request>()
{
  return "bug2_interface::srv::GoToPoint_Request";
}

template<>
inline const char * name<bug2_interface::srv::GoToPoint_Request>()
{
  return "bug2_interface/srv/GoToPoint_Request";
}

template<>
struct has_fixed_size<bug2_interface::srv::GoToPoint_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<bug2_interface::srv::GoToPoint_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<bug2_interface::srv::GoToPoint_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bug2_interface::srv::GoToPoint_Response>()
{
  return "bug2_interface::srv::GoToPoint_Response";
}

template<>
inline const char * name<bug2_interface::srv::GoToPoint_Response>()
{
  return "bug2_interface/srv/GoToPoint_Response";
}

template<>
struct has_fixed_size<bug2_interface::srv::GoToPoint_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<bug2_interface::srv::GoToPoint_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<bug2_interface::srv::GoToPoint_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bug2_interface::srv::GoToPoint>()
{
  return "bug2_interface::srv::GoToPoint";
}

template<>
inline const char * name<bug2_interface::srv::GoToPoint>()
{
  return "bug2_interface/srv/GoToPoint";
}

template<>
struct has_fixed_size<bug2_interface::srv::GoToPoint>
  : std::integral_constant<
    bool,
    has_fixed_size<bug2_interface::srv::GoToPoint_Request>::value &&
    has_fixed_size<bug2_interface::srv::GoToPoint_Response>::value
  >
{
};

template<>
struct has_bounded_size<bug2_interface::srv::GoToPoint>
  : std::integral_constant<
    bool,
    has_bounded_size<bug2_interface::srv::GoToPoint_Request>::value &&
    has_bounded_size<bug2_interface::srv::GoToPoint_Response>::value
  >
{
};

template<>
struct is_service<bug2_interface::srv::GoToPoint>
  : std::true_type
{
};

template<>
struct is_service_request<bug2_interface::srv::GoToPoint_Request>
  : std::true_type
{
};

template<>
struct is_service_response<bug2_interface::srv::GoToPoint_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__TRAITS_HPP_
