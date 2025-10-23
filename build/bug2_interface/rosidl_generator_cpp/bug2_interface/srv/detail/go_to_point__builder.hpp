// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bug2_interface:srv/GoToPoint.idl
// generated code does not contain a copyright notice

#ifndef BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__BUILDER_HPP_
#define BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__BUILDER_HPP_

#include "bug2_interface/srv/detail/go_to_point__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace bug2_interface
{

namespace srv
{

namespace builder
{

class Init_GoToPoint_Request_target_position
{
public:
  explicit Init_GoToPoint_Request_target_position(::bug2_interface::srv::GoToPoint_Request & msg)
  : msg_(msg)
  {}
  ::bug2_interface::srv::GoToPoint_Request target_position(::bug2_interface::srv::GoToPoint_Request::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bug2_interface::srv::GoToPoint_Request msg_;
};

class Init_GoToPoint_Request_move_switch
{
public:
  Init_GoToPoint_Request_move_switch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToPoint_Request_target_position move_switch(::bug2_interface::srv::GoToPoint_Request::_move_switch_type arg)
  {
    msg_.move_switch = std::move(arg);
    return Init_GoToPoint_Request_target_position(msg_);
  }

private:
  ::bug2_interface::srv::GoToPoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::bug2_interface::srv::GoToPoint_Request>()
{
  return bug2_interface::srv::builder::Init_GoToPoint_Request_move_switch();
}

}  // namespace bug2_interface


namespace bug2_interface
{

namespace srv
{

namespace builder
{

class Init_GoToPoint_Response_success
{
public:
  Init_GoToPoint_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::bug2_interface::srv::GoToPoint_Response success(::bug2_interface::srv::GoToPoint_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bug2_interface::srv::GoToPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::bug2_interface::srv::GoToPoint_Response>()
{
  return bug2_interface::srv::builder::Init_GoToPoint_Response_success();
}

}  // namespace bug2_interface

#endif  // BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__BUILDER_HPP_
