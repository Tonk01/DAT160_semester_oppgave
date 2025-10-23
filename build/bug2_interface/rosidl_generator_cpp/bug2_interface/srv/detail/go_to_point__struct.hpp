// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bug2_interface:srv/GoToPoint.idl
// generated code does not contain a copyright notice

#ifndef BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__STRUCT_HPP_
#define BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__bug2_interface__srv__GoToPoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__bug2_interface__srv__GoToPoint_Request __declspec(deprecated)
#endif

namespace bug2_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GoToPoint_Request_
{
  using Type = GoToPoint_Request_<ContainerAllocator>;

  explicit GoToPoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->move_switch = false;
    }
  }

  explicit GoToPoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->move_switch = false;
    }
  }

  // field types and members
  using _move_switch_type =
    bool;
  _move_switch_type move_switch;
  using _target_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _target_position_type target_position;

  // setters for named parameter idiom
  Type & set__move_switch(
    const bool & _arg)
  {
    this->move_switch = _arg;
    return *this;
  }
  Type & set__target_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->target_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bug2_interface::srv::GoToPoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const bug2_interface::srv::GoToPoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bug2_interface::srv::GoToPoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bug2_interface::srv::GoToPoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bug2_interface::srv::GoToPoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bug2_interface::srv::GoToPoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bug2_interface::srv::GoToPoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bug2_interface::srv::GoToPoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bug2_interface::srv::GoToPoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bug2_interface::srv::GoToPoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bug2_interface__srv__GoToPoint_Request
    std::shared_ptr<bug2_interface::srv::GoToPoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bug2_interface__srv__GoToPoint_Request
    std::shared_ptr<bug2_interface::srv::GoToPoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToPoint_Request_ & other) const
  {
    if (this->move_switch != other.move_switch) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToPoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToPoint_Request_

// alias to use template instance with default allocator
using GoToPoint_Request =
  bug2_interface::srv::GoToPoint_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace bug2_interface


#ifndef _WIN32
# define DEPRECATED__bug2_interface__srv__GoToPoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__bug2_interface__srv__GoToPoint_Response __declspec(deprecated)
#endif

namespace bug2_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GoToPoint_Response_
{
  using Type = GoToPoint_Response_<ContainerAllocator>;

  explicit GoToPoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit GoToPoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bug2_interface::srv::GoToPoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const bug2_interface::srv::GoToPoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bug2_interface::srv::GoToPoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bug2_interface::srv::GoToPoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bug2_interface::srv::GoToPoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bug2_interface::srv::GoToPoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bug2_interface::srv::GoToPoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bug2_interface::srv::GoToPoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bug2_interface::srv::GoToPoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bug2_interface::srv::GoToPoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bug2_interface__srv__GoToPoint_Response
    std::shared_ptr<bug2_interface::srv::GoToPoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bug2_interface__srv__GoToPoint_Response
    std::shared_ptr<bug2_interface::srv::GoToPoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToPoint_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToPoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToPoint_Response_

// alias to use template instance with default allocator
using GoToPoint_Response =
  bug2_interface::srv::GoToPoint_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace bug2_interface

namespace bug2_interface
{

namespace srv
{

struct GoToPoint
{
  using Request = bug2_interface::srv::GoToPoint_Request;
  using Response = bug2_interface::srv::GoToPoint_Response;
};

}  // namespace srv

}  // namespace bug2_interface

#endif  // BUG2_INTERFACE__SRV__DETAIL__GO_TO_POINT__STRUCT_HPP_
