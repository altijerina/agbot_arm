// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arm_msgs:srv/QuaternionToEuler.idl
// generated code does not contain a copyright notice

#ifndef ARM_MSGS__SRV__DETAIL__QUATERNION_TO_EULER__STRUCT_H_
#define ARM_MSGS__SRV__DETAIL__QUATERNION_TO_EULER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/QuaternionToEuler in the package arm_msgs.
typedef struct arm_msgs__srv__QuaternionToEuler_Request
{
  double x;
  double y;
  double z;
  double w;
} arm_msgs__srv__QuaternionToEuler_Request;

// Struct for a sequence of arm_msgs__srv__QuaternionToEuler_Request.
typedef struct arm_msgs__srv__QuaternionToEuler_Request__Sequence
{
  arm_msgs__srv__QuaternionToEuler_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_msgs__srv__QuaternionToEuler_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/QuaternionToEuler in the package arm_msgs.
typedef struct arm_msgs__srv__QuaternionToEuler_Response
{
  double roll;
  double pitch;
  double yaw;
} arm_msgs__srv__QuaternionToEuler_Response;

// Struct for a sequence of arm_msgs__srv__QuaternionToEuler_Response.
typedef struct arm_msgs__srv__QuaternionToEuler_Response__Sequence
{
  arm_msgs__srv__QuaternionToEuler_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_msgs__srv__QuaternionToEuler_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARM_MSGS__SRV__DETAIL__QUATERNION_TO_EULER__STRUCT_H_
