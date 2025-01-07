// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arm_msgs:action/ArmTask.idl
// generated code does not contain a copyright notice

#ifndef ARM_MSGS__ACTION__DETAIL__ARM_TASK__TRAITS_HPP_
#define ARM_MSGS__ACTION__DETAIL__ARM_TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arm_msgs/action/detail/arm_task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arm_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ArmTask_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_number
  {
    out << "task_number: ";
    rosidl_generator_traits::value_to_yaml(msg.task_number, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmTask_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_number: ";
    rosidl_generator_traits::value_to_yaml(msg.task_number, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmTask_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arm_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_msgs::action::ArmTask_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_msgs::action::ArmTask_Goal & msg)
{
  return arm_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_msgs::action::ArmTask_Goal>()
{
  return "arm_msgs::action::ArmTask_Goal";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_Goal>()
{
  return "arm_msgs/action/ArmTask_Goal";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<arm_msgs::action::ArmTask_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace arm_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ArmTask_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmTask_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmTask_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arm_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_msgs::action::ArmTask_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_msgs::action::ArmTask_Result & msg)
{
  return arm_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_msgs::action::ArmTask_Result>()
{
  return "arm_msgs::action::ArmTask_Result";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_Result>()
{
  return "arm_msgs/action/ArmTask_Result";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<arm_msgs::action::ArmTask_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace arm_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ArmTask_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: percentage
  {
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmTask_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmTask_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arm_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_msgs::action::ArmTask_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_msgs::action::ArmTask_Feedback & msg)
{
  return arm_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_msgs::action::ArmTask_Feedback>()
{
  return "arm_msgs::action::ArmTask_Feedback";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_Feedback>()
{
  return "arm_msgs/action/ArmTask_Feedback";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<arm_msgs::action::ArmTask_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "arm_msgs/action/detail/arm_task__traits.hpp"

namespace arm_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ArmTask_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmTask_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmTask_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arm_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_msgs::action::ArmTask_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_msgs::action::ArmTask_SendGoal_Request & msg)
{
  return arm_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_msgs::action::ArmTask_SendGoal_Request>()
{
  return "arm_msgs::action::ArmTask_SendGoal_Request";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_SendGoal_Request>()
{
  return "arm_msgs/action/ArmTask_SendGoal_Request";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<arm_msgs::action::ArmTask_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<arm_msgs::action::ArmTask_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_msgs::action::ArmTask_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace arm_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ArmTask_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmTask_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmTask_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arm_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_msgs::action::ArmTask_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_msgs::action::ArmTask_SendGoal_Response & msg)
{
  return arm_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_msgs::action::ArmTask_SendGoal_Response>()
{
  return "arm_msgs::action::ArmTask_SendGoal_Response";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_SendGoal_Response>()
{
  return "arm_msgs/action/ArmTask_SendGoal_Response";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<arm_msgs::action::ArmTask_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_msgs::action::ArmTask_SendGoal>()
{
  return "arm_msgs::action::ArmTask_SendGoal";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_SendGoal>()
{
  return "arm_msgs/action/ArmTask_SendGoal";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_msgs::action::ArmTask_SendGoal_Request>::value &&
    has_fixed_size<arm_msgs::action::ArmTask_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_msgs::action::ArmTask_SendGoal_Request>::value &&
    has_bounded_size<arm_msgs::action::ArmTask_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<arm_msgs::action::ArmTask_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<arm_msgs::action::ArmTask_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_msgs::action::ArmTask_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace arm_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ArmTask_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmTask_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmTask_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arm_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_msgs::action::ArmTask_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_msgs::action::ArmTask_GetResult_Request & msg)
{
  return arm_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_msgs::action::ArmTask_GetResult_Request>()
{
  return "arm_msgs::action::ArmTask_GetResult_Request";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_GetResult_Request>()
{
  return "arm_msgs/action/ArmTask_GetResult_Request";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_msgs::action::ArmTask_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "arm_msgs/action/detail/arm_task__traits.hpp"

namespace arm_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ArmTask_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmTask_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmTask_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arm_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_msgs::action::ArmTask_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_msgs::action::ArmTask_GetResult_Response & msg)
{
  return arm_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_msgs::action::ArmTask_GetResult_Response>()
{
  return "arm_msgs::action::ArmTask_GetResult_Response";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_GetResult_Response>()
{
  return "arm_msgs/action/ArmTask_GetResult_Response";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<arm_msgs::action::ArmTask_Result>::value> {};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<arm_msgs::action::ArmTask_Result>::value> {};

template<>
struct is_message<arm_msgs::action::ArmTask_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_msgs::action::ArmTask_GetResult>()
{
  return "arm_msgs::action::ArmTask_GetResult";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_GetResult>()
{
  return "arm_msgs/action/ArmTask_GetResult";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_msgs::action::ArmTask_GetResult_Request>::value &&
    has_fixed_size<arm_msgs::action::ArmTask_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_msgs::action::ArmTask_GetResult_Request>::value &&
    has_bounded_size<arm_msgs::action::ArmTask_GetResult_Response>::value
  >
{
};

template<>
struct is_service<arm_msgs::action::ArmTask_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<arm_msgs::action::ArmTask_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_msgs::action::ArmTask_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "arm_msgs/action/detail/arm_task__traits.hpp"

namespace arm_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ArmTask_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmTask_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmTask_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arm_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_msgs::action::ArmTask_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_msgs::action::ArmTask_FeedbackMessage & msg)
{
  return arm_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_msgs::action::ArmTask_FeedbackMessage>()
{
  return "arm_msgs::action::ArmTask_FeedbackMessage";
}

template<>
inline const char * name<arm_msgs::action::ArmTask_FeedbackMessage>()
{
  return "arm_msgs/action/ArmTask_FeedbackMessage";
}

template<>
struct has_fixed_size<arm_msgs::action::ArmTask_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<arm_msgs::action::ArmTask_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_msgs::action::ArmTask_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<arm_msgs::action::ArmTask_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_msgs::action::ArmTask_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<arm_msgs::action::ArmTask>
  : std::true_type
{
};

template<>
struct is_action_goal<arm_msgs::action::ArmTask_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<arm_msgs::action::ArmTask_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<arm_msgs::action::ArmTask_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ARM_MSGS__ACTION__DETAIL__ARM_TASK__TRAITS_HPP_
