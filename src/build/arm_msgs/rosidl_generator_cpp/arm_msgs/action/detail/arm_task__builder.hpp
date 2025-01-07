// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arm_msgs:action/ArmTask.idl
// generated code does not contain a copyright notice

#ifndef ARM_MSGS__ACTION__DETAIL__ARM_TASK__BUILDER_HPP_
#define ARM_MSGS__ACTION__DETAIL__ARM_TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arm_msgs/action/detail/arm_task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arm_msgs
{

namespace action
{

namespace builder
{

class Init_ArmTask_Goal_task_number
{
public:
  Init_ArmTask_Goal_task_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arm_msgs::action::ArmTask_Goal task_number(::arm_msgs::action::ArmTask_Goal::_task_number_type arg)
  {
    msg_.task_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_msgs::action::ArmTask_Goal>()
{
  return arm_msgs::action::builder::Init_ArmTask_Goal_task_number();
}

}  // namespace arm_msgs


namespace arm_msgs
{

namespace action
{

namespace builder
{

class Init_ArmTask_Result_success
{
public:
  Init_ArmTask_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arm_msgs::action::ArmTask_Result success(::arm_msgs::action::ArmTask_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_msgs::action::ArmTask_Result>()
{
  return arm_msgs::action::builder::Init_ArmTask_Result_success();
}

}  // namespace arm_msgs


namespace arm_msgs
{

namespace action
{

namespace builder
{

class Init_ArmTask_Feedback_percentage
{
public:
  Init_ArmTask_Feedback_percentage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arm_msgs::action::ArmTask_Feedback percentage(::arm_msgs::action::ArmTask_Feedback::_percentage_type arg)
  {
    msg_.percentage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_msgs::action::ArmTask_Feedback>()
{
  return arm_msgs::action::builder::Init_ArmTask_Feedback_percentage();
}

}  // namespace arm_msgs


namespace arm_msgs
{

namespace action
{

namespace builder
{

class Init_ArmTask_SendGoal_Request_goal
{
public:
  explicit Init_ArmTask_SendGoal_Request_goal(::arm_msgs::action::ArmTask_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::arm_msgs::action::ArmTask_SendGoal_Request goal(::arm_msgs::action::ArmTask_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_SendGoal_Request msg_;
};

class Init_ArmTask_SendGoal_Request_goal_id
{
public:
  Init_ArmTask_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmTask_SendGoal_Request_goal goal_id(::arm_msgs::action::ArmTask_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ArmTask_SendGoal_Request_goal(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_msgs::action::ArmTask_SendGoal_Request>()
{
  return arm_msgs::action::builder::Init_ArmTask_SendGoal_Request_goal_id();
}

}  // namespace arm_msgs


namespace arm_msgs
{

namespace action
{

namespace builder
{

class Init_ArmTask_SendGoal_Response_stamp
{
public:
  explicit Init_ArmTask_SendGoal_Response_stamp(::arm_msgs::action::ArmTask_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::arm_msgs::action::ArmTask_SendGoal_Response stamp(::arm_msgs::action::ArmTask_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_SendGoal_Response msg_;
};

class Init_ArmTask_SendGoal_Response_accepted
{
public:
  Init_ArmTask_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmTask_SendGoal_Response_stamp accepted(::arm_msgs::action::ArmTask_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ArmTask_SendGoal_Response_stamp(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_msgs::action::ArmTask_SendGoal_Response>()
{
  return arm_msgs::action::builder::Init_ArmTask_SendGoal_Response_accepted();
}

}  // namespace arm_msgs


namespace arm_msgs
{

namespace action
{

namespace builder
{

class Init_ArmTask_GetResult_Request_goal_id
{
public:
  Init_ArmTask_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arm_msgs::action::ArmTask_GetResult_Request goal_id(::arm_msgs::action::ArmTask_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_msgs::action::ArmTask_GetResult_Request>()
{
  return arm_msgs::action::builder::Init_ArmTask_GetResult_Request_goal_id();
}

}  // namespace arm_msgs


namespace arm_msgs
{

namespace action
{

namespace builder
{

class Init_ArmTask_GetResult_Response_result
{
public:
  explicit Init_ArmTask_GetResult_Response_result(::arm_msgs::action::ArmTask_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::arm_msgs::action::ArmTask_GetResult_Response result(::arm_msgs::action::ArmTask_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_GetResult_Response msg_;
};

class Init_ArmTask_GetResult_Response_status
{
public:
  Init_ArmTask_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmTask_GetResult_Response_result status(::arm_msgs::action::ArmTask_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ArmTask_GetResult_Response_result(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_msgs::action::ArmTask_GetResult_Response>()
{
  return arm_msgs::action::builder::Init_ArmTask_GetResult_Response_status();
}

}  // namespace arm_msgs


namespace arm_msgs
{

namespace action
{

namespace builder
{

class Init_ArmTask_FeedbackMessage_feedback
{
public:
  explicit Init_ArmTask_FeedbackMessage_feedback(::arm_msgs::action::ArmTask_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::arm_msgs::action::ArmTask_FeedbackMessage feedback(::arm_msgs::action::ArmTask_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_FeedbackMessage msg_;
};

class Init_ArmTask_FeedbackMessage_goal_id
{
public:
  Init_ArmTask_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmTask_FeedbackMessage_feedback goal_id(::arm_msgs::action::ArmTask_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ArmTask_FeedbackMessage_feedback(msg_);
  }

private:
  ::arm_msgs::action::ArmTask_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_msgs::action::ArmTask_FeedbackMessage>()
{
  return arm_msgs::action::builder::Init_ArmTask_FeedbackMessage_goal_id();
}

}  // namespace arm_msgs

#endif  // ARM_MSGS__ACTION__DETAIL__ARM_TASK__BUILDER_HPP_
