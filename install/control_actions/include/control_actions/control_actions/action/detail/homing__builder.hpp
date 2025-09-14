// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from control_actions:action/Homing.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_ACTIONS__ACTION__DETAIL__HOMING__BUILDER_HPP_
#define CONTROL_ACTIONS__ACTION__DETAIL__HOMING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "control_actions/action/detail/homing__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace control_actions
{

namespace action
{

namespace builder
{

class Init_Homing_Goal_duration
{
public:
  Init_Homing_Goal_duration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::control_actions::action::Homing_Goal duration(::control_actions::action::Homing_Goal::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_actions::action::Homing_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_actions::action::Homing_Goal>()
{
  return control_actions::action::builder::Init_Homing_Goal_duration();
}

}  // namespace control_actions


namespace control_actions
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_actions::action::Homing_Result>()
{
  return ::control_actions::action::Homing_Result(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace control_actions


namespace control_actions
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_actions::action::Homing_Feedback>()
{
  return ::control_actions::action::Homing_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace control_actions


namespace control_actions
{

namespace action
{

namespace builder
{

class Init_Homing_SendGoal_Request_goal
{
public:
  explicit Init_Homing_SendGoal_Request_goal(::control_actions::action::Homing_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::control_actions::action::Homing_SendGoal_Request goal(::control_actions::action::Homing_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_actions::action::Homing_SendGoal_Request msg_;
};

class Init_Homing_SendGoal_Request_goal_id
{
public:
  Init_Homing_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Homing_SendGoal_Request_goal goal_id(::control_actions::action::Homing_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Homing_SendGoal_Request_goal(msg_);
  }

private:
  ::control_actions::action::Homing_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_actions::action::Homing_SendGoal_Request>()
{
  return control_actions::action::builder::Init_Homing_SendGoal_Request_goal_id();
}

}  // namespace control_actions


namespace control_actions
{

namespace action
{

namespace builder
{

class Init_Homing_SendGoal_Response_stamp
{
public:
  explicit Init_Homing_SendGoal_Response_stamp(::control_actions::action::Homing_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::control_actions::action::Homing_SendGoal_Response stamp(::control_actions::action::Homing_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_actions::action::Homing_SendGoal_Response msg_;
};

class Init_Homing_SendGoal_Response_accepted
{
public:
  Init_Homing_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Homing_SendGoal_Response_stamp accepted(::control_actions::action::Homing_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Homing_SendGoal_Response_stamp(msg_);
  }

private:
  ::control_actions::action::Homing_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_actions::action::Homing_SendGoal_Response>()
{
  return control_actions::action::builder::Init_Homing_SendGoal_Response_accepted();
}

}  // namespace control_actions


namespace control_actions
{

namespace action
{

namespace builder
{

class Init_Homing_GetResult_Request_goal_id
{
public:
  Init_Homing_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::control_actions::action::Homing_GetResult_Request goal_id(::control_actions::action::Homing_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_actions::action::Homing_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_actions::action::Homing_GetResult_Request>()
{
  return control_actions::action::builder::Init_Homing_GetResult_Request_goal_id();
}

}  // namespace control_actions


namespace control_actions
{

namespace action
{

namespace builder
{

class Init_Homing_GetResult_Response_result
{
public:
  explicit Init_Homing_GetResult_Response_result(::control_actions::action::Homing_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::control_actions::action::Homing_GetResult_Response result(::control_actions::action::Homing_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_actions::action::Homing_GetResult_Response msg_;
};

class Init_Homing_GetResult_Response_status
{
public:
  Init_Homing_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Homing_GetResult_Response_result status(::control_actions::action::Homing_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Homing_GetResult_Response_result(msg_);
  }

private:
  ::control_actions::action::Homing_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_actions::action::Homing_GetResult_Response>()
{
  return control_actions::action::builder::Init_Homing_GetResult_Response_status();
}

}  // namespace control_actions


namespace control_actions
{

namespace action
{

namespace builder
{

class Init_Homing_FeedbackMessage_feedback
{
public:
  explicit Init_Homing_FeedbackMessage_feedback(::control_actions::action::Homing_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::control_actions::action::Homing_FeedbackMessage feedback(::control_actions::action::Homing_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_actions::action::Homing_FeedbackMessage msg_;
};

class Init_Homing_FeedbackMessage_goal_id
{
public:
  Init_Homing_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Homing_FeedbackMessage_feedback goal_id(::control_actions::action::Homing_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Homing_FeedbackMessage_feedback(msg_);
  }

private:
  ::control_actions::action::Homing_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_actions::action::Homing_FeedbackMessage>()
{
  return control_actions::action::builder::Init_Homing_FeedbackMessage_goal_id();
}

}  // namespace control_actions

#endif  // CONTROL_ACTIONS__ACTION__DETAIL__HOMING__BUILDER_HPP_
