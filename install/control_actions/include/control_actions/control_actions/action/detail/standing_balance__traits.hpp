// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from control_actions:action/StandingBalance.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_ACTIONS__ACTION__DETAIL__STANDING_BALANCE__TRAITS_HPP_
#define CONTROL_ACTIONS__ACTION__DETAIL__STANDING_BALANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "control_actions/action/detail/standing_balance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace control_actions
{

namespace action
{

inline void to_flow_style_yaml(
  const StandingBalance_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: enable
  {
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StandingBalance_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StandingBalance_Goal & msg, bool use_flow_style = false)
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

}  // namespace control_actions

namespace rosidl_generator_traits
{

[[deprecated("use control_actions::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_actions::action::StandingBalance_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_actions::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_actions::action::to_yaml() instead")]]
inline std::string to_yaml(const control_actions::action::StandingBalance_Goal & msg)
{
  return control_actions::action::to_yaml(msg);
}

template<>
inline const char * data_type<control_actions::action::StandingBalance_Goal>()
{
  return "control_actions::action::StandingBalance_Goal";
}

template<>
inline const char * name<control_actions::action::StandingBalance_Goal>()
{
  return "control_actions/action/StandingBalance_Goal";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<control_actions::action::StandingBalance_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace control_actions
{

namespace action
{

inline void to_flow_style_yaml(
  const StandingBalance_Result & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StandingBalance_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StandingBalance_Result & msg, bool use_flow_style = false)
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

}  // namespace control_actions

namespace rosidl_generator_traits
{

[[deprecated("use control_actions::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_actions::action::StandingBalance_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_actions::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_actions::action::to_yaml() instead")]]
inline std::string to_yaml(const control_actions::action::StandingBalance_Result & msg)
{
  return control_actions::action::to_yaml(msg);
}

template<>
inline const char * data_type<control_actions::action::StandingBalance_Result>()
{
  return "control_actions::action::StandingBalance_Result";
}

template<>
inline const char * name<control_actions::action::StandingBalance_Result>()
{
  return "control_actions/action/StandingBalance_Result";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<control_actions::action::StandingBalance_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace control_actions
{

namespace action
{

inline void to_flow_style_yaml(
  const StandingBalance_Feedback & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StandingBalance_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StandingBalance_Feedback & msg, bool use_flow_style = false)
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

}  // namespace control_actions

namespace rosidl_generator_traits
{

[[deprecated("use control_actions::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_actions::action::StandingBalance_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_actions::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_actions::action::to_yaml() instead")]]
inline std::string to_yaml(const control_actions::action::StandingBalance_Feedback & msg)
{
  return control_actions::action::to_yaml(msg);
}

template<>
inline const char * data_type<control_actions::action::StandingBalance_Feedback>()
{
  return "control_actions::action::StandingBalance_Feedback";
}

template<>
inline const char * name<control_actions::action::StandingBalance_Feedback>()
{
  return "control_actions/action/StandingBalance_Feedback";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<control_actions::action::StandingBalance_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "control_actions/action/detail/standing_balance__traits.hpp"

namespace control_actions
{

namespace action
{

inline void to_flow_style_yaml(
  const StandingBalance_SendGoal_Request & msg,
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
  const StandingBalance_SendGoal_Request & msg,
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

inline std::string to_yaml(const StandingBalance_SendGoal_Request & msg, bool use_flow_style = false)
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

}  // namespace control_actions

namespace rosidl_generator_traits
{

[[deprecated("use control_actions::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_actions::action::StandingBalance_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_actions::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_actions::action::to_yaml() instead")]]
inline std::string to_yaml(const control_actions::action::StandingBalance_SendGoal_Request & msg)
{
  return control_actions::action::to_yaml(msg);
}

template<>
inline const char * data_type<control_actions::action::StandingBalance_SendGoal_Request>()
{
  return "control_actions::action::StandingBalance_SendGoal_Request";
}

template<>
inline const char * name<control_actions::action::StandingBalance_SendGoal_Request>()
{
  return "control_actions/action/StandingBalance_SendGoal_Request";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<control_actions::action::StandingBalance_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<control_actions::action::StandingBalance_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<control_actions::action::StandingBalance_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace control_actions
{

namespace action
{

inline void to_flow_style_yaml(
  const StandingBalance_SendGoal_Response & msg,
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
  const StandingBalance_SendGoal_Response & msg,
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

inline std::string to_yaml(const StandingBalance_SendGoal_Response & msg, bool use_flow_style = false)
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

}  // namespace control_actions

namespace rosidl_generator_traits
{

[[deprecated("use control_actions::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_actions::action::StandingBalance_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_actions::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_actions::action::to_yaml() instead")]]
inline std::string to_yaml(const control_actions::action::StandingBalance_SendGoal_Response & msg)
{
  return control_actions::action::to_yaml(msg);
}

template<>
inline const char * data_type<control_actions::action::StandingBalance_SendGoal_Response>()
{
  return "control_actions::action::StandingBalance_SendGoal_Response";
}

template<>
inline const char * name<control_actions::action::StandingBalance_SendGoal_Response>()
{
  return "control_actions/action/StandingBalance_SendGoal_Response";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<control_actions::action::StandingBalance_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<control_actions::action::StandingBalance_SendGoal>()
{
  return "control_actions::action::StandingBalance_SendGoal";
}

template<>
inline const char * name<control_actions::action::StandingBalance_SendGoal>()
{
  return "control_actions/action/StandingBalance_SendGoal";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<control_actions::action::StandingBalance_SendGoal_Request>::value &&
    has_fixed_size<control_actions::action::StandingBalance_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<control_actions::action::StandingBalance_SendGoal_Request>::value &&
    has_bounded_size<control_actions::action::StandingBalance_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<control_actions::action::StandingBalance_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<control_actions::action::StandingBalance_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<control_actions::action::StandingBalance_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace control_actions
{

namespace action
{

inline void to_flow_style_yaml(
  const StandingBalance_GetResult_Request & msg,
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
  const StandingBalance_GetResult_Request & msg,
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

inline std::string to_yaml(const StandingBalance_GetResult_Request & msg, bool use_flow_style = false)
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

}  // namespace control_actions

namespace rosidl_generator_traits
{

[[deprecated("use control_actions::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_actions::action::StandingBalance_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_actions::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_actions::action::to_yaml() instead")]]
inline std::string to_yaml(const control_actions::action::StandingBalance_GetResult_Request & msg)
{
  return control_actions::action::to_yaml(msg);
}

template<>
inline const char * data_type<control_actions::action::StandingBalance_GetResult_Request>()
{
  return "control_actions::action::StandingBalance_GetResult_Request";
}

template<>
inline const char * name<control_actions::action::StandingBalance_GetResult_Request>()
{
  return "control_actions/action/StandingBalance_GetResult_Request";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<control_actions::action::StandingBalance_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "control_actions/action/detail/standing_balance__traits.hpp"

namespace control_actions
{

namespace action
{

inline void to_flow_style_yaml(
  const StandingBalance_GetResult_Response & msg,
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
  const StandingBalance_GetResult_Response & msg,
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

inline std::string to_yaml(const StandingBalance_GetResult_Response & msg, bool use_flow_style = false)
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

}  // namespace control_actions

namespace rosidl_generator_traits
{

[[deprecated("use control_actions::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_actions::action::StandingBalance_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_actions::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_actions::action::to_yaml() instead")]]
inline std::string to_yaml(const control_actions::action::StandingBalance_GetResult_Response & msg)
{
  return control_actions::action::to_yaml(msg);
}

template<>
inline const char * data_type<control_actions::action::StandingBalance_GetResult_Response>()
{
  return "control_actions::action::StandingBalance_GetResult_Response";
}

template<>
inline const char * name<control_actions::action::StandingBalance_GetResult_Response>()
{
  return "control_actions/action/StandingBalance_GetResult_Response";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<control_actions::action::StandingBalance_Result>::value> {};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<control_actions::action::StandingBalance_Result>::value> {};

template<>
struct is_message<control_actions::action::StandingBalance_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<control_actions::action::StandingBalance_GetResult>()
{
  return "control_actions::action::StandingBalance_GetResult";
}

template<>
inline const char * name<control_actions::action::StandingBalance_GetResult>()
{
  return "control_actions/action/StandingBalance_GetResult";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<control_actions::action::StandingBalance_GetResult_Request>::value &&
    has_fixed_size<control_actions::action::StandingBalance_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<control_actions::action::StandingBalance_GetResult_Request>::value &&
    has_bounded_size<control_actions::action::StandingBalance_GetResult_Response>::value
  >
{
};

template<>
struct is_service<control_actions::action::StandingBalance_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<control_actions::action::StandingBalance_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<control_actions::action::StandingBalance_GetResult_Response>
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
// #include "control_actions/action/detail/standing_balance__traits.hpp"

namespace control_actions
{

namespace action
{

inline void to_flow_style_yaml(
  const StandingBalance_FeedbackMessage & msg,
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
  const StandingBalance_FeedbackMessage & msg,
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

inline std::string to_yaml(const StandingBalance_FeedbackMessage & msg, bool use_flow_style = false)
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

}  // namespace control_actions

namespace rosidl_generator_traits
{

[[deprecated("use control_actions::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_actions::action::StandingBalance_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_actions::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_actions::action::to_yaml() instead")]]
inline std::string to_yaml(const control_actions::action::StandingBalance_FeedbackMessage & msg)
{
  return control_actions::action::to_yaml(msg);
}

template<>
inline const char * data_type<control_actions::action::StandingBalance_FeedbackMessage>()
{
  return "control_actions::action::StandingBalance_FeedbackMessage";
}

template<>
inline const char * name<control_actions::action::StandingBalance_FeedbackMessage>()
{
  return "control_actions/action/StandingBalance_FeedbackMessage";
}

template<>
struct has_fixed_size<control_actions::action::StandingBalance_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<control_actions::action::StandingBalance_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<control_actions::action::StandingBalance_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<control_actions::action::StandingBalance_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<control_actions::action::StandingBalance_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<control_actions::action::StandingBalance>
  : std::true_type
{
};

template<>
struct is_action_goal<control_actions::action::StandingBalance_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<control_actions::action::StandingBalance_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<control_actions::action::StandingBalance_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // CONTROL_ACTIONS__ACTION__DETAIL__STANDING_BALANCE__TRAITS_HPP_
