// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from control_actions:action/InitialStanding.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_ACTIONS__ACTION__DETAIL__INITIAL_STANDING__STRUCT_H_
#define CONTROL_ACTIONS__ACTION__DETAIL__INITIAL_STANDING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/InitialStanding in the package control_actions.
typedef struct control_actions__action__InitialStanding_Goal
{
  double hip_height;
  double duration;
} control_actions__action__InitialStanding_Goal;

// Struct for a sequence of control_actions__action__InitialStanding_Goal.
typedef struct control_actions__action__InitialStanding_Goal__Sequence
{
  control_actions__action__InitialStanding_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_actions__action__InitialStanding_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/InitialStanding in the package control_actions.
typedef struct control_actions__action__InitialStanding_Result
{
  double hip_height;
} control_actions__action__InitialStanding_Result;

// Struct for a sequence of control_actions__action__InitialStanding_Result.
typedef struct control_actions__action__InitialStanding_Result__Sequence
{
  control_actions__action__InitialStanding_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_actions__action__InitialStanding_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/InitialStanding in the package control_actions.
typedef struct control_actions__action__InitialStanding_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} control_actions__action__InitialStanding_Feedback;

// Struct for a sequence of control_actions__action__InitialStanding_Feedback.
typedef struct control_actions__action__InitialStanding_Feedback__Sequence
{
  control_actions__action__InitialStanding_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_actions__action__InitialStanding_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "control_actions/action/detail/initial_standing__struct.h"

/// Struct defined in action/InitialStanding in the package control_actions.
typedef struct control_actions__action__InitialStanding_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  control_actions__action__InitialStanding_Goal goal;
} control_actions__action__InitialStanding_SendGoal_Request;

// Struct for a sequence of control_actions__action__InitialStanding_SendGoal_Request.
typedef struct control_actions__action__InitialStanding_SendGoal_Request__Sequence
{
  control_actions__action__InitialStanding_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_actions__action__InitialStanding_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/InitialStanding in the package control_actions.
typedef struct control_actions__action__InitialStanding_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} control_actions__action__InitialStanding_SendGoal_Response;

// Struct for a sequence of control_actions__action__InitialStanding_SendGoal_Response.
typedef struct control_actions__action__InitialStanding_SendGoal_Response__Sequence
{
  control_actions__action__InitialStanding_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_actions__action__InitialStanding_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/InitialStanding in the package control_actions.
typedef struct control_actions__action__InitialStanding_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} control_actions__action__InitialStanding_GetResult_Request;

// Struct for a sequence of control_actions__action__InitialStanding_GetResult_Request.
typedef struct control_actions__action__InitialStanding_GetResult_Request__Sequence
{
  control_actions__action__InitialStanding_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_actions__action__InitialStanding_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "control_actions/action/detail/initial_standing__struct.h"

/// Struct defined in action/InitialStanding in the package control_actions.
typedef struct control_actions__action__InitialStanding_GetResult_Response
{
  int8_t status;
  control_actions__action__InitialStanding_Result result;
} control_actions__action__InitialStanding_GetResult_Response;

// Struct for a sequence of control_actions__action__InitialStanding_GetResult_Response.
typedef struct control_actions__action__InitialStanding_GetResult_Response__Sequence
{
  control_actions__action__InitialStanding_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_actions__action__InitialStanding_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "control_actions/action/detail/initial_standing__struct.h"

/// Struct defined in action/InitialStanding in the package control_actions.
typedef struct control_actions__action__InitialStanding_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  control_actions__action__InitialStanding_Feedback feedback;
} control_actions__action__InitialStanding_FeedbackMessage;

// Struct for a sequence of control_actions__action__InitialStanding_FeedbackMessage.
typedef struct control_actions__action__InitialStanding_FeedbackMessage__Sequence
{
  control_actions__action__InitialStanding_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_actions__action__InitialStanding_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_ACTIONS__ACTION__DETAIL__INITIAL_STANDING__STRUCT_H_
