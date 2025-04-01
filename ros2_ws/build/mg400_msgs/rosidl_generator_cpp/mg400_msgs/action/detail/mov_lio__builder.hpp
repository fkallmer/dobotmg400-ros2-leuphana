// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mg400_msgs:action/MovLIO.idl
// generated code does not contain a copyright notice

#ifndef MG400_MSGS__ACTION__DETAIL__MOV_LIO__BUILDER_HPP_
#define MG400_MSGS__ACTION__DETAIL__MOV_LIO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mg400_msgs/action/detail/mov_lio__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mg400_msgs
{

namespace action
{

namespace builder
{

class Init_MovLIO_Goal_cp
{
public:
  explicit Init_MovLIO_Goal_cp(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  ::mg400_msgs::action::MovLIO_Goal cp(::mg400_msgs::action::MovLIO_Goal::_cp_type arg)
  {
    msg_.cp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_set_cp
{
public:
  explicit Init_MovLIO_Goal_set_cp(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_cp set_cp(::mg400_msgs::action::MovLIO_Goal::_set_cp_type arg)
  {
    msg_.set_cp = std::move(arg);
    return Init_MovLIO_Goal_cp(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_acc_l
{
public:
  explicit Init_MovLIO_Goal_acc_l(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_set_cp acc_l(::mg400_msgs::action::MovLIO_Goal::_acc_l_type arg)
  {
    msg_.acc_l = std::move(arg);
    return Init_MovLIO_Goal_set_cp(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_set_acc_l
{
public:
  explicit Init_MovLIO_Goal_set_acc_l(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_acc_l set_acc_l(::mg400_msgs::action::MovLIO_Goal::_set_acc_l_type arg)
  {
    msg_.set_acc_l = std::move(arg);
    return Init_MovLIO_Goal_acc_l(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_speed_l
{
public:
  explicit Init_MovLIO_Goal_speed_l(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_set_acc_l speed_l(::mg400_msgs::action::MovLIO_Goal::_speed_l_type arg)
  {
    msg_.speed_l = std::move(arg);
    return Init_MovLIO_Goal_set_acc_l(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_set_speed_l
{
public:
  explicit Init_MovLIO_Goal_set_speed_l(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_speed_l set_speed_l(::mg400_msgs::action::MovLIO_Goal::_set_speed_l_type arg)
  {
    msg_.set_speed_l = std::move(arg);
    return Init_MovLIO_Goal_speed_l(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_status
{
public:
  explicit Init_MovLIO_Goal_status(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_set_speed_l status(::mg400_msgs::action::MovLIO_Goal::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MovLIO_Goal_set_speed_l(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_index
{
public:
  explicit Init_MovLIO_Goal_index(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_status index(::mg400_msgs::action::MovLIO_Goal::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_MovLIO_Goal_status(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_distance
{
public:
  explicit Init_MovLIO_Goal_distance(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_index distance(::mg400_msgs::action::MovLIO_Goal::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_MovLIO_Goal_index(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_mode
{
public:
  explicit Init_MovLIO_Goal_mode(::mg400_msgs::action::MovLIO_Goal & msg)
  : msg_(msg)
  {}
  Init_MovLIO_Goal_distance mode(::mg400_msgs::action::MovLIO_Goal::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MovLIO_Goal_distance(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

class Init_MovLIO_Goal_pose
{
public:
  Init_MovLIO_Goal_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovLIO_Goal_mode pose(::mg400_msgs::action::MovLIO_Goal::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_MovLIO_Goal_mode(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mg400_msgs::action::MovLIO_Goal>()
{
  return mg400_msgs::action::builder::Init_MovLIO_Goal_pose();
}

}  // namespace mg400_msgs


namespace mg400_msgs
{

namespace action
{

namespace builder
{

class Init_MovLIO_Result_error_id
{
public:
  explicit Init_MovLIO_Result_error_id(::mg400_msgs::action::MovLIO_Result & msg)
  : msg_(msg)
  {}
  ::mg400_msgs::action::MovLIO_Result error_id(::mg400_msgs::action::MovLIO_Result::_error_id_type arg)
  {
    msg_.error_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Result msg_;
};

class Init_MovLIO_Result_result
{
public:
  Init_MovLIO_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovLIO_Result_error_id result(::mg400_msgs::action::MovLIO_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_MovLIO_Result_error_id(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mg400_msgs::action::MovLIO_Result>()
{
  return mg400_msgs::action::builder::Init_MovLIO_Result_result();
}

}  // namespace mg400_msgs


namespace mg400_msgs
{

namespace action
{

namespace builder
{

class Init_MovLIO_Feedback_current_pose
{
public:
  Init_MovLIO_Feedback_current_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mg400_msgs::action::MovLIO_Feedback current_pose(::mg400_msgs::action::MovLIO_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mg400_msgs::action::MovLIO_Feedback>()
{
  return mg400_msgs::action::builder::Init_MovLIO_Feedback_current_pose();
}

}  // namespace mg400_msgs


namespace mg400_msgs
{

namespace action
{

namespace builder
{

class Init_MovLIO_SendGoal_Request_goal
{
public:
  explicit Init_MovLIO_SendGoal_Request_goal(::mg400_msgs::action::MovLIO_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::mg400_msgs::action::MovLIO_SendGoal_Request goal(::mg400_msgs::action::MovLIO_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_SendGoal_Request msg_;
};

class Init_MovLIO_SendGoal_Request_goal_id
{
public:
  Init_MovLIO_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovLIO_SendGoal_Request_goal goal_id(::mg400_msgs::action::MovLIO_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MovLIO_SendGoal_Request_goal(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mg400_msgs::action::MovLIO_SendGoal_Request>()
{
  return mg400_msgs::action::builder::Init_MovLIO_SendGoal_Request_goal_id();
}

}  // namespace mg400_msgs


namespace mg400_msgs
{

namespace action
{

namespace builder
{

class Init_MovLIO_SendGoal_Response_stamp
{
public:
  explicit Init_MovLIO_SendGoal_Response_stamp(::mg400_msgs::action::MovLIO_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::mg400_msgs::action::MovLIO_SendGoal_Response stamp(::mg400_msgs::action::MovLIO_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_SendGoal_Response msg_;
};

class Init_MovLIO_SendGoal_Response_accepted
{
public:
  Init_MovLIO_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovLIO_SendGoal_Response_stamp accepted(::mg400_msgs::action::MovLIO_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MovLIO_SendGoal_Response_stamp(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mg400_msgs::action::MovLIO_SendGoal_Response>()
{
  return mg400_msgs::action::builder::Init_MovLIO_SendGoal_Response_accepted();
}

}  // namespace mg400_msgs


namespace mg400_msgs
{

namespace action
{

namespace builder
{

class Init_MovLIO_GetResult_Request_goal_id
{
public:
  Init_MovLIO_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mg400_msgs::action::MovLIO_GetResult_Request goal_id(::mg400_msgs::action::MovLIO_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mg400_msgs::action::MovLIO_GetResult_Request>()
{
  return mg400_msgs::action::builder::Init_MovLIO_GetResult_Request_goal_id();
}

}  // namespace mg400_msgs


namespace mg400_msgs
{

namespace action
{

namespace builder
{

class Init_MovLIO_GetResult_Response_result
{
public:
  explicit Init_MovLIO_GetResult_Response_result(::mg400_msgs::action::MovLIO_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::mg400_msgs::action::MovLIO_GetResult_Response result(::mg400_msgs::action::MovLIO_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_GetResult_Response msg_;
};

class Init_MovLIO_GetResult_Response_status
{
public:
  Init_MovLIO_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovLIO_GetResult_Response_result status(::mg400_msgs::action::MovLIO_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MovLIO_GetResult_Response_result(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mg400_msgs::action::MovLIO_GetResult_Response>()
{
  return mg400_msgs::action::builder::Init_MovLIO_GetResult_Response_status();
}

}  // namespace mg400_msgs


namespace mg400_msgs
{

namespace action
{

namespace builder
{

class Init_MovLIO_FeedbackMessage_feedback
{
public:
  explicit Init_MovLIO_FeedbackMessage_feedback(::mg400_msgs::action::MovLIO_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::mg400_msgs::action::MovLIO_FeedbackMessage feedback(::mg400_msgs::action::MovLIO_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_FeedbackMessage msg_;
};

class Init_MovLIO_FeedbackMessage_goal_id
{
public:
  Init_MovLIO_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovLIO_FeedbackMessage_feedback goal_id(::mg400_msgs::action::MovLIO_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MovLIO_FeedbackMessage_feedback(msg_);
  }

private:
  ::mg400_msgs::action::MovLIO_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mg400_msgs::action::MovLIO_FeedbackMessage>()
{
  return mg400_msgs::action::builder::Init_MovLIO_FeedbackMessage_goal_id();
}

}  // namespace mg400_msgs

#endif  // MG400_MSGS__ACTION__DETAIL__MOV_LIO__BUILDER_HPP_
