/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef ACTIONLIB__SERVER__STATUS_TRACKER_IMP_H_
#define ACTIONLIB__SERVER__STATUS_TRACKER_IMP_H_

#include <ros/time.h>

namespace actionlib
{

static uint8_t GOAL_COPY_BUFFER[MAX_GOAL_SIZE];

template<class ActionGoal>
void serializeToBuffer(std::vector<uint8_t>& buffer, const ActionGoal& goal)
{
  const auto size = goal.serialize(GOAL_COPY_BUFFER);
  buffer.resize(size);
  memcpy(buffer.data(), GOAL_COPY_BUFFER, size);
}

template<class ActionGoal>
GoalCopy<ActionGoal>::GoalCopy(const ActionGoal& goal)
{
  serializeToBuffer(buffer_, goal);
  goal_ = std::make_shared<ActionGoal>();
  goal_->deserialize(buffer_.data());
}

template<class ActionGoal>
GoalCopy<ActionGoal>::GoalCopy(const GoalCopy& other)
{
  buffer_ = other.buffer_;
  goal_ = std::make_shared<ActionGoal>();
  goal_->deserialize(buffer_.data());
}

template<class ActionGoal>
GoalCopy<ActionGoal>::GoalCopy(GoalCopy&& other) noexcept
{
  buffer_ = std::move(other.buffer_);
  goal_ = std::move(other.goal_);
}

template<class ActionGoal>
GoalCopy<ActionGoal>& GoalCopy<ActionGoal>::operator=(const GoalCopy& other)
{
  buffer_ = other.buffer_;
  goal_->deserialize(buffer_.data());
  return *this;
}

template<class ActionGoal>
GoalCopy<ActionGoal>& GoalCopy<ActionGoal>::operator=(GoalCopy&& other) noexcept
{
  buffer_ = std::move(other.buffer_);
  goal_ = std::move(other.goal_);
  return *this;
}

template<class ActionGoal>
GoalCopy<ActionGoal>::~GoalCopy() = default;

template<class ActionSpec>
StatusTracker<ActionSpec>::StatusTracker(const actionlib_msgs::GoalID & goal_id,
  unsigned int status) : id_generator_(goal_id.id)
{
  // set the goal id and status appropriately
  status_.goal_id = goal_id;
  status_.goal_id.id = strdup(status_.goal_id.id);
  free_goal_id_ = true;
  status_.status = status;
}

template<class ActionSpec>
StatusTracker<ActionSpec>::StatusTracker(const ActionGoal & goal)
: goal_copy_(goal), goal_(goal_copy_->goal_), id_generator_(goal_->goal_id.id)
{
  // set the goal_id from the message
  status_.goal_id = goal_->goal_id;

  // initialize the status of the goal to pending
  status_.status = actionlib_msgs::GoalStatus::PENDING;

  // if the goal id is zero, then we need to make up an id for the goal
  if (strlen(status_.goal_id.id) == 0) {
    status_.goal_id = id_generator_.generateID();
  }

  // if the timestamp of the goal is zero, then we'll set it to now()
  if (status_.goal_id.stamp == ros::Time()) {
    status_.goal_id.stamp = ros::Time::now();
  }
}

template<class ActionSpec>
StatusTracker<ActionSpec>::~StatusTracker()
{
  if (free_goal_id_)
    free(const_cast<char*>(status_.goal_id.id));
}

template<class ActionSpec>
const actionlib_msgs::GoalStatus& StatusTracker<ActionSpec>::getGoalStatus() const
{
  return status_;
}

template<class ActionSpec>
void StatusTracker<ActionSpec>::setStatus(actionlib_msgs::GoalStatus::_status_type status)
{
  status_.status = status;
}

template<class ActionSpec>
void StatusTracker<ActionSpec>::setText(const std::string& text)
{
  text_ = text;
  status_.text = text_.c_str();
}

}  // namespace actionlib
#endif  // ACTIONLIB__SERVER__STATUS_TRACKER_IMP_H_
