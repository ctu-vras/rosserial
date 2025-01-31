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
#ifndef ACTIONLIB__SERVER__SERVER_GOAL_HANDLE_IMP_H_
#define ACTIONLIB__SERVER__SERVER_GOAL_HANDLE_IMP_H_

#include <list>
#include <string>

#include "ros/node_handle.h"

namespace actionlib
{
template<class ActionSpec>
ServerGoalHandle<ActionSpec>::ServerGoalHandle()
: as_(nullptr) {}

template<class ActionSpec>
ServerGoalHandle<ActionSpec>::ServerGoalHandle(const ServerGoalHandle & gh)
: status_it_(gh.status_it_), goal_(gh.goal_), as_(gh.as_), nh_(gh.nh_), handle_tracker_(gh.handle_tracker_),
  guard_(gh.guard_) {}

template<class ActionSpec>
void ServerGoalHandle<ActionSpec>::setAccepted(const std::string & text)
{
  if (as_ == nullptr) {
    // ROS_ERROR_NAMED("actionlib",
    //   "You are attempting to call methods on an uninitialized goal handle");
    return;
  }

  // check to see if we can use the action server
  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected()) {
    nh_->logerror(
    "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
    return;
  }

  nh_->log(rosserial_msgs::Log::ROSDEBUG, "Accepting goal, id: %s, stamp: %.2f",
    getGoalID().id, getGoalID().stamp.toSec());
  if (goal_) {
    unsigned int status = status_it_->getGoalStatus().status;

    // if we were pending before, then we'll go active
    if (status == actionlib_msgs::GoalStatus::PENDING) {
      status_it_->setStatus(actionlib_msgs::GoalStatus::ACTIVE);
      status_it_->setText(text);
      as_->publishStatus();
    } else if (status == actionlib_msgs::GoalStatus::RECALLING) {
      // if we were recalling before, now we'll go to preempting
      status_it_->setStatus(actionlib_msgs::GoalStatus::PREEMPTING);
      status_it_->setText(text);
      as_->publishStatus();
    } else {
      nh_->log(rosserial_msgs::Log::ERROR,
        "To transition to an active state, the goal must be in a pending or recalling state, it is currently in state: %d",
        status_it_->getGoalStatus().status);
    }
  } else {
    nh_->logerror("Attempt to set status on an uninitialized ServerGoalHandle");
  }
}

template<class ActionSpec>
void ServerGoalHandle<ActionSpec>::setCanceled(const Result & result, const std::string & text)
{
  if (as_ == nullptr) {
    // ROS_ERROR_NAMED("actionlib",
    //   "You are attempting to call methods on an uninitialized goal handle");
    return;
  }

  // check to see if we can use the action server
  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected()) {
    nh_->logerror(
      "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
    return;
  }

  nh_->log(rosserial_msgs::Log::ROSDEBUG, "Setting status to canceled on goal, id: %s, stamp: %.2f",
    getGoalID().id, getGoalID().stamp.toSec());
  if (goal_) {
    unsigned int status = status_it_->getGoalStatus().status;
    if (status == actionlib_msgs::GoalStatus::PENDING ||
      status == actionlib_msgs::GoalStatus::RECALLING)
    {
      status_it_->setStatus(actionlib_msgs::GoalStatus::RECALLED);
      status_it_->setText(text);
      as_->publishResult(status_it_->getGoalStatus(), result);
    } else if (status == actionlib_msgs::GoalStatus::ACTIVE ||
      status == actionlib_msgs::GoalStatus::PREEMPTING) {
      status_it_->setStatus(actionlib_msgs::GoalStatus::PREEMPTED);
      status_it_->setText(text);
      as_->publishResult(status_it_->getGoalStatus(), result);
    } else {
      nh_->log(rosserial_msgs::Log::ERROR,
        "To transition to a cancelled state, the goal must be in a pending, recalling, active, or preempting state, it is currently in state: %d",
        status_it_->getGoalStatus().status);
    }
  } else {
    nh_->logerror("Attempt to set status on an uninitialized ServerGoalHandle");
  }
}

template<class ActionSpec>
void ServerGoalHandle<ActionSpec>::setRejected(const Result & result, const std::string & text)
{
  if (as_ == nullptr) {
    // ROS_ERROR_NAMED("actionlib",
    //   "You are attempting to call methods on an uninitialized goal handle");
    return;
  }

  // check to see if we can use the action server
  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected()) {
    nh_->logerror(
      "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
    return;
  }

  nh_->log(rosserial_msgs::Log::ROSDEBUG, "Setting status to rejected on goal, id: %s, stamp: %.2f",
    getGoalID().id, getGoalID().stamp.toSec());
  if (goal_) {
    unsigned int status = status_it_->getGoalStatus().status;
    if (status == actionlib_msgs::GoalStatus::PENDING ||
      status == actionlib_msgs::GoalStatus::RECALLING)
    {
      status_it_->setStatus(actionlib_msgs::GoalStatus::REJECTED);
      status_it_->setText(text);
      as_->publishResult(status_it_->getGoalStatus(), result);
    } else {
      nh_->log(rosserial_msgs::Log::ERROR,
        "To transition to a rejected state, the goal must be in a pending or recalling state, it is currently in state: %d",
        status_it_->getGoalStatus().status);
    }
  } else {
    nh_->logerror("Attempt to set status on an uninitialized ServerGoalHandle");
  }
}

template<class ActionSpec>
void ServerGoalHandle<ActionSpec>::setAborted(const Result & result, const std::string & text)
{
  if (as_ == nullptr) {
    // ROS_ERROR_NAMED("actionlib",
    //   "You are attempting to call methods on an uninitialized goal handle");
    return;
  }

  // check to see if we can use the action server
  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected()) {
    nh_->logerror(
      "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
    return;
  }

  nh_->log(rosserial_msgs::Log::ROSDEBUG, "Setting status to aborted on goal, id: %s, stamp: %.2f",
    getGoalID().id, getGoalID().stamp.toSec());
  if (goal_) {
    unsigned int status = status_it_->getGoalStatus().status;
    if (status == actionlib_msgs::GoalStatus::PREEMPTING ||
      status == actionlib_msgs::GoalStatus::ACTIVE)
    {
      status_it_->setStatus(actionlib_msgs::GoalStatus::ABORTED);
      status_it_->setText(text);
      as_->publishResult(status_it_->getGoalStatus(), result);
    } else {
      nh_->log(rosserial_msgs::Log::ERROR,
        "To transition to an aborted state, the goal must be in a preempting or active state, it is currently in state: %d",
        status);
    }
  } else {
    nh_->logerror("Attempt to set status on an uninitialized ServerGoalHandle");
  }
}

template<class ActionSpec>
void ServerGoalHandle<ActionSpec>::setSucceeded(const Result & result, const std::string & text)
{
  if (as_ == nullptr) {
    // ROS_ERROR_NAMED("actionlib",
    //   "You are attempting to call methods on an uninitialized goal handle");
    return;
  }

  // check to see if we can use the action server
  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected()) {
    nh_->logerror(
      "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
    return;
  }

  nh_->log(rosserial_msgs::Log::ROSDEBUG, "Setting status to succeeded on goal, id: %s, stamp: %.2f",
    getGoalID().id, getGoalID().stamp.toSec());
  if (goal_) {
    unsigned int status = status_it_->getGoalStatus().status;
    if (status == actionlib_msgs::GoalStatus::PREEMPTING ||
      status == actionlib_msgs::GoalStatus::ACTIVE)
    {
      status_it_->setStatus(actionlib_msgs::GoalStatus::SUCCEEDED);
      status_it_->setText(text);
      as_->publishResult(status_it_->getGoalStatus(), result);
    } else {
      nh_->log(rosserial_msgs::Log::ERROR,
        "To transition to a succeeded state, the goal must be in a preempting or active state, it is currently in state: %d",
        status);
    }
  } else {
    nh_->logerror("Attempt to set status on an uninitialized ServerGoalHandle");
  }
}

template<class ActionSpec>
void ServerGoalHandle<ActionSpec>::publishFeedback(const Feedback & feedback)
{
  if (as_ == nullptr) {
    // ROS_ERROR_NAMED("actionlib",
    //   "You are attempting to call methods on an uninitialized goal handle");
    return;
  }

  // check to see if we can use the action server
  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected()) {
    nh_->logerror(
      "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
    return;
  }

  nh_->log(rosserial_msgs::Log::ROSDEBUG, "Publishing feedback for goal, id: %s, stamp: %.2f",
    getGoalID().id, getGoalID().stamp.toSec());
  if (goal_) {
    as_->publishFeedback(status_it_->getGoalStatus(), feedback);
  } else {
    nh_->logerror(
      "Attempt to publish feedback on an uninitialized ServerGoalHandle");
  }
}

template<class ActionSpec>
bool ServerGoalHandle<ActionSpec>::isValid() const
{
  return goal_ && as_ != nullptr;
}

template<class ActionSpec>
std::shared_ptr<const typename ServerGoalHandle<ActionSpec>::Goal> ServerGoalHandle<ActionSpec>::
getGoal() const
{
  // if we have a goal that is non-null
  if (goal_) {
    // create the deleter for our goal subtype
    EnclosureDeleter<const ActionGoal> d(goal_);
    return std::shared_ptr<const Goal>(&(goal_->goal), d);
  }
  return std::shared_ptr<const Goal>();
}

template<class ActionSpec>
actionlib_msgs::GoalID ServerGoalHandle<ActionSpec>::getGoalID() const
{
  if (goal_ && as_ != nullptr) {
    DestructionGuard::ScopedProtector protector(*guard_);
    if (protector.isProtected()) {
      return status_it_->getGoalStatus().goal_id;
    } else {
      return actionlib_msgs::GoalID();
    }
  } else {
    if (nh_)
      nh_->logerror(
        "Attempt to get a goal id on an uninitialized ServerGoalHandle or one that has no ActionServer associated with it.");
    return actionlib_msgs::GoalID();
  }
}

template<class ActionSpec>
actionlib_msgs::GoalStatus ServerGoalHandle<ActionSpec>::getGoalStatus() const
{
  if (goal_ && as_ != nullptr) {
    DestructionGuard::ScopedProtector protector(*guard_);
    if (protector.isProtected()) {
      return status_it_->getGoalStatus();
    } else {
      return actionlib_msgs::GoalStatus();
    }
  } else {
    if (nh_)
      nh_->logerror(
        "Attempt to get goal status on an uninitialized ServerGoalHandle or one that has no ActionServer associated with it.");
    return actionlib_msgs::GoalStatus();
  }
}

template<class ActionSpec>
ServerGoalHandle<ActionSpec> & ServerGoalHandle<ActionSpec>::operator=(const ServerGoalHandle & gh)
{
  status_it_ = gh.status_it_;
  goal_ = gh.goal_;
  as_ = gh.as_;
  nh_ = gh.nh_;
  handle_tracker_ = gh.handle_tracker_;
  guard_ = gh.guard_;
  return *this;
}

template<class ActionSpec>
bool ServerGoalHandle<ActionSpec>::operator==(const ServerGoalHandle & other) const
{
  if (!goal_ && !other.goal_) {
    return true;
  }

  if (!goal_ || !other.goal_) {
    return false;
  }

  actionlib_msgs::GoalID my_id = getGoalID();
  actionlib_msgs::GoalID their_id = other.getGoalID();
  return strcmp(my_id.id, their_id.id) == 0;
}

template<class ActionSpec>
bool ServerGoalHandle<ActionSpec>::operator!=(const ServerGoalHandle & other) const
{
  return !(*this == other);
}

template<class ActionSpec>
ServerGoalHandle<ActionSpec>::ServerGoalHandle(
  typename std::list<StatusTracker<ActionSpec> >::iterator status_it,
  ActionServerBase<ActionSpec> * as, std::shared_ptr<void> handle_tracker,
  std::shared_ptr<DestructionGuard> guard)
: status_it_(status_it), goal_((*status_it).goal_),
  as_(as), nh_(as->nh_), handle_tracker_(handle_tracker), guard_(guard) {}

template<class ActionSpec>
bool ServerGoalHandle<ActionSpec>::setCancelRequested()
{
  if (as_ == nullptr) {
    // ROS_ERROR_NAMED("actionlib",
    //   "You are attempting to call methods on an uninitialized goal handle");
    return false;
  }

  // check to see if we can use the action server
  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected()) {
    nh_->logerror(
      "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
    return false;
  }

  nh_->log(rosserial_msgs::Log::ROSDEBUG,
    "Transitioning to a cancel requested state on goal id: %s, stamp: %.2f",
    getGoalID().id, getGoalID().stamp.toSec());
  if (goal_) {
    unsigned int status = status_it_->getGoalStatus().status;
    if (status == actionlib_msgs::GoalStatus::PENDING) {
      status_it_->setStatus(actionlib_msgs::GoalStatus::RECALLING);
      as_->publishStatus();
      return true;
    }

    if (status == actionlib_msgs::GoalStatus::ACTIVE) {
      status_it_->setStatus(actionlib_msgs::GoalStatus::PREEMPTING);
      as_->publishStatus();
      return true;
    }
  }
  return false;
}
}  // namespace actionlib
#endif  // ACTIONLIB__SERVER__SERVER_GOAL_HANDLE_IMP_H_
