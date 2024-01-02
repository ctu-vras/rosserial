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
#ifndef ACTIONLIB__SERVER__SIMPLE_ACTION_SERVER_IMP_H_
#define ACTIONLIB__SERVER__SIMPLE_ACTION_SERVER_IMP_H_

#include <string>

#include <ros/node_handle.h>

namespace actionlib
{

template<class ActionSpec>
SimpleActionServer<ActionSpec>::SimpleActionServer(const std::string& name, ExecuteCallback execute_callback)
: new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false), execute_callback_(
    execute_callback), need_to_terminate_(false)
{
  // create the action server
  as_ = std::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(name,
      std::bind(&SimpleActionServer::goalCallback, this, std::placeholders::_1),
      std::bind(&SimpleActionServer::preemptCallback, this, std::placeholders::_1)));

}

template<class ActionSpec>
SimpleActionServer<ActionSpec>::~SimpleActionServer()
{
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::shutdown()
{
}

template<class ActionSpec>
std::shared_ptr<const typename SimpleActionServer<ActionSpec>::Goal> SimpleActionServer<ActionSpec>
::acceptNewGoal()
{
  if (!new_goal_ || !next_goal_.getGoal()) {
    nh_->logerror("Attempting to accept the next goal when a new goal is not available");
    return std::shared_ptr<const Goal>();
  }

  // check if we need to send a preempted message for the goal that we're currently pursuing
  if (isActive() &&
    current_goal_.getGoal() &&
    current_goal_ != next_goal_)
  {
    current_goal_.setCanceled(
      Result(),
      "This goal was canceled because another goal was recieved by the simple action server");
  }

  nh_->logdebug("Accepting a new goal");

  // accept the next goal
  current_goal_ = next_goal_;
  new_goal_ = false;

  // set preempt to request to equal the preempt state of the new goal
  preempt_request_ = new_goal_preempt_request_;
  new_goal_preempt_request_ = false;

  // set the status of the current goal to be active
  current_goal_.setAccepted("This goal has been accepted by the simple action server");

  return current_goal_.getGoal();
}

template<class ActionSpec>
bool SimpleActionServer<ActionSpec>::isNewGoalAvailable()
{
  return new_goal_;
}


template<class ActionSpec>
bool SimpleActionServer<ActionSpec>::isPreemptRequested()
{
  return preempt_request_;
}

template<class ActionSpec>
bool SimpleActionServer<ActionSpec>::isActive()
{
  if (!current_goal_.getGoal()) {
    return false;
  }
  unsigned int status = current_goal_.getGoalStatus().status;
  return status == actionlib_msgs::GoalStatus::ACTIVE ||
         status == actionlib_msgs::GoalStatus::PREEMPTING;
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::setSucceeded(const Result & result, const std::string & text)
{
  nh_->logdebug("Setting the current goal as succeeded");
  current_goal_.setSucceeded(result, text);
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::setAborted(const Result & result, const std::string & text)
{
  nh_->logdebug("Setting the current goal as aborted");
  current_goal_.setAborted(result, text);
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::setPreempted(const Result & result, const std::string & text)
{
  nh_->logdebug("Setting the current goal as canceled");
  current_goal_.setCanceled(result, text);
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::registerGoalCallback(std::function<void()> cb)
{
  goal_callback_ = cb;
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::registerPreemptCallback(std::function<void()> cb)
{
  preempt_callback_ = cb;
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::publishFeedback(const FeedbackConstPtr & feedback)
{
  current_goal_.publishFeedback(*feedback);
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::publishFeedback(const Feedback & feedback)
{
  current_goal_.publishFeedback(feedback);
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::goalCallback(GoalHandle goal)
{
  nh_->logdebug("A new goal has been recieved by the single goal action server");

  // check that the timestamp is past or equal to that of the current goal and the next goal
  if ((!current_goal_.getGoal() || goal.getGoalID().stamp >= current_goal_.getGoalID().stamp) &&
    (!next_goal_.getGoal() || goal.getGoalID().stamp >= next_goal_.getGoalID().stamp))
  {
    // if next_goal has not been accepted already... its going to get bumped, but we need to let the client know we're preempting
    if (next_goal_.getGoal() && (!current_goal_.getGoal() || next_goal_ != current_goal_)) {
      next_goal_.setCanceled(
        Result(),
        "This goal was canceled because another goal was recieved by the simple action server");
    }

    next_goal_ = goal;
    new_goal_ = true;
    new_goal_preempt_request_ = false;

    // if the server is active, we'll want to call the preempt callback for the current goal
    if (isActive()) {
      preempt_request_ = true;
      // if the user has registered a preempt callback, we'll call it now
      if (preempt_callback_) {
        preempt_callback_();
      }
    }

    // if the user has defined a goal callback, we'll call it now
    if (goal_callback_) {
      goal_callback_();
    }
  } else {
    // the goal requested has already been preempted by a different goal, so we're not going to execute it
    goal.setCanceled(
      Result(),
      "This goal was canceled because another goal was recieved by the simple action server");
  }
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::preemptCallback(GoalHandle preempt)
{
  nh_->logdebug("A preempt has been received by the SimpleActionServer");

  // if the preempt is for the current goal, then we'll set the preemptRequest flag and call the user's preempt callback
  if (preempt == current_goal_) {
    nh_->logdebug(
      "Setting preempt_request bit for the current goal to TRUE and invoking callback");
    preempt_request_ = true;

    // if the user has registered a preempt callback, we'll call it now
    if (preempt_callback_) {
      preempt_callback_();
    }
  } else if (preempt == next_goal_) {
    // if the preempt applies to the next goal, we'll set the preempt bit for that
    nh_->logdebug("Setting preempt request bit for the next goal to TRUE");
    new_goal_preempt_request_ = true;
  }
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::loop()
{
  if (need_to_terminate_) {
    return;
  }

  if (!isActive() && isNewGoalAvailable())
  {
    acceptNewGoal();
  }

  if (!isActive())
    return;

  execute_callback_(current_goal_.getGoal());

  as_->loop();
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::setup(ros::NodeHandleBase_& nh)
{
  nh_ = &nh;
  as_->setup(nh);
}

template<class ActionSpec>
void SimpleActionServer<ActionSpec>::init(ros::NodeHandleBase_& nh)
{
  as_->init(nh);
}

}  // namespace actionlib

#endif  // ACTIONLIB__SERVER__SIMPLE_ACTION_SERVER_IMP_H_
