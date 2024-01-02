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
#ifndef ACTIONLIB__SERVER__ACTION_SERVER_IMP_H_
#define ACTIONLIB__SERVER__ACTION_SERVER_IMP_H_

#include <functional>
#include <list>
#include <string>

#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosserial_msgs/Log.h>

namespace actionlib
{
template<class ActionSpec>
ActionServer<ActionSpec>::ActionServer(const std::string& name,
  std::function<void(GoalHandle)> goal_cb,
  std::function<void(GoalHandle)> cancel_cb)
: ActionServerBase<ActionSpec>(name, goal_cb, cancel_cb),
  name(name),
  resultTopic(name + "/result"), feedbackTopic(name + "/feedback"), statusTopic(name + "/status"),
  goalTopic(name + "/goal"), cancelTopic(name + "/cancel"),
  result_pub_(resultTopic.c_str(), &resultMsg),
  feedback_pub_(feedbackTopic.c_str(), &feedbackMsg),
  status_pub_(statusTopic.c_str(), &statusMsg),
  goal_sub_(goalTopic.c_str(), &ActionServerBase<ActionSpec>::goalCallback, this),
  cancel_sub_(cancelTopic.c_str(), &ActionServerBase<ActionSpec>::cancelCallback, this)
{
}

template<class ActionSpec>
ActionServer<ActionSpec>::~ActionServer()
{
}

template<class ActionSpec>
void ActionServer<ActionSpec>::setup(ros::NodeHandleBase_& nh)
{
  ActionServerBase<ActionSpec>::setup(nh);

  this->nh_->advertise(this->result_pub_);
  this->nh_->advertise(this->feedback_pub_);
  this->nh_->advertise(this->status_pub_);

  this->nh_->subscribe(this->goal_sub_);
  this->nh_->subscribe(this->cancel_sub_);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::init(ros::NodeHandleBase_& nh)
{
  ActionServerBase<ActionSpec>::init(nh);

  // read the frequency with which to publish status from the parameter server
  // if not specified locally explicitly, use search param to find actionlib_status_frequency
  double status_frequency {5.0};
  if (!this->nh_->getParam((name + "/status_frequency").c_str(), status_frequency))
    this->nh_->getParam("actionlib_status_frequency", status_frequency);
  this->status_period_.fromSec(status_frequency);

  double status_list_timeout {5.0};
  this->nh_->getParam((name + "/status_list_timeout").c_str(), status_list_timeout);
  this->status_list_timeout_.fromSec(status_list_timeout);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::loop()
{
  if (!this->started_)
    return;

  if (last_status_publish_ + status_period_ >= ros::Time::now())
    return;

  last_status_publish_ = ros::Time::now();
  this->publishStatus();
}

template<class ActionSpec>
void ActionServer<ActionSpec>::publishResult(const actionlib_msgs::GoalStatus & status,
  const Result & result)
{
  // we'll create a shared_ptr to pass to ROS to limit copying
  resultMsg.header.stamp = ros::Time::now();
  resultMsg.status = status;
  resultMsg.result = result;
   this->nh_->log(rosserial_msgs::Log::ROSDEBUG, "Publishing result for goal with id: %s and stamp: %.2f",
     status.goal_id.id, status.goal_id.stamp.toSec());
  result_pub_.publish(&resultMsg);
  publishStatus();
}

template<class ActionSpec>
void ActionServer<ActionSpec>::publishFeedback(const actionlib_msgs::GoalStatus & status,
  const Feedback & feedback)
{
  // we'll create a shared_ptr to pass to ROS to limit copying
  feedbackMsg.header.stamp = ros::Time::now();
  feedbackMsg.status = status;
  feedbackMsg.feedback = feedback;
  this->nh_->log(rosserial_msgs::Log::ROSDEBUG, "Publishing feedback for goal with id: %s and stamp: %.2f",
     status.goal_id.id, status.goal_id.stamp.toSec());
  feedback_pub_.publish(&feedbackMsg);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::publishStatus()
{
  // build a status array
  statusMsg.header.stamp = ros::Time::now();

  static std::vector<actionlib_msgs::GoalStatus> status_list;
  status_list.resize(this->status_list_.size());

  unsigned int i = 0;
  for (typename std::list<StatusTracker<ActionSpec> >::iterator it = this->status_list_.begin();
    it != this->status_list_.end(); )
  {
    status_list[i] = it->getGoalStatus();

    // check if the item is due for deletion from the status list
    if ((*it).handle_destruction_time_ != ros::Time() &&
      (*it).handle_destruction_time_ + this->status_list_timeout_ < ros::Time::now())
    {
      it = this->status_list_.erase(it);
    } else {
      ++it;
    }
    ++i;
  }

  statusMsg.status_list_length = this->status_list_.size();
  statusMsg.status_list = status_list.data();

  status_pub_.publish(&statusMsg);
}

}  // namespace actionlib
#endif  // ACTIONLIB__SERVER__ACTION_SERVER_IMP_H_
