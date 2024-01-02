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
#ifndef ACTIONLIB__SERVER__ACTION_SERVER_H_
#define ACTIONLIB__SERVER__ACTION_SERVER_H_

#include <functional>
#include <memory>
#include <string>

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib/enclosure_deleter.h>
#include <actionlib/goal_id_generator.h>
#include <actionlib/action_definition.h>
#include <actionlib/server/status_tracker.h>
#include <actionlib/server/handle_tracker_deleter.h>
#include <actionlib/server/server_goal_handle.h>
#include <actionlib/server/action_server_base.h>
#include <actionlib/destruction_guard.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <list>
#include <string>

namespace actionlib
{
/**
 * @class ActionServer
 * @brief The ActionServer is a helpful tool for managing goal requests to a
 * node. It allows the user to specify callbacks that are invoked when goal
 * or cancel requests come over the wire, and passes back GoalHandles that
 * can be used to track the state of a given goal request. The ActionServer
 * makes no assumptions about the policy used to service these goals, and
 * sends status for each goal over the wire until the last GoalHandle
 * associated with a goal request is destroyed.
 */
template<class ActionSpec>
class ActionServer : public ActionServerBase<ActionSpec>
{
public:
  // for convenience when referring to ServerGoalHandles
  typedef ServerGoalHandle<ActionSpec> GoalHandle;

  // generates typedefs that we'll use to make our lives easier
  ACTION_DEFINITION(ActionSpec)

  /**
   * @brief  Constructor for an ActionServer
   * @param  n A NodeHandle to create a namespace under
   * @param  name The name of the action
   * @param  goal_cb A goal callback to be called when the ActionServer receives a new goal over the wire
   * @param  cancel_cb A cancel callback to be called when the ActionServer receives a new cancel request over the wire
   * @param  auto_start A boolean value that tells the ActionServer whether or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.
   */
  ActionServer(const std::string& name,
    std::function<void(GoalHandle)> goal_cb,
    std::function<void(GoalHandle)> cancel_cb);

  /**
   * @brief  Destructor for the ActionServer
   */
  ~ActionServer() override;


 /**
  * @brief  Initialize all ROS connections and setup timers
  */
 void setup(ros::NodeHandleBase_& nh) override;
 void init(ros::NodeHandleBase_& nh) override;

 void loop() override;

private:

  /**
   * @brief  Publishes a result for a given goal
   * @param status The status of the goal with which the result is associated
   * @param result The result to publish
   */
  virtual void publishResult(const actionlib_msgs::GoalStatus & status, const Result & result);

  /**
   * @brief  Publishes feedback for a given goal
   * @param status The status of the goal with which the feedback is associated
   * @param feedback The feedback to publish
   */
  virtual void publishFeedback(const actionlib_msgs::GoalStatus & status,
    const Feedback & feedback);

  /**
   * @brief  Explicitly publish status
   */
  virtual void publishStatus();

  std::string name;

  std::string resultTopic;
  std::string feedbackTopic;
  std::string statusTopic;
  std::string goalTopic;
  std::string cancelTopic;

  ActionResult resultMsg;
  ActionFeedback feedbackMsg;
  actionlib_msgs::GoalStatusArray statusMsg;



  ros::Subscriber<ActionGoal, ActionServer<ActionSpec>> goal_sub_;
  ros::Subscriber<actionlib_msgs::GoalID, ActionServer<ActionSpec>> cancel_sub_;
  ros::Publisher status_pub_, result_pub_, feedback_pub_;

  ros::Time last_status_publish_;
  ros::Duration status_period_;
};
}  // namespace actionlib

// include the implementation
#include <actionlib/server/action_server_imp.h>
#endif  // ACTIONLIB__SERVER__ACTION_SERVER_H_
