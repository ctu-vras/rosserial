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
#ifndef ACTIONLIB__SERVER__SIMPLE_ACTION_SERVER_H_
#define ACTIONLIB__SERVER__SIMPLE_ACTION_SERVER_H_

#include <actionlib/server/action_server.h>
#include <actionlib/action_definition.h>
#include <ros/node_handle.h>

#include <string>

namespace actionlib
{
/** @class SimpleActionServer @brief The SimpleActionServer
 * implements a single goal policy on top of the ActionServer class. The
 * specification of the policy is as follows: only one goal can have an
 * active status at a time, new goals preempt previous goals based on the
 * stamp in their GoalID field (later goals preempt earlier ones), an
 * explicit preempt goal preempts all goals with timestamps that are less
 * than or equal to the stamp associated with the preempt, accepting a new
 * goal implies successful preemption of any old goal and the status of the
 * old goal will be changed automatically to reflect this.
 */
template<class ActionSpec>
class SimpleActionServer
{
public:
  // generates typedefs that we'll use to make our lives easier
  ACTION_DEFINITION(ActionSpec)

  typedef typename ActionServer<ActionSpec>::GoalHandle GoalHandle;
  typedef std::function<void (const GoalConstPtr &)> ExecuteCallback;

  /**
   * @brief  Constructor for a SimpleActionServer
   * @param name A name for the action server
   * @param execute_callback Callback that gets called periodically to check the goal progress.
   */
  SimpleActionServer(const std::string& name, ExecuteCallback execute_callback);

  ~SimpleActionServer();

  /**
   * @brief  Accepts a new goal when one is available. The status of this
   * goal is set to active upon acceptance, and the status of any
   * previously active goal is set to preempted. Preempts received for the
   * new goal between checking if isNewGoalAvailable or invocation of a
   * goal callback and the acceptNewGoal call will not trigger a preempt
   * callback.  This means, isPreemptRequested should be called after
   * accepting the goal even for callback-based implementations to make
   * sure the new goal does not have a pending preempt request.
   * @return A shared_ptr to the new goal.
   */
  std::shared_ptr<const Goal> acceptNewGoal();

  /**
   * @brief  Allows polling implementations to query about the availability of a new goal
   * @return True if a new goal is available, false otherwise
   */
  bool isNewGoalAvailable();


  /**
   * @brief  Allows polling implementations to query about preempt requests
   * @return True if a preempt is requested, false otherwise
   */
  bool isPreemptRequested();

  /**
   * @brief  Allows polling implementations to query about the status of the current goal
   * @return True if a goal is active, false otherwise
   */
  bool isActive();

  /**
   * @brief  Sets the status of the active goal to succeeded
   * @param  result An optional result to send back to any clients of the goal
   * @param  result An optional text message to send back to any clients of the goal
   */
  void setSucceeded(const Result & result = Result(), const std::string & text = std::string(""));

  /**
   * @brief  Sets the status of the active goal to aborted
   * @param  result An optional result to send back to any clients of the goal
   * @param  result An optional text message to send back to any clients of the goal
   */
  void setAborted(const Result & result = Result(), const std::string & text = std::string(""));


  /**
  * @brief  Publishes feedback for a given goal
  * @param  feedback Shared pointer to the feedback to publish
  */
  void publishFeedback(const FeedbackConstPtr & feedback);

  /**
  * @brief  Publishes feedback for a given goal
  * @param  feedback The feedback to publish
  */
  void publishFeedback(const Feedback & feedback);

  /**
   * @brief  Sets the status of the active goal to preempted
   * @param  result An optional result to send back to any clients of the goal
   * @param  result An optional text message to send back to any clients of the goal
   */
  void setPreempted(const Result & result = Result(), const std::string & text = std::string(""));

  /**
   * @brief  Allows users to register a callback to be invoked when a new goal is available
   * @param cb The callback to be invoked
   */
  void registerGoalCallback(std::function<void()> cb);

  /**
   * @brief  Allows users to register a callback to be invoked when a new preempt request is available
   * @param cb The callback to be invoked
   */
  void registerPreemptCallback(std::function<void()> cb);

  /**
   * @brief  Explicitly start the action server, used it auto_start is set to false
   */
  void setup(ros::NodeHandleBase_& nh);
  void init(ros::NodeHandleBase_& nh);

  /**
   * @brief  Explicitly shutdown the action server
   */
  void shutdown();

 /**
  * @brief  Called from a separate thread to call blocking execute calls
  */
 void loop();

private:
  /**
   * @brief  Callback for when the ActionServer receives a new goal and passes it on
   */
  void goalCallback(GoalHandle goal);

  /**
   * @brief  Callback for when the ActionServer receives a new preempt and passes it on
   */
  void preemptCallback(GoalHandle preempt);

  ros::NodeHandleBase_* nh_;

  std::shared_ptr<ActionServer<ActionSpec> > as_;

  GoalHandle current_goal_, next_goal_;

  bool new_goal_, preempt_request_, new_goal_preempt_request_;

  std::function<void()> goal_callback_;
  std::function<void()> preempt_callback_;
  ExecuteCallback execute_callback_;

  bool need_to_terminate_;
};

}  // namespace actionlib

// include the implementation here
#include <actionlib/server/simple_action_server_imp.h>
#endif  // ACTIONLIB__SERVER__SIMPLE_ACTION_SERVER_H_
