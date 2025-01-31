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
#ifndef ACTIONLIB__SERVER__HANDLE_TRACKER_DELETER_IMP_H_
#define ACTIONLIB__SERVER__HANDLE_TRACKER_DELETER_IMP_H_

#include <list>

#include <ros/time.h>

namespace actionlib
{
template<class ActionSpec>
HandleTrackerDeleter<ActionSpec>::HandleTrackerDeleter(ActionServerBase<ActionSpec> * as,
  typename std::list<StatusTracker<ActionSpec> >::iterator status_it,
  std::shared_ptr<DestructionGuard> guard)
: as_(as), status_it_(status_it), guard_(guard) {}

template<class ActionSpec>
void HandleTrackerDeleter<ActionSpec>::operator()(void *)
{
  if (as_) {
    // make sure that the action server hasn't been destroyed yet
    DestructionGuard::ScopedProtector protector(*guard_);
    if (protector.isProtected()) {
      // make sure to lock while we erase status for this goal from the list
      (*status_it_).handle_destruction_time_ = ros::Time::now();
      // as_->status_list_.erase(status_it_);
    }
  }
}
}  // namespace actionlib
#endif  // ACTIONLIB__SERVER__HANDLE_TRACKER_DELETER_IMP_H_
