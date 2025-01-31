/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/time.h"
#include "ros/node_handle.h"

#include <tuple>

namespace ros
{

NodeHandleBase_* Time::nh;

void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
{
  uint32_t nsec_part = nsec % 1000000000UL;
  uint32_t sec_part = nsec / 1000000000UL;
  sec += sec_part;
  nsec = nsec_part;
}

Time& Time::fromNSec(int32_t t)
{
  sec = t / 1000000000;
  nsec = t % 1000000000;
  normalizeSecNSec(sec, nsec);
  return *this;
}

Time& Time::operator +=(const Duration &rhs)
{
  sec = sec - 1 + rhs.sec;
  nsec = nsec + 1000000000UL + rhs.nsec;
  normalizeSecNSec(sec, nsec);
  return *this;
}

Time Time::operator+(const Duration &rhs)
{
  ros::Time t(*this);
  t += rhs;
  return t;
}

Time& Time::operator -=(const Duration &rhs){
  sec = sec - 1 - rhs.sec;
  nsec = nsec + 1000000000UL - rhs.nsec;
  normalizeSecNSec(sec, nsec);
  return *this;
}

Duration Time::operator-(const Time &rhs) const {
  // Note: Considers wrap around as a continuation of time, e.g.,
  // (0,0) - (0xFFFFFFFF, 0) = (1, 0)
  Duration d;
  d.sec = sec > rhs.sec ? sec - rhs.sec : -(rhs.sec - sec);
  d.nsec = nsec > rhs.nsec ? nsec - rhs.nsec : -(rhs.nsec - nsec);
  normalizeSecNSecSigned(d.sec, d.nsec);
  return d;
}

Time Time::now()
{
  if (nh)
    return nh->now();
  return {};
}

bool Time::operator<(const Time& t) const {
  return std::tie(sec, nsec) < std::tie(t.sec, t.nsec);
}

bool Time::operator<=(const Time& t) const {
  return std::tie(sec, nsec) <= std::tie(t.sec, t.nsec);
}

bool Time::operator>=(const Time& t) const {
  return !(*this < t);
}

bool Time::operator>(const Time& t) const {
  return !(*this <= t);
}

bool Time::operator==(const Time& t) const {
  return this->sec == t.sec && this->nsec == t.nsec;
}

bool Time::operator!=(const Time& t) const {
  return !(*this == t);
}

}
