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

#ifndef ROS_NODE_HANDLE_H_
#define ROS_NODE_HANDLE_H_

#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <iterator>  // For std::size
#include <memory>
#include <sstream>

#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "rosserial_msgs/TopicInfo.h"
#include "rosserial_msgs/Log.h"
#include "rosserial_msgs/RequestParam.h"

#include "ros/msg.h"

namespace ros
{

class HardwareTemplate {
public:
  void init() {
    // Initialize the hardware connecting to a default port
  }

  void init(const char* port) {
    // Initialize the hardware connecting to the given port
  }

  void print_stats() {
    // Print some useful statistics about the hardware
  }

  void reset_rbuf(uint8_t* data, const uint16_t length) {
    // Save the incoming data from hardware into a buffer from which read() can read them a little later.
  }

  int read() {
    // Read data from the buffer if there are some.
    return -1;
  }

  void flush() {
    // Flush data written to the hardware (if the hardware needs flushing).
  }

  bool write(const uint8_t* data, const size_t length) {
    // Write the given data to the hardware. They can be transmitted right away or may be delayed. Flush might be needed.
    return false;
  }

  uint32_t time() const {
    // Get the current hardware time in ms.
    return 0;
  }

  bool connected() const {
    // Tell whether the hardware is connected to the other endpoint.
    return false;
  }

  bool checkConnection() const {
    // If the hardware is not connected, tries to connect and returns whether it is now connected.
    return false;
  }

  void disconnect() {
    // Disconnect the hardware and clear buffers.
  }

  void printf(const char* fmt, va_list args)
  {
    // Print the given string like vprintf() to log output.
  }
};

class Publisher;
class Subscriber_;
template<typename MReq , typename MRes, typename ObjT>
class ServiceServer;
template<typename MReq , typename MRes>
class ServiceClient;

class NodeHandleBase_
{
public:
  virtual ~NodeHandleBase_() = default;
  virtual int publish(int id, const Msg* msg) const = 0;
  virtual int spinOnce() = 0;
  virtual bool connected() const = 0;
  virtual Time now() const = 0;
  virtual void initNode() = 0;
  virtual void initNode(const char* portName) = 0;
  virtual bool checkHardwareConnection() = 0;
  virtual bool advertise(Publisher & p) = 0;
  virtual bool subscribe(Subscriber_& s) = 0;
  virtual void printf(const char* fmt, ...) const = 0;

  /* Register a new Service Server */
  template<typename MReq, typename MRes, typename ObjT>
  bool advertiseService(ServiceServer<MReq, MRes, ObjT>& srv)
  {
    bool v = advertise(srv.pub);
    bool w = subscribe(srv);
    return v && w;
  }

  /* Register a new Service Client */
  template<typename MReq, typename MRes>
  bool serviceClient(ServiceClient<MReq, MRes>& srv)
  {
    bool v = advertise(srv.pub);
    bool w = subscribe(srv);
    return v && w;
  }

  bool getParam(const char* name, int32_t* param, uint32_t length = 1, uint32_t timeout = 1000) const
  {
    if (requestParam(name, timeout))
    {
      if (length == req_param_resp.ints_length)
      {
        //copy it over
        for (uint32_t i = 0; i < length; i++)
          param[i] = req_param_resp.ints[i];
        return true;
      }
      else
      {
        if (req_param_resp.ints_length == 0)
          log(rosserial_msgs::Log::INFO, "Parameter '%s' not found.", name);
        else
          log(rosserial_msgs::Log::WARN, "Failed to get param '%s': length mismatch", name);
      }
    }
    return false;
  }

  bool getParam(const char* name, int& param, uint32_t timeout = 1000) const
  {
    int32_t p;
    const auto res = this->getParam(name, &p, 1, timeout);
    param = p;
    return res;
  }

  bool getParam(const char* name, int* param, uint32_t timeout = 1000) const
  {
    return this->getParam(name, *param, timeout);
  }

  bool getParam(const char* name, float* param, uint32_t length = 1, uint32_t timeout = 1000) const
  {
    if (requestParam(name, timeout))
    {
      if (length == req_param_resp.floats_length)
      {
        //copy it over
        for (uint32_t i = 0; i < length; i++)
          param[i] = req_param_resp.floats[i];
        return true;
      }
      else
      {
        if (req_param_resp.floats_length == 0)
          log(rosserial_msgs::Log::INFO, "Parameter '%s' not found.", name);
        else
          log(rosserial_msgs::Log::WARN, "Failed to get param '%s': length mismatch", name);
      }
    }
    return false;
  }

  bool getParam(const char* name, double* param, uint32_t length = 1, uint32_t timeout = 1000) const
  {
    float p;
    const auto res = getParam(name, &p, length, timeout);
    if (res)
      *param = p;
    return res;
  }

  bool getParam(const char* name, double& param, uint32_t timeout = 1000) const
  {
    return this->getParam(name, &param, 1, timeout);
  }

  bool getParam(const char* name, std::string& param, uint32_t timeout = 1000) const
  {
    if (this->requestParam(name, timeout))
    {
      if (1 == req_param_resp.strings_length)
      {
        param = req_param_resp.strings[0];
        return true;
      }
      else
      {
        if (req_param_resp.strings_length == 0)
          log(rosserial_msgs::Log::INFO, "Parameter '%s' not found.", name);
        else
          log(rosserial_msgs::Log::WARN, "Failed to get param '%s': length mismatch", name);
      }
    }
    return false;
  }

  bool getParam(const char* name, char** param, uint32_t length = 1, uint32_t timeout = 1000) const
  {
    if (requestParam(name, timeout))
    {
      if (length == req_param_resp.strings_length)
      {
        //copy it over
        for (uint32_t i = 0; i < length; i++)
          strcpy(param[i], req_param_resp.strings[i]);
        return true;
      }
      else
      {
        if (req_param_resp.strings_length == 0)
          log(rosserial_msgs::Log::INFO, "Parameter '%s' not found.", name);
        else
          log(rosserial_msgs::Log::WARN, "Failed to get param '%s': length mismatch", name);
      }
    }
    return false;
  }

  bool getParam(const char* name, bool& param, uint32_t timeout = 1000) const
  {
    return this->getParam(name, &param, 1, timeout);
  }
  bool getParam(const char* name, bool* param, uint32_t length = 1, uint32_t timeout = 1000) const
  {
    if (requestParam(name, timeout))
    {
      if (length == req_param_resp.ints_length)
      {
        //copy it over
        for (uint32_t i = 0; i < length; i++)
          param[i] = req_param_resp.ints[i];
        return true;
      }
      else
      {
        if (req_param_resp.ints_length == 0)
          log(rosserial_msgs::Log::INFO, "Parameter '%s' not found.", name);
        else
          log(rosserial_msgs::Log::WARN, "Failed to get param '%s': length mismatch", name);
      }
    }
    return false;
  }

  void setParam(const char* name, const bool& param) const
  {
    setParamXmlRpcValue(name, param ? "<boolean>1</boolean>" : "<boolean>0</boolean>");
  }

  void setParam(const char* name, const int& param) const
  {
    std::stringstream ss;
    ss << "<i4>" << param << "</i4>";
    setParamXmlRpcValue(name, ss.str());
  }

  void setParam(const char* name, const int32_t& param) const
  {
    std::stringstream ss;
    ss << "<i4>" << param << "</i4>";
    setParamXmlRpcValue(name, ss.str());
  }

  void setParam(const char* name, const float& param) const
  {
    std::stringstream ss;
    ss << "<double>" << param << "</double>";
    setParamXmlRpcValue(name, ss.str());
  }

  void setParam(const char* name, const double& param) const
  {
    std::stringstream ss;
    ss << "<double>" << param << "</double>";
    setParamXmlRpcValue(name, ss.str());
  }

  void setParam(const char* name, const char* param) const
  {
    std::stringstream ss;
    ss << "<string>" << param << "</string>";
    setParamXmlRpcValue(name, ss.str());
  }

  void setParam(const char* name, const std::string& param) const
  {
    std::stringstream ss;
    ss << "<string>" << param << "</string>";
    setParamXmlRpcValue(name, ss.str());
  }

  void log(rosserial_msgs::Log::_level_type level, const char * fmt, ...) const
  {
    static char msg[512];
    va_list argptr;
    va_start(argptr, fmt);
    vsnprintf(msg, std::size(msg), fmt, argptr);
    va_end(argptr);

    logImpl(level, msg);
  }

  void logdebug(const char* msg) const
  {
    logImpl(rosserial_msgs::Log::ROSDEBUG, msg);
  }
  void loginfo(const char * msg) const
  {
    logImpl(rosserial_msgs::Log::INFO, msg);
  }
  void logwarn(const char *msg) const
  {
    logImpl(rosserial_msgs::Log::WARN, msg);
  }
  void logerror(const char*msg) const
  {
    logImpl(rosserial_msgs::Log::ERROR, msg);
  }
  void logfatal(const char*msg) const
  {
    logImpl(rosserial_msgs::Log::FATAL, msg);
  }

protected:
  mutable bool param_received{false};
  rosserial_msgs::RequestParamResponse req_param_resp{};

  virtual bool requestParam(const char * name, uint32_t time_out) const = 0;
  virtual void setParamXmlRpcValue(const char* name, const std::string& valueXmlRpc) const = 0;
  virtual void logImpl(rosserial_msgs::Log::_level_type level, const char * msg) const = 0;
};
}

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/service_server.h"
#include "ros/service_client.h"

namespace ros
{

const int SPIN_OK = 0;
const int SPIN_ERR = -1;
const int SPIN_TIMEOUT = -2;
const int SPIN_TX_STOP_REQUESTED = -3;
const int SPIN_TIME_RECV = -4;

const uint8_t SYNC_SECONDS  = 5;
const uint8_t MODE_FIRST_FF = 0;
/*
 * The second sync byte is a protocol version. It's value is 0xff for the first
 * version of the rosserial protocol (used up to hydro), 0xfe for the second version
 * (introduced in hydro), 0xfd for the next, and so on. Its purpose is to enable
 * detection of mismatched protocol versions (e.g. hydro rosserial_python with groovy
 * rosserial_arduino. It must be changed in both this file and in
 * rosserial_python/src/rosserial_python/SerialClient.py
 */
const uint8_t MODE_PROTOCOL_VER   = 1;
const uint8_t PROTOCOL_VER1       = 0xff; // through groovy
const uint8_t PROTOCOL_VER2       = 0xfe; // in hydro
const uint8_t PROTOCOL_VER        = PROTOCOL_VER2;
const uint8_t MODE_SIZE_L         = 2;
const uint8_t MODE_SIZE_H         = 3;
const uint8_t MODE_SIZE_CHECKSUM  = 4;    // checksum for msg size received from size L and H
const uint8_t MODE_TOPIC_L        = 5;    // waiting for topic id
const uint8_t MODE_TOPIC_H        = 6;
const uint8_t MODE_MESSAGE        = 7;
const uint8_t MODE_MSG_CHECKSUM   = 8;    // checksum for msg and topic id


const uint8_t SERIAL_MSG_TIMEOUT  = 20;   // 20 milliseconds to recieve all of message data

static constexpr rosserial_msgs::Log::_level_type MIN_LOCAL_LOGGING_LEVEL {rosserial_msgs::Log::INFO};

using rosserial_msgs::TopicInfo;

/* Node Handle */
template<class Hardware = HardwareTemplate,
         int MAX_SUBSCRIBERS = 25,
         int MAX_PUBLISHERS = 25,
         int INPUT_SIZE = 512,
         int OUTPUT_SIZE = 512>
class NodeHandle_ : public NodeHandleBase_
{
protected:
  std::unique_ptr<Hardware> hardware_{std::make_unique<Hardware>()};

  /* time used for syncing */
  uint32_t rt_time{0};

  /* used for computing current time */
  uint32_t sec_offset{0}, nsec_offset{0};

  /* Spinonce maximum work timeout */
  uint32_t spin_timeout_{0};

  mutable uint8_t message_in[INPUT_SIZE] = {0};
  mutable uint8_t message_out[OUTPUT_SIZE] = {0};

  Publisher* publishers[MAX_PUBLISHERS] = {nullptr};
  Subscriber_* subscribers[MAX_SUBSCRIBERS] {nullptr};

  /*
   * Setup Functions
   */
public:
  Hardware* getHardware()
  {
    return hardware_.get();
  }

  /* Start serial, initialize buffers */
  void initNode() override
  {
    hardware_->init();
    Time::nh = this;
    mode_ = 0;
    bytes_ = 0;
    index_ = 0;
    topic_ = 0;
  };

  /* Start a named port, which may be network server IP, initialize buffers */
  void initNode(const char* portName) override
  {
    hardware_->init(portName);
    Time::nh = this;
    mode_ = 0;
    bytes_ = 0;
    index_ = 0;
    topic_ = 0;
  };

  /**
   * @brief Sets the maximum time in millisconds that spinOnce() can work.
   * This will not effect the processing of the buffer, as spinOnce processes
   * one byte at a time. It simply sets the maximum time that one call can
   * process for. You can choose to clear the buffer if that is beneficial if
   * SPIN_TIMEOUT is returned from spinOnce().
   * @param timeout The timeout in milliseconds that spinOnce will function.
   */
  void setSpinTimeout(const uint32_t& timeout)
  {
     spin_timeout_ = timeout;
  }

protected:
  // State machine variables for spinOnce
  int mode_{0};
  int bytes_{0};
  int topic_{0};
  int index_{0};
  int checksum_{0};

  mutable bool configured_{false};

  /* used for syncing the time */
  uint32_t last_sync_time{0};
  uint32_t last_sync_receive_time{0};
  uint32_t last_msg_timeout_time{0};

public:
  /* This function goes in your loop() function, it handles
   *  serial input and callbacks for subscribers.
   */


  int spinOnce() override
  {
    if (!hardware_->checkConnection())
    {
      if (configured_)
        printf("ROS connection lost, reconnecting\r\n");
      configured_ = false;
      return SPIN_ERR;
    }

    // Hardware is assumed to be connected from now on

    /* restart if timed out */
    uint32_t c_time = hardware_->time();
    if (configured_ && c_time - last_sync_receive_time > SYNC_SECONDS * 2200)
    {
      printf("ROS connection lost, reconnecting\r\n");
      configured_ = false;
      hardware_->disconnect();
    }

    /* reset if message has timed out */
    if (mode_ != MODE_FIRST_FF)
    {
      if (c_time > last_msg_timeout_time)
      {
        mode_ = MODE_FIRST_FF;
      }
    }

    bool tx_stop_requested = false;
    bool saw_time_msg = false;

    /* while available buffer, read data */
    while (true)
    {
      // If a timeout has been specified, check how long spinOnce has been running.
      if (spin_timeout_ > 0)
      {
        // If the maximum processing timeout has been exceeded, exit with error.
        // The next spinOnce can continue where it left off, or optionally
        // based on the application in use, the hardware buffer could be flushed
        // and start fresh.
        if ((hardware_->time() - c_time) > spin_timeout_)
        {
          // Exit the spin, processing timeout exceeded.
          return SPIN_TIMEOUT;
        }
      }
      int data = hardware_->read();
      if (data < 0)
        break;
      checksum_ += data;
      if (mode_ == MODE_MESSAGE)          /* message data being recieved */
      {
        message_in[index_++] = data;
        bytes_--;
        if (bytes_ == 0)                 /* is message complete? if so, checksum */
          mode_ = MODE_MSG_CHECKSUM;
      }
      else if (mode_ == MODE_FIRST_FF)
      {
        if (data == 0xff)
        {
          mode_++;
          last_msg_timeout_time = c_time + SERIAL_MSG_TIMEOUT;
        }
        else if (hardware_->time() - c_time > (SYNC_SECONDS * 1000))
        {
          /* We have been stuck in spinOnce too long, return error */
          configured_ = false;
          return SPIN_TIMEOUT;
        }
      }
      else if (mode_ == MODE_PROTOCOL_VER)
      {
        if (data == PROTOCOL_VER)
        {
          mode_++;
        }
        else
        {
          mode_ = MODE_FIRST_FF;
          if (configured_ == false)
            requestSyncTime();  /* send a msg back showing our protocol version */
        }
      }
      else if (mode_ == MODE_SIZE_L)      /* bottom half of message size */
      {
        bytes_ = data;
        index_ = 0;
        mode_++;
        checksum_ = data;               /* first byte for calculating size checksum */
      }
      else if (mode_ == MODE_SIZE_H)      /* top half of message size */
      {
        bytes_ += data << 8;
        mode_++;
      }
      else if (mode_ == MODE_SIZE_CHECKSUM)
      {
        if ((checksum_ % 256) == 255)
          mode_++;
        else
          mode_ = MODE_FIRST_FF;          /* Abandon the frame if the msg len is wrong */
      }
      else if (mode_ == MODE_TOPIC_L)     /* bottom half of topic id */
      {
        topic_ = data;
        mode_++;
        checksum_ = data;               /* first byte included in checksum */
      }
      else if (mode_ == MODE_TOPIC_H)     /* top half of topic id */
      {
        topic_ += data << 8;
        mode_ = MODE_MESSAGE;
        if (bytes_ == 0)
          mode_ = MODE_MSG_CHECKSUM;
      }
      else if (mode_ == MODE_MSG_CHECKSUM)    /* do checksum */
      {
        mode_ = MODE_FIRST_FF;
        if ((checksum_ % 256) == 255)
        {
          if (topic_ == TopicInfo::ID_PUBLISHER)
          {
            requestSyncTime();
            negotiateTopics();
            last_sync_time = c_time;
            last_sync_receive_time = c_time;
            return SPIN_ERR;
          }
          else if (topic_ == TopicInfo::ID_TIME)
          {
            saw_time_msg = true;
            syncTime(message_in);
          }
          else if (topic_ == TopicInfo::ID_PARAMETER_REQUEST)
          {
            req_param_resp.deserialize(message_in);
            param_received = true;
          }
          else if (topic_ == TopicInfo::ID_TX_STOP)
          {
            configured_ = false;
            tx_stop_requested = true;
            hardware_->disconnect();
          }
          else
          {
            if (subscribers[topic_ - 100])
              subscribers[topic_ - 100]->callback(message_in);
          }
        }
      }
    }

    /* occasionally sync time */
    if (connected() && ((c_time - last_sync_time) > (SYNC_SECONDS * 500)))
    {
      requestSyncTime();
      last_sync_time = c_time;
    }

    return saw_time_msg ? SPIN_TIME_RECV : (tx_stop_requested ? SPIN_TX_STOP_REQUESTED : SPIN_OK);
  }


  /* Are we connected to the PC? */
  bool connected() const override
  {
    if (!hardware_->connected())
    {
      if (configured_)
        printf("ROS connection lost, reconnecting\r\n");
      configured_ = false;
    }
    return configured_;
  };

  bool checkHardwareConnection() override
  {
    if (!hardware_->checkConnection())
    {
      if (configured_)
        printf("ROS connection lost, reconnecting\r\n");
      configured_ = false;
      return false;
    }
    return true;
  }

  /********************************************************************
   * Time functions
   */

  void requestSyncTime()
  {
    std_msgs::Time t;
    publish(TopicInfo::ID_TIME, &t);
    rt_time = hardware_->time();
  }

  void syncTime(uint8_t * data)
  {
    std_msgs::Time t;
    uint32_t offset = hardware_->time() - rt_time;

    t.deserialize(data);
    t.data.sec += offset / 1000;
    t.data.nsec += (offset % 1000) * 1000000UL;

    this->setNow(t.data);
    last_sync_receive_time = hardware_->time();
  }

  Time convertHWTime(const uint32_t hwTime) const
  {
    Time current_time;
    current_time.sec = hwTime / 1000 + sec_offset;
    current_time.nsec = (hwTime % 1000) * 1000000UL + nsec_offset;
    normalizeSecNSec(current_time.sec, current_time.nsec);
    return current_time;
  }

  Time now() const override
  {
    return convertHWTime(hardware_->time());
  }

  void setNow(const Time & new_now)
  {
    uint32_t ms = hardware_->time();
    sec_offset = new_now.sec - ms / 1000 - 1;
    nsec_offset = new_now.nsec - (ms % 1000) * 1000000UL + 1000000000UL;
    normalizeSecNSec(sec_offset, nsec_offset);
  }

  /********************************************************************
   * Topic Management
   */

  /* Register a new publisher */
  bool advertise(Publisher & p) override
  {
    for (int i = 0; i < MAX_PUBLISHERS; i++)
    {
      if (publishers[i] == 0) // empty slot
      {
        publishers[i] = &p;
        p.id_ = i + 100 + MAX_SUBSCRIBERS;
        p.nh_ = this;
        return true;
      }
    }
    printf("ERROR: Ran out of publishers!");
    return false;
  }

  /* Register a new subscriber */
  bool subscribe(Subscriber_& s) override
  {
    for (int i = 0; i < MAX_SUBSCRIBERS; i++)
    {
      if (subscribers[i] == 0) // empty slot
      {
        subscribers[i] = &s;
        s.id_ = i + 100;
        return true;
      }
    }
    printf("ERROR: Ran out of subscribers!");
    return false;
  }

  void negotiateTopics()
  {
    rosserial_msgs::TopicInfo ti;
    int i;
    for (i = 0; i < MAX_PUBLISHERS; i++)
    {
      if (publishers[i] != 0) // non-empty slot
      {
        ti.topic_id = publishers[i]->id_;
        ti.topic_name = (char *) publishers[i]->topic_;
        ti.message_type = (char *) publishers[i]->msg_->getType();
        ti.md5sum = (char *) publishers[i]->msg_->getMD5();
        ti.buffer_size = OUTPUT_SIZE;
        publish(publishers[i]->getEndpointType(), &ti);
        printf("Publisher %i: %s\r\n", ti.topic_id, ti.topic_name);
      }
    }
    for (i = 0; i < MAX_SUBSCRIBERS; i++)
    {
      if (subscribers[i] != 0) // non-empty slot
      {
        ti.topic_id = subscribers[i]->id_;
        ti.topic_name = (char *) subscribers[i]->topic_;
        ti.message_type = (char *) subscribers[i]->getMsgType();
        ti.md5sum = (char *) subscribers[i]->getMsgMD5();
        ti.buffer_size = INPUT_SIZE;
        publish(subscribers[i]->getEndpointType(), &ti);
        printf("Subscriber %i: %s\r\n", ti.topic_id, ti.topic_name);
      }
    }
    configured_ = true;
    for (i = 0; i < MAX_PUBLISHERS; i++)
    {
      if (publishers[i] != 0) // non-empty slot
        publishers[i]->publishLatched();
    }
    printf("ROS connected\r\n");
  }

  int publish(int id, const Msg * msg) const override
  {
    if (id >= 100 && !connected())
      return 0;

    /* serialize message */
    int l = msg->serialize(message_out + 7);

    if (l > OUTPUT_SIZE)
    {
      logerror("Message from device dropped: message larger than buffer.");
      return -1;
    }

    /* setup the header */
    message_out[0] = 0xff;
    message_out[1] = PROTOCOL_VER;
    message_out[2] = (uint8_t)((uint16_t)l & 255);
    message_out[3] = (uint8_t)((uint16_t)l >> 8);
    message_out[4] = 255 - ((message_out[2] + message_out[3]) % 256);
    message_out[5] = (uint8_t)((int16_t)id & 255);
    message_out[6] = (uint8_t)((int16_t)id >> 8);

    /* calculate checksum */
    int chk = 0;
    for (int i = 5; i < l + 7; i++)
      chk += message_out[i];
    l += 7;
    message_out[l++] = 255 - (chk % 256);

    if (!hardware_->write(message_out, l))
    {
      printf("ERROR: Failed publishing message of length %u on topic %u.\r\n", l, id);
      return -l;
    }
    return l;
  }

  /********************************************************************
   * Logging
   */

protected:
  static constexpr const char* LOG_LEVELS[rosserial_msgs::Log::FATAL + 1] = {
    "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
  };

  void printf(const char* fmt, ...) const override
  {
    va_list argptr;
    va_start(argptr, fmt);
    hardware_->printf(fmt, argptr);
    va_end(argptr);
  }

  void logImpl(rosserial_msgs::Log::_level_type level, const char * msg) const override
  {
    rosserial_msgs::Log l;
    l.level = level;
    l.msg = msg;
    if (level >= MIN_LOCAL_LOGGING_LEVEL)
      printf("[%5s] [%lu]: %s\r\n", LOG_LEVELS[level], hardware_->time(), msg);
    publish(rosserial_msgs::TopicInfo::ID_LOG, &l);
  }
  

  /********************************************************************
   * Parameters
   */

  bool requestParam(const char * name, uint32_t time_out) const override
  {
    param_received = false;
    rosserial_msgs::RequestParamRequest req;
    req.name  = (char*)name;
    publish(TopicInfo::ID_PARAMETER_REQUEST, &req);
    uint32_t end_time = hardware_->time() + time_out;
    while (!param_received)
    {
      const_cast<NodeHandle_*>(this)->spinOnce();
      if (hardware_->time() > end_time)
      {
        log(rosserial_msgs::Log::WARN, "Failed to get param '%s': timeout expired", name);
        return false;
      }
    }
    return true;
  }

  void setParamXmlRpcValue(const char* name, const std::string& valueXmlRpc) const override
  {
    std::stringstream ss;
    ss << "<value><struct>";
    ss << "<member><name>name</name><value>" << name << "</value></member>";
    ss << "<member><name>value</name><value>" << valueXmlRpc << "</value></member>";
    ss << "</struct></value>";

    const auto xmlRpcReq = ss.str();

    std_msgs::String req;
    req.data  = xmlRpcReq.c_str();
    publish(66, &req);
  }
  
};

}

#endif
