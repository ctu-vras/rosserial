/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009-2010, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/


/**

 Author: Blaise Gassend

 Handles synchronizing node state with the configuration server, and
 handling of services to get and set configuration.

*/

#ifndef __SERVER_H__
#define __SERVER_H__

#include <functional>

#include <ros.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

/**
 * @todo Add diagnostics.
 */

namespace dynamic_reconfigure
{
/**
 * Keeps track of the reconfigure callback function.
 */
template <class ConfigType>
class Server
{
public:
  Server(const std::string& ns, ConfigType* config) : ns_(ns), config_(*config),
    set_service_topic_(ns + "/set_parameters"), update_topic_(ns + "/parameter_updates"),
    descr_topic_(ns + "/parameter_descriptions"),
    set_service_(set_service_topic_.c_str(), &Server::setConfigCallback, this),
    update_pub_(update_topic_.c_str(), &config_msg_, true),
    descr_pub_(descr_topic_.c_str(), &description_message, true)
  {
  }

  void setup(ros::NodeHandle& nh)
  {
    node_handle_ = &nh;
    nh.advertiseService(set_service_);
    nh.advertise(update_pub_);
    nh.advertise(descr_pub_);
  }

  void init(ros::NodeHandle& nh)
  {
    //Grab copys of the data from the config files.  These are declared in the generated config file.
    min_ = ConfigType::__getMin__();
    max_ = ConfigType::__getMax__();
    default_ = ConfigType::__getDefault__();

    description_message = ConfigType::__getDescriptionMessage__();
    descr_pub_.publish(&description_message);

    ConfigType init_config = ConfigType::__getDefault__();
    init_config.__fromServer__(nh, ns_);
    init_config.__clamp__();
    updateConfigInternal(init_config);
  }

  typedef std::function<void(ConfigType &, uint32_t level)> CallbackType;

  // Only call this after init().
  void setCallback(const CallbackType &callback)
  {
    callback_ = callback;
    callCallback(config_, ~0); // At startup we need to load the configuration with all level bits set. (Everything has changed.)
    updateConfigInternal(config_);
  }

  void clearCallback()
  {
    callback_.clear();
  }

  void updateConfig(const ConfigType &config)
  {
    updateConfigInternal(config);
  }


  void getConfigMax(ConfigType &config)
  {
    config = max_;
  }

  void getConfigMin(ConfigType &config)
  {
    config = min_;
  }

  void getConfigDefault(ConfigType &config)
  {
    config = default_;
  }

  void setConfigMax(const ConfigType &config)
  {
    max_ = config;
    PublishDescription();
  }

  void setConfigMin(const ConfigType &config)
  {
    min_ = config;
    PublishDescription();
  }

  void setConfigDefault(const ConfigType &config)
  {
    default_ = config;
    PublishDescription();
  }


private:
  std::string ns_;
  ros::NodeHandle* node_handle_;
  ConfigType& config_;
  std::string set_service_topic_;
  std::string update_topic_;
  std::string descr_topic_;
  ros::ServiceServer<ReconfigureRequest, ReconfigureResponse, Server> set_service_;
  ros::Publisher update_pub_;
  ros::Publisher descr_pub_;
  CallbackType callback_;
  ConfigType min_;
  ConfigType max_;
  ConfigType default_;
  ConfigDescription description_message;
  Config config_msg_;



  void PublishDescription()
  {
    //Copy over min_ max_ default_
    description_message = ConfigType::__getDescriptionMessage__();

    max_.__toMessage__(description_message.max, ConfigType::__getParamDescriptions__(),ConfigType::__getGroupDescriptions__());
    min_.__toMessage__(description_message.min,ConfigType::__getParamDescriptions__(),ConfigType::__getGroupDescriptions__());
    default_.__toMessage__(description_message.dflt,ConfigType::__getParamDescriptions__(),ConfigType::__getGroupDescriptions__());

    //Publish description
    descr_pub_.publish(&description_message);
  }

  void callCallback(ConfigType &config, int level)
  {
    if (callback_) // At startup we need to load the configuration with all level bits set. (Everything has changed.)
      callback_(config, level);
  }

  void setConfigCallback(const Reconfigure::Request &req, Reconfigure::Response &rsp)
  {
    ConfigType new_config = config_;
    new_config.__fromMessage__(req.config);
    new_config.__clamp__();
    uint32_t level = config_.__level__(new_config);

    callCallback(new_config, level);

    updateConfigInternal(new_config);
    new_config.__toMessage__(rsp.config);
  }

  void updateConfigInternal(const ConfigType &config)
  {
    config_ = config;
    config_.__toServer__(*node_handle_, ns_);
    config_.__toMessage__(config_msg_);
    update_pub_.publish(&config_msg_);
  }
};

}
#endif
