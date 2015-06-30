//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_GENERIC_PARAMS_PARAMETER_SET_H__
#define VIGIR_GENERIC_PARAMS_PARAMETER_SET_H__

#include <iostream>
#include <algorithm>

#include <ros/ros.h>

#include <vigir_generic_params/generic_params_msgs.h>



namespace vigir_generic_params
{
/**
 * This class wraps up the vigir_footstep_planning_ParameterSetMsg message and gives access
 * to the stored parameters.
 */
class ParameterSet
{
public:
  ParameterSet(const std::string& name = std::string());
  ParameterSet(const XmlRpc::XmlRpcValue& val);
  ParameterSet(const ParameterSetMsg& params);
  virtual ~ParameterSet();

  friend std::ostream& operator<<(std::ostream& os, const ParameterSet& params);

  void clear();

  unsigned int size() const;

  void setName(const std::string& name); // Use it carefully as parameter manager isn't aware of any (internal) renaming!
  const std::string& getName() const;

  /**
   * Adds or overwrites param with given name.
   */
  template<typename T>
  void setParam(const std::string& key, const T& p)
  {
    setParam(key, XmlRpc::XmlRpcValue(p));
  }

  void setParam(const ParameterMsg& msg);

  /**
   * Retrieves param with given name. If param couldn't be found false will be returned.
   */
  template<typename T>
  bool getParam(const std::string& key, T& p) const
  {
    p = T();

    XmlRpc::XmlRpcValue val;
    if (!getParam(key, val))
      return false;

    if (XmlRpc::XmlRpcValue(T()).getType() != val.getType())
    {
      ROS_ERROR("[getParam] Expected XmlRpc type '%s' but got '%s' for parameter '%s'", vigir_generic_params::toString(XmlRpc::XmlRpcValue(T()).getType()).c_str(), vigir_generic_params::toString(val.getType()).c_str(), key.c_str());
      return false;
    }

    p = (T)val;
    return true;
  }
  template<typename T>
  bool getParam(const std::string& key, T& p, const T& default_val) const
  {
    if (getParam<T>(key, p))
      return true;
    else
    {
      p = default_val;
      return false;
    }
  }

  bool hasParam(const std::string& key) const;

  bool fromXmlRpcValue(const XmlRpc::XmlRpcValue& val);

  void updateFromMsg(const ParameterSetMsg& param_set);
  void fromMsg(const ParameterSetMsg& param_set);
  void toMsg(ParameterSetMsg& param_set) const;

  std::string toString() const;

  // typedefs
  typedef boost::shared_ptr<ParameterSet> Ptr;
  typedef boost::shared_ptr<const ParameterSet> ConstPtr;

protected:
  /**
   * Recursively adds parameters from XmlRpc
   */
  bool addXmlRpcValue(const std::string& ns, XmlRpc::XmlRpcValue& val);

  std::string name;
  std::map<std::string, XmlRpc::XmlRpcValue> params;
};

template<> void ParameterSet::setParam(const std::string& key, const XmlRpc::XmlRpcValue& p);
template<> void ParameterSet::setParam(const std::string& key, const ros::NodeHandle& nh);

template<> bool ParameterSet::getParam(const std::string& key, XmlRpc::XmlRpcValue& p) const;
template<> bool ParameterSet::getParam(const std::string& key, unsigned int& p) const;
template<> bool ParameterSet::getParam(const std::string& key, float& p) const;
template<> bool ParameterSet::getParam(const std::string& key, ParameterMsg& p) const;
}

#endif
