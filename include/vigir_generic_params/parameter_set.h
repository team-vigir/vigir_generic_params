//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
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
  // typedefs
  typedef boost::shared_ptr<ParameterSet> Ptr;
  typedef boost::shared_ptr<const ParameterSet> ConstPtr;

  ParameterSet(const std::string& name = std::string());
  ParameterSet(const XmlRpc::XmlRpcValue& val) { fromXmlRpcValue(val); }
  ParameterSet(const ParameterSetMsg& params) { fromMsg(params); }

  friend std::ostream& operator<<(std::ostream& os, const ParameterSet& params);

  friend ParameterSet operator+(ParameterSet lhs, const ParameterSet& rhs);

  void clear();

  inline bool empty() const { return params_.size() == 0; }
  inline unsigned int size() const { return params_.size(); }

  inline void setName(const std::string& name) { setParam("name", name); } // Use it carefully as parameter manager isn't aware of any (internal) renaming!
  inline const std::string& getName() const { return name_; }

  /**
   * Adds or overwrites param with given name.
   */
  template<typename T>
  void setParam(const std::string& key, const T& p)
  {
    setParam(key, XmlRpc::XmlRpcValue(p));
  }
  template<typename T>
  void setParam(const std::string& key, const std::vector<T>& p)
  {
    XmlRpc::XmlRpcValue val;
    val.setSize(p.size());

    for (size_t i = 0; i < p.size(); i++)
      val[i] = p[i];

    setParam(key, val);
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
  bool getParam(const std::string& key, std::vector<T>& p) const
  {
    p.clear();

    XmlRpc::XmlRpcValue val;
    if (!getParam(key, val))
      return false;

    if (val.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("[getParam] Expected XmlRpc type 'TypeArray' but got '%s' for parameter '%s'", vigir_generic_params::toString(val.getType()).c_str(), key.c_str());
      return false;
    }

    p.resize(val.size());

    XmlRpc::XmlRpcValue::Type type = XmlRpc::XmlRpcValue(T()).getType();
    for (size_t i = 0; i < val.size(); i++)
    {
      if (type != val[i].getType())
      {
        ROS_ERROR("[getParam] Expected XmlRpc type '%s' but got '%s' for element of list '%s'", vigir_generic_params::toString(XmlRpc::XmlRpcValue(T()).getType()).c_str(), vigir_generic_params::toString(val.getType()).c_str(), key.c_str());
        return false;
      }
      p[i] = (T)val[i];
    }

    return true;
  }

  /**
   * @brief Retrieves parameter from parameter set
   * @param key key of parameter
   * @param p [out] return variable for parameter
   * @param default_val default value
   * @param ignore_warnings (default = false) When set to true, no warnings will be printed if param is not available
   * @return true when parameter was found
   */
  template<typename T>
  bool getParam(const std::string& key, T& p, const T& default_val, bool ignore_warnings = false) const
  {
    if ((ignore_warnings && !hasParam(key)) || !getParam(key, p))
    {
      p = default_val;
      return false;
    }
    else
      return true;
  }

  /**
   * @brief Retrieves parameter from parameter set
   * @param key key of parameter
   * @param default_val default value
   * @param ignore_warnings (default = false) When set to true, no warnings will be printed if param is not available
   * @return retrieved parameter if available, otherwise given default value
   */
  template<typename T>
  T param(const std::string& key, const T& default_val, bool ignore_warnings = false) const
  {
    T val;
    getParam(key, val, default_val, ignore_warnings);
    return val;
  }

  /**
   * @brief Determines if param with given key exists in set.
   * @param key parameter name
   * @return true if parameter with given key exists
   */
  bool hasParam(const std::string& key) const;

  /**
   * @brief merge Merges other parameter set into this one
   * @param other other parameter set
   */
  void merge(const ParameterSet& other);

  ParameterSet getSubset(const std::string& key) const;

  bool updateFromXmlRpcValue(const XmlRpc::XmlRpcValue& val);
  bool fromXmlRpcValue(const XmlRpc::XmlRpcValue& val);

  void updateFromMsg(const ParameterSetMsg& param_set);
  void fromMsg(const ParameterSetMsg& param_set);
  void toMsg(ParameterSetMsg& param_set) const;

  /**
   * @brief Generates list of all namespaces at given level
   * @param level level for namespaces to lookup. 0 is the immediate next level.
   * @param include_params if true, leaf names (parameter names) will be included
   * @return list of all namespaces at given level
   */
  std::list<std::string> getNamespaces(unsigned level = 0, bool include_params = false) const;

  std::string toString() const;

protected:
  /**
   * Recursively adds parameters from XmlRpc
   */
  bool addFromXmlRpcValue(const std::string& ns, const XmlRpc::XmlRpcValue& val);

  /**
   * @brief Extract any namespace at specific level from given key
   * @param key key from where the namespace should be extracted
   * @param level level of namespace
   * @param include_params if true, then parameters will be traded as namespace too
   * @return
   */
  std::string extractNamespaceAtLevel(const std::string& key, unsigned int level, bool include_params) const;

  std::string name_;
  std::map<std::string, XmlRpc::XmlRpcValue> params_;
};

template<> void ParameterSet::setParam(const std::string& key, const XmlRpc::XmlRpcValue& p);
template<> void ParameterSet::setParam(const std::string& key, const ros::NodeHandle& nh);

template<> bool ParameterSet::getParam(const std::string& key, XmlRpc::XmlRpcValue& p) const;
template<> bool ParameterSet::getParam(const std::string& key, unsigned int& p) const;
template<> bool ParameterSet::getParam(const std::string& key, float& p) const;
template<> bool ParameterSet::getParam(const std::string& key, ParameterMsg& p) const;

template<> bool ParameterSet::getParam(const std::string& key, ParameterSet& p) const; // derives subset from namespace given as key
}

#endif
