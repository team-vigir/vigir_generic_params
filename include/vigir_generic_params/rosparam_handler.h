//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_GENERIC_PARAMS_ROSPARAM_HANDLER_H__
#define VIGIR_GENERIC_PARAMS_ROSPARAM_HANDLER_H__

#include <ros/ros.h>

#include <vigir_generic_params/generic_params_msgs.h>



namespace vigir_generic_params
{
class RosparamHandler
{
public:
  // typedefs
  typedef boost::shared_ptr<RosparamHandler> Ptr;
  typedef boost::shared_ptr<const RosparamHandler> ConstPtr;

  RosparamHandler(const std::string& private_ns, const std::string& public_ns, const ros::NodeHandle& nh = ros::NodeHandle());

  /**
   * @brief Retrieves parameter from rosparam server
   * @param name name of parameter
   * @param val [out] return variable for parameter
   * @param def default value
   * @param ignore_warnings if true no warnings will be printed out when param was not present
   * @return true when parameter was found at rosparam
   */
  template<typename T>
  bool getParam(const std::string& name, T& val, const T& def = T(), bool ignore_warnings = false) const
  {
    std::string name_private_ns = private_ns_ + "/" + name;
    std::string name_public_ns = public_ns_ + "/" + name;
    if (root_nh_.hasParam(name_private_ns) && root_nh_.getParam(name_private_ns, val))
      return true;
    else if (root_nh_.hasParam(name_public_ns) && root_nh_.getParam(name_public_ns, val))
      return true;
    else
    {
      val = def;
      if (!ignore_warnings)
      {
        if (private_ns_.empty() && public_ns_.empty())
          ROS_WARN("[RosparamHandler]: No parameters defined!");
        else
        {
          std::string model_ns = "/" + strip_const(root_nh_.getNamespace(), '/');
          ROS_WARN("[RosparamHandler]: '%s' parameter is missing in namespaces '%s...'", name.c_str(), model_ns.c_str());
          ROS_WARN("    private: '...%s'", ("/" + private_ns_).c_str());
          ROS_WARN("    public: '...%s'", ("/" + public_ns_).c_str());
        }
      }
      return false;
    }
  }

  /**
   * @brief Retrieves parameter from rosparam server
   * @param name name of parameter
   * @param def default value
   * @param ignore_warnings When true no warnings will be printed out when param was not present
   * @return retrieved parameter if available, otherwise given default value
   */
  template<typename T>
  T getParam(const std::string& name, const T& def = T(), bool ignore_warnings = false) const
  {
    T result;
    getParam(name, result, def, ignore_warnings);
    return result;
  }

private:
  ros::NodeHandle root_nh_;

  std::string private_ns_;
  std::string public_ns_;
};
}

#endif
