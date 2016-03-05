#include <vigir_generic_params/rosparam_handler.h>

namespace vigir_generic_params
{
RosparamHandler::RosparamHandler(const std::string& private_ns, const std::string& public_ns, const ros::NodeHandle& nh)
  : private_ns_(private_ns)
  , public_ns_(public_ns)
  , root_nh_(nh)
{
}
}
