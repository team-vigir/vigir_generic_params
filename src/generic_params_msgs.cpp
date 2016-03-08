#include <vigir_generic_params/generic_params_msgs.h>



namespace vigir_generic_params
{
std::string toString(const XmlRpc::XmlRpcValue& val)
{
  std::ostringstream ss;
  val.write(ss);
  return ss.str();
}

std::string toString(const XmlRpc::XmlRpcValue::Type& type)
{
  switch (type)
  {
    case XmlRpc::XmlRpcValue::TypeInvalid:  return "TypeInvalid";
    case XmlRpc::XmlRpcValue::TypeBoolean:  return "TypeBoolean";
    case XmlRpc::XmlRpcValue::TypeInt:      return "TypeInt";
    case XmlRpc::XmlRpcValue::TypeDouble:   return "TypeDouble";
    case XmlRpc::XmlRpcValue::TypeString:   return "TypeString";
    case XmlRpc::XmlRpcValue::TypeDateTime: return "TypeDateTime";
    case XmlRpc::XmlRpcValue::TypeBase64:   return "TypeBase64";
    case XmlRpc::XmlRpcValue::TypeArray:    return "TypeArray";
    case XmlRpc::XmlRpcValue::TypeStruct:   return "TypeStruct";
    default: return "Unknown Type!";
  }
}

std::string& strip(std::string& s, const char c)
{
  while (!s.empty() && s[0] == c)
    s = s.substr(1);
  while (!s.empty() && s[s.size()-1] == c)
    s = s.substr(0, s.size()-1);
  return s;
}

std::string strip_const(const std::string& s, const char c)
{
  std::string _s = s;
  return strip(_s, c);
}
} // namespace
