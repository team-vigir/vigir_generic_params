#include <vigir_generic_params/parameter_set.h>

namespace vigir_generic_params
{
ParameterSet::ParameterSet(const std::string& name)
  : name_(name)
{}

std::ostream& operator<<(std::ostream& os, const ParameterSet& params)
{
  os << "Name: " << params.getName() << ", size: " << params.params_.size();

  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = params.params_.begin(); itr != params.params_.end(); itr++)
    os << "\n" << itr->first << ": " /*<< itr->second*/;

  return os;
}

ParameterSet operator+(ParameterSet lhs, const ParameterSet& rhs)
{
  lhs.merge(rhs);
  return lhs;
}

void ParameterSet::clear()
{
  name_ = "";
  params_.clear();
}

template <>
void ParameterSet::setParam(const std::string& key, const XmlRpc::XmlRpcValue& p)
{
  if (p.valid())
  {
    // strip '/' from key
    std::string _key = strip_const(key, '/');

    if (_key.empty())
    {
      ROS_WARN("[ParameterSet] setParam: Got empty key ('%s'). Skipping!", key.c_str());
      return;
    }

    // key should be always lower case
    std::transform(_key.begin(), _key.end(), _key.begin(), ::tolower);

    // special case for 'name' key
    if (_key == "name")
    {
      if (p.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        std::string old_name = name_;
        name_ = static_cast<std::string>(XmlRpc::XmlRpcValue(p));

        if (old_name == name_)
          return;

        if (!old_name.empty())
          ROS_INFO("[ParameterSet] setParam: Renamed parameter set '%s' to '%s'.", old_name.c_str(), name_.c_str());
      }
      else
      {
        ROS_ERROR("[ParameterSet] setParam: Parameter 'name' must be a string!");
        return;
      }
    }

    params_[_key] = p;
  }
  else
    ROS_ERROR("[ParameterSet] setParam: Type of parameter '%s' not supported!", key.c_str());
}

template <>
void ParameterSet::setParam(const std::string& key, const ros::NodeHandle& nh)
{
  try
  {
    if (!nh.hasParam(key))
    {
      ROS_WARN("[ParameterSet] setParam: Requested parameter '%s' was not found in '%s'!", key.c_str(), ros::names::append(nh.getNamespace(), key).c_str());
      return;
    }

    XmlRpc::XmlRpcValue p;
    nh.getParam(key, p);
    setParam(key, p);
  }
  catch (std::exception& e)
  {
    ROS_ERROR("[ParameterSet] setParam: Catched exception while retrieving '%s': %s", key.c_str(), e.what());
    return;
  }
}

void ParameterSet::setParam(const ParameterMsg& msg)
{
  XmlRpc::XmlRpcValue val;
  val << msg.data;
  addFromXmlRpcValue(msg.key.data, val);
}

template <>
bool ParameterSet::getParam(const std::string& key, XmlRpc::XmlRpcValue& p) const
{
  p = XmlRpc::XmlRpcValue();

  std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = params_.find(key);
  if (itr == params_.end())
  {
    ROS_ERROR("[ParameterSet] getParam: Couldn't find parameter '%s'!", key.c_str());
    return false;
  }

  p = itr->second;
  return true;
}

template <>
bool ParameterSet::getParam(const std::string& key, unsigned int& p) const
{
  return getParam(key, (int&)p);
}

template <>
bool ParameterSet::getParam(const std::string& key, float& p) const
{
  double _p;
  bool success = getParam(key, _p);
  p = _p;
  return success;
}

template <>
bool ParameterSet::getParam(const std::string& key, ParameterMsg& p) const
{
  XmlRpc::XmlRpcValue val;
  if (!getParam(key, val))
    return false;

  p.key.data = key;
  return p.data << val;
}

template <>
bool ParameterSet::getParam(const std::string& key, ParameterSet& p) const
{
  p.clear();

  // strip '/' from key
  std::string _key = strip_const(key, '/');

  for (auto kv : params_)
  {
    if (strncmp(_key.c_str(), kv.first.c_str(), _key.length()) == 0 && kv.first.length() != _key.length())
      p.setParam(kv.first.substr(_key.length()), kv.second);
  }

  return true;
}

template <>
bool ParameterSet::getParam(const std::string& key, std::vector<unsigned int>& p) const
{
  p.clear();

  std::vector<int> v;
  if (getParam(key, v, std::vector<int>()))
  {
    for (const int& i : v)
      p.push_back(static_cast<unsigned int>(i));

    return true;
  }

  return false;
}

template <>
bool ParameterSet::getParam(const std::string& key, std::vector<float>& p) const
{
  p.clear();

  std::vector<double> v;
  if (getParam(key, v, std::vector<double>()))
  {
    for (const double& d : v)
      p.push_back(static_cast<float>(d));

    return true;
  }

  return false;
}

bool ParameterSet::hasParam(const std::string& key) const { return params_.find(key) != params_.end(); }

bool ParameterSet::updateFromXmlRpcValue(const XmlRpc::XmlRpcValue& val)
{
  // start parsing recursively
  return addFromXmlRpcValue("", val);
}

bool ParameterSet::fromXmlRpcValue(const XmlRpc::XmlRpcValue& val)
{
  XmlRpc::XmlRpcValue& _val = const_cast<XmlRpc::XmlRpcValue&>(val);  // needed because XmlRpc doesn't implement const getters

  if (_val.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("[ParameterSet] fromXmlRpcValue: Invalid format given! Expected TypeStruct.");
    return false;
  }

  if (!_val.hasMember("name") || _val["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
    name_ = "NO_NAME_SPECIFIED";

  // start parsing recursively
  return addFromXmlRpcValue("", val);
}

void ParameterSet::merge(const ParameterSet& other)
{
  for (auto itr : other.params_)
    setParam(itr.first, itr.second);
}

ParameterSet ParameterSet::getSubset(const std::string& key) const
{
  ParameterSet params;
  getParam(key, params);
  return params;
}

void ParameterSet::updateFromMsg(const ParameterSetMsg& param_set)
{
  for (std::vector<ParameterMsg>::const_iterator itr = param_set.params.begin(); itr != param_set.params.end(); itr++)
    setParam(*itr);
  setName(param_set.name.data);
}

void ParameterSet::fromMsg(const ParameterSetMsg& param_set)
{
  clear();
  updateFromMsg(param_set);
}

void ParameterSet::toMsg(ParameterSetMsg& param_set) const
{
  param_set.params.clear();

  param_set.name.data = name_;

  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = params_.begin(); itr != params_.end(); itr++)
  {
    // we need only to serialize top level elements to prevent redundancies in the msg
    if (itr->first.find('/') == std::string::npos)
    {
      ParameterMsg param;
      param.key.data = itr->first;
      param.data << itr->second;
      param_set.params.push_back(param);
    }
  }
}

std::list<std::string> ParameterSet::getNamespaces(unsigned int level, bool include_params) const
{
  std::list<std::string> namespaces;

  for (auto param : params_)
  {
    std::string ns = extractNamespaceAtLevel(param.first, level, include_params);
    if (!ns.empty())
      namespaces.push_back(ns);
  }

  // remove duplicates
  namespaces.unique();

  return namespaces;
}

std::string ParameterSet::toString() const
{
  std::ostringstream ss;

  ss << "Set name: " << name_;
  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = this->params_.begin(); itr != this->params_.end(); itr++)
    ss << "\n" << itr->first << ": " << vigir_generic_params::toString(itr->second);

  return ss.str();
}

bool ParameterSet::addFromXmlRpcValue(const std::string& ns, const XmlRpc::XmlRpcValue& val)
{
  switch (val.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    case XmlRpc::XmlRpcValue::TypeInt:
    case XmlRpc::XmlRpcValue::TypeDouble:
    case XmlRpc::XmlRpcValue::TypeString:
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeBase64:
    case XmlRpc::XmlRpcValue::TypeArray:
    {
      setParam(ns, val);
      return true;
    }
    case XmlRpc::XmlRpcValue::TypeStruct:
    {
      // strip '/' from key
      std::string key = strip_const(ns, '/');
      if (!key.empty())  // exclude root element
        setParam(key, val);

      XmlRpc::XmlRpcValue& _val = const_cast<XmlRpc::XmlRpcValue&>(val);  // needed because XmlRpc doesn't implement const getters

      bool result = true;
      for (XmlRpc::XmlRpcValue::iterator itr = _val.begin(); itr != _val.end() && result; itr++)
        result = addFromXmlRpcValue(ns + (ns.empty() ? itr->first : "/" + itr->first), itr->second);
      return result;
    }
    default:
      ROS_ERROR("[ParameterSet] addXmlRpcValue: Unknown type '%u'!", val.getType());
      return false;
  }
}

std::string ParameterSet::extractNamespaceAtLevel(const std::string& key, unsigned int level, bool include_params) const
{
  std::string result = key;

  // if parameter names should be added, just add '/' to the end to trade them as namespace
  if (include_params)
    result.append("/");

  // check first level (a key without '/' can't be a namespace (it's a parameter))
  size_t pos = result.find("/");
  if (pos == std::string::npos)
    return std::string();

  // iterate into deeper levels if needed
  while (level-- > 0)
  {
    result = result.substr(pos + 1);
    pos = result.find("/");
    if (pos == std::string::npos)
      return std::string();
  }

  // remove tail
  pos = result.find("/");
  if (pos != std::string::npos)
    result = result.substr(0, pos);

  return result;
}
}  // namespace vigir_generic_params
