#include <vigir_generic_params/parameter_set.h>

namespace vigir_generic_params
{
ParameterSet::ParameterSet(const std::string& name)
  : name_(name)
{
}

ParameterSet::ParameterSet(const XmlRpc::XmlRpcValue& val)
{
  fromXmlRpcValue(val);
}

ParameterSet::ParameterSet(const ParameterSetMsg& params)
{
  fromMsg(params);
}

ParameterSet::~ParameterSet()
{
}

std::ostream& operator<<(std::ostream& os, const ParameterSet& params)
{
  os << "Name: " << params.getName() << ", size: " << params.params_.size();

  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = params.params_.begin(); itr != params.params_.end(); itr++)
    os << "\n" << itr->first << ": " /*<< itr->second*/;

  return os;
}

void ParameterSet::clear()
{
  name_ = "";
  params_.clear();
}

unsigned int ParameterSet::size() const
{
  return params_.size();
}

void ParameterSet::setName(const std::string& name)
{
  setParam("name", name);
}

const std::string& ParameterSet::getName() const
{
  return name_;
}

template<>
void ParameterSet::setParam(const std::string& key, const XmlRpc::XmlRpcValue& p)
{
  if (p.valid())
  {
    // strip '/' from key
    std::string _key = strip_const(key, '/');

    if (_key.empty())
    {
      ROS_WARN("[ParameterSet] setParam: Got empty key. Skipping!");
      return;
    }

    // key should be always lower case
    std::transform(_key.begin(), _key.end(), _key.begin(), ::tolower);

    // special case for 'name' key
    if(_key == "name")
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

template<>
void ParameterSet::setParam(const std::string& key, const ros::NodeHandle& nh)
{
  try
  {
    if (!nh.hasParam(key))
    {
      ROS_WARN("[ParameterSet] setParam: Requested parameter '%s' was not found in '%s'!", key.c_str(), (nh.getNamespace() + "/" + key).c_str());
      return;
    }

    XmlRpc::XmlRpcValue p;
    nh.getParam(key, p);
    setParam(key, p);
  }
  catch(std::exception& e)
  {
    ROS_ERROR("[ParameterSet] setParam: Catched exception while retrieving '%s': %s", key.c_str(), e.what());
    return;
  }
}

void ParameterSet::setParam(const ParameterMsg& msg)
{
  XmlRpc::XmlRpcValue val;
  val << msg.data;
  setParam(msg.key.data, val);
}

template<>
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

template<>
bool ParameterSet::getParam(const std::string& key, unsigned int& p) const
{
  return getParam(key, (int&)p);
}

template<>
bool ParameterSet::getParam(const std::string& key, float& p) const
{
  double _p;
  bool success = getParam(key, _p);
  p = _p;
  return success;
}

template<>
bool ParameterSet::getParam(const std::string& key, ParameterMsg& p) const
{
  XmlRpc::XmlRpcValue val;
  if (!getParam(key, val))
    return false;

  p.key.data = key;
  return p.data << val;
}

bool ParameterSet::hasParam(const std::string& key) const
{
  return params_.find(key) != params_.end();
}

bool ParameterSet::fromXmlRpcValue(const XmlRpc::XmlRpcValue& val)
{
  XmlRpc::XmlRpcValue& _val = const_cast<XmlRpc::XmlRpcValue&>(val); // needed because XmlRpc doesn't implement const getters

  if (_val.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("[ParameterSet] fromXmlRpcValue: Invalid format given! Expected TypeStruct.");
    return false;
  }

  if (!_val.hasMember("name") || _val["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROS_ERROR("[ParameterSet] fromXmlRpcValue: Invalid format: Parameter set must contain 'name'!");
    return false;
  }

  // start parsing recursively
  return addXmlRpcValue("", _val);
}

void ParameterSet::updateFromMsg(const ParameterSetMsg& param_Set)
{
  for (std::vector<ParameterMsg>::const_iterator itr = param_Set.params.begin(); itr != param_Set.params.end(); itr++)
    setParam(*itr);
  setName(param_Set.name.data);
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

  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = this->params_.begin(); itr != this->params_.end(); itr++)
  {
    ParameterMsg param;
    param.key.data = itr->first;
    param.data << itr->second;
    param_set.params.push_back(param);
  }
}

std::string ParameterSet::toString() const
{
  std::ostringstream ss;

  ss << "Set name: " << name_;
  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = this->params_.begin(); itr != this->params_.end(); itr++)
    ss << "\n" << itr->first << ": " << vigir_generic_params::toString(itr->second);

  return ss.str();
}

bool ParameterSet::addXmlRpcValue(const std::string& ns, XmlRpc::XmlRpcValue& val)
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
      bool result  = true;
      for (XmlRpc::XmlRpcValue::iterator itr = val.begin(); itr != val.end() && result; itr++)
        result = addXmlRpcValue(ns + (ns.empty() ? itr->first : "/" + itr->first), itr->second);
      return result;
    }
    default:
      ROS_ERROR("[ParameterSet] addXmlRpcValue: Unknown type '%u'!", val.getType());
      return false;
  }
}
}
