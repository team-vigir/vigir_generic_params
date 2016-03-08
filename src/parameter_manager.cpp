#include <vigir_generic_params/parameter_manager.h>



namespace vigir_generic_params
{
ParameterManager::Ptr ParameterManager::singelton_ = ParameterManager::Ptr();

ParameterManager::ParameterManager()
  : active_parameter_set_(new ParameterSet())
{
}

void ParameterManager::initialize(ros::NodeHandle& nh)
{
  // subscribe topics
  Instance()->update_parameter_set_sub_ = nh.subscribe("params/update_parameter_set", 1, &ParameterManager::updateParameterSet, Instance().get());

  // start own services
  Instance()->set_parameter_set_srv_ = nh.advertiseService("params/set_parameter_set", &ParameterManager::setParameterSetService, Instance().get());
  Instance()->get_parameter_set_srv_ = nh.advertiseService("params/get_parameter_set", &ParameterManager::getParameterSetService, Instance().get());
  Instance()->get_all_parameter_sets_srv_ = nh.advertiseService("params/get_all_parameter_sets", &ParameterManager::getAllParameterSetsService, Instance().get());
  Instance()->get_parameter_set_names_srv_ = nh.advertiseService("params/get_parameter_set_names", &ParameterManager::getParameterSetNamesService, Instance().get());

  // init action servers
  Instance()->set_parameter_set_as_.reset(new SetParameterSetActionServer(nh, "params/set_parameter_set", boost::bind(&ParameterManager::setParameterSetAction, Instance().get(), _1), false));
  Instance()->get_parameter_set_as_.reset(new GetParameterSetActionServer(nh, "params/get_parameter_set", boost::bind(&ParameterManager::getParameterSetAction, Instance().get(), _1), false));
  Instance()->get_all_parameter_sets_as_.reset(new GetAllParameterSetsActionServer(nh, "params/get_all_parameter_sets", boost::bind(&ParameterManager::getAllParameterSetsAction, Instance().get(), _1), false));
  Instance()->get_parameter_set_names_as_.reset(new GetParameterSetNamesActionServer(nh, "params/get_parameter_set_names", boost::bind(&ParameterManager::getParameterSetNamesAction, Instance().get(), _1), false));

  // start action servers
  Instance()->set_parameter_set_as_->start();
  Instance()->get_parameter_set_as_->start();
  Instance()->get_all_parameter_sets_as_->start();
  Instance()->get_parameter_set_names_as_->start();
}

ParameterManager::Ptr ParameterManager::Instance()
{
 if (!singelton_)
    singelton_.reset(new ParameterManager());
 return singelton_;
}

void ParameterManager::clear()
{
  Instance()->param_sets_.clear();
  Instance()->active_parameter_set_.reset(new ParameterSet());
}

bool ParameterManager::empty()
{
  return Instance()->param_sets_.empty();
}

size_t ParameterManager::size()
{
  return Instance()->param_sets_.size();
}

bool ParameterManager::loadFromFile(const boost::filesystem::path& path, ParameterSet& params)
{
  // load yaml file into rosparam server
  std::string name_space = ros::this_node::getNamespace() + "/" + path.filename().c_str();
  name_space.resize(name_space.size()-5);
  std::replace(name_space.begin(), name_space.end(), '.', '_');

  std::string cmd = "rosparam load " + std::string(path.c_str()) + " " + std::string(name_space.c_str());
  if (system(cmd.c_str()))
    return false;

  // get parameter as XmlRpcValue
  ros::NodeHandle nh(name_space);
  XmlRpc::XmlRpcValue val;
  nh.getParam("/", val);

  // cleanup
  cmd = "rosparam delete " + std::string(name_space.c_str());
  if (system(cmd.c_str()))
    return false;

  // parse XmlRpcValue
  params.clear();
  return params.fromXmlRpcValue(val);
}

void ParameterManager::loadParameterSets(const std::string& path)
{
  if (!boost::filesystem::exists(path))
  {
    ROS_ERROR("[loadParameterSets] Path not found: '%s'", path.c_str());
    return;
  }

  if (!boost::filesystem::is_directory(path))
  {
    ROS_ERROR("[loadParameterSets] '%s' is not a directory!", path.c_str());
    return;
  }

  // cycle through the directory
  for(boost::filesystem::directory_iterator itr(path); itr != boost::filesystem::directory_iterator(); itr++)
  {
    if (boost::filesystem::is_regular_file(*itr) and itr->path().extension() == ".yaml")
    {
      ParameterSet params;
      if (Instance()->loadFromFile(itr->path(), params))
      {
        if (!hasParameterSet(params.getName()))
        {
          updateParameterSet(params);
          ROS_INFO("[loadParameterSets] Loaded '%s' from '%s' with %u parameters", params.getName().c_str(), itr->path().filename().c_str(), params.size());
        }
        else
          ROS_WARN("[loadParameterSets] Set named '%s' already exist. Parameters from '%s' with %u parameters can't be added.", params.getName().c_str(), itr->path().filename().c_str(), params.size());
      }
      else
        ROS_ERROR("[loadParameterSets] Couldn't load parameters from '%s'", itr->path().filename().c_str());
    }
  }

  if (Instance()->param_sets_.empty())
    ROS_ERROR("Couldn't load any parameters!");
}

void ParameterManager::updateParameterSet(const ParameterSet& params)
{
  Instance()->param_sets_[params.getName()] = params;
  ROS_INFO("Updated parameter set '%s'.", params.getName().c_str());
}

void ParameterManager::updateParameterSet(const ParameterSetMsg& params)
{
  Instance()->param_sets_[params.name.data].fromMsg(params);
  ROS_INFO("Updated parameter set '%s'.", params.name.data.c_str());
}

bool ParameterManager::getParameterSet(const std::string& name, ParameterSet& params)
{
  std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets_.find(name);
  if (itr == Instance()->param_sets_.end())
    return false;

  params = itr->second;
  return true;
}

bool ParameterManager::getParameterSet(const std::string& name, ParameterSetMsg& params)
{
  std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets_.find(name);
  if (itr == Instance()->param_sets_.end())
    return false;

  itr->second.toMsg(params);
  return true;
}

void ParameterManager::getAllParameterSets(std::vector<ParameterSet>& param_sets)
{
  param_sets.clear();
  for (std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets_.begin(); itr != Instance()->param_sets_.end(); itr++)
    param_sets.push_back(itr->second);
}

void ParameterManager::getAllParameterSets(std::vector<ParameterSetMsg>& param_sets)
{
  param_sets.clear();
  ParameterSetMsg params;
  for (std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets_.begin(); itr != Instance()->param_sets_.end(); itr++)
  {
    itr->second.toMsg(params);
    param_sets.push_back(params);
  }
}

void ParameterManager::removeParameterSet(const std::string& name)
{
  Instance()->param_sets_.erase(name);

  if (Instance()->active_parameter_set_->getName() == name)
    Instance()->active_parameter_set_.reset(new ParameterSet());
}

bool ParameterManager::hasParameterSet(const std::string& name)
{
  return Instance()->param_sets_.find(name) != Instance()->param_sets_.end();
}

void ParameterManager::getParameterSetNames(std::vector<std::string>& names)
{
  names.clear();

  for (std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets_.begin(); itr != Instance()->param_sets_.end(); itr++)
    names.push_back(itr->first);
}

void ParameterManager::getParameterSetNames(std::vector<std_msgs::String>& names)
{
  names.clear();

  std_msgs::String name;
  for (std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets_.begin(); itr != Instance()->param_sets_.end(); itr++)
  {
    name.data = itr->first;
    names.push_back(name);
  }
}

bool ParameterManager::setActive(const std::string& name)
{
  if (getParameterSet(name, *Instance()->active_parameter_set_))
  {
    ROS_INFO("[ParameterManager] Set '%s' as active parameter set.", name.c_str());
  }
  else
  {
    ROS_ERROR("[ParameterManager] Can't set '%s' as active parameter set!", name.c_str());
    return false;
  }
  return true;
}

const ParameterSet& ParameterManager::getActive()
{
  return *(Instance()->active_parameter_set_);
}

// --- Subscriber calls ---

void ParameterManager::updateParameterSet(const ParameterSetMsgConstPtr& params)
{
  updateParameterSet(*params);
}

// --- Service calls ---

bool ParameterManager::setParameterSetService(SetParameterSetService::Request& req, SetParameterSetService::Response& resp)
{
  updateParameterSet(req.params);
  return true;
}

bool ParameterManager::getParameterSetService(GetParameterSetService::Request& req, GetParameterSetService::Response& resp)
{
  getParameterSet(req.name.data, resp.params);
  return true;
}

bool ParameterManager::getAllParameterSetsService(GetAllParameterSetsService::Request& req, GetAllParameterSetsService::Response& resp)
{
  getAllParameterSets(resp.param_sets);
  return true;
}

bool ParameterManager::getParameterSetNamesService(GetParameterSetNamesService::Request& req, GetParameterSetNamesService::Response& resp)
{
  getParameterSetNames(resp.names);
  return true;
}

// --- Action Server calls ---

void ParameterManager::setParameterSetAction(const SetParameterSetGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (set_parameter_set_as_->isPreemptRequested())
  {
    set_parameter_set_as_->setPreempted();
    return;
  }

  SetParameterSetResult result;
  updateParameterSet(goal->params);

  set_parameter_set_as_->setSucceeded(result);
}

void ParameterManager::getParameterSetAction(const GetParameterSetGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (get_parameter_set_as_->isPreemptRequested())
  {
    get_parameter_set_as_->setPreempted();
    return;
  }

  GetParameterSetResult result;
  if (getParameterSet(goal->name.data, result.params))
    get_parameter_set_as_->setSucceeded(result);
  else
    get_parameter_set_as_->setAborted(result, "[ParameterManager] getParameterSetAction: Couldn't get params named '" + goal->name.data + "'");
}

void ParameterManager::getAllParameterSetsAction(const GetAllParameterSetsGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (get_all_parameter_sets_as_->isPreemptRequested())
  {
    get_all_parameter_sets_as_->setPreempted();
    return;
  }

  GetAllParameterSetsResult result;
  getAllParameterSets(result.param_sets);

  get_all_parameter_sets_as_->setSucceeded(result);
}

void ParameterManager::getParameterSetNamesAction(const GetParameterSetNamesGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (get_parameter_set_names_as_->isPreemptRequested())
  {
    get_parameter_set_names_as_->setPreempted();
    return;
  }

  GetParameterSetNamesResult result;
  getParameterSetNames(result.names);

  get_parameter_set_names_as_->setSucceeded(result);
}
} // namespace
