#include "feeding_utilities.hpp"

template <typename ParamType>
bool getROSParam(ros::NodeHandle nh, const std::string& param_name, ParamType& param_variable) {
  if (!nh.getParam(param_name, param_variable)) {
    ROS_ERROR("Couldn't retrieve param: %s.", param_name.c_str());
    return false;
  }
  return true;
}

template bool getROSParam(ros::NodeHandle nh, const std::string& param_name, int& param_variable);
template bool getROSParam(ros::NodeHandle nh, const std::string& param_name, double& param_variable);
template bool getROSParam(ros::NodeHandle nh, const std::string& param_name, std::string& param_variable);
template bool getROSParam(ros::NodeHandle nh, const std::string& param_name, std::vector<double>& param_variable);
