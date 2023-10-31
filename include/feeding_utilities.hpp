#ifndef FEEDING_UTILITIES_HPP
#define FEEDING_UTILITIES_HPP

#include <ros/ros.h>
#include <string>

template <typename ParamType>
bool getROSParam(ros::NodeHandle nh, const std::string& param_name, ParamType& param_variable);

#endif // FEEDING_UTILITIES_HPP