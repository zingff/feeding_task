import rospy

class StateOutputStyle:
    success = '\033[1;38;5;46m'  # green
    failure = '\033[1;31m'  # red
    default = '\033[0m'  # default
    
    grey = '\033[1;30m' 
    red = '\033[1;31m' 
    green = '\033[1;32m' 
    yellow = '\033[1;33m' 
    blue = '\033[1;34m' 
    purple = '\033[1;35m'
    sky_blue = '\033[1;36m'
    
    # 20231208 the azure color is inspired from 
    # 20231208 today's clear and azure sky
    # 20231208 what a nice day, yet a slave day.
    azure = '\033[1;38;5;81m'
    
    bg_red = '\033[1;41m'
    bg_green = '\033[1;42m'
    bg_yellow = '\033[1;43m'
    bg_blue = '\033[1;44m'
    bg_purple = '\033[1;45m'
    bg_sky_blue = '\033[1;46m'
    
    collision = '\033[1;38;5;123m'  # cyan
    stop_call = '\033[1;38;5;121m'
    striking  = '\033[1;38;5;190m'
    normal = '\033[38;5;152m'

    
def success_loginfo(msg):
    rospy.loginfo(StateOutputStyle.success + msg + StateOutputStyle.default)


def warn_loginfo(msg):
    rospy.loginfo(StateOutputStyle.yellow + msg + StateOutputStyle.default)


def failure_loginfo(msg):
    rospy.loginfo(StateOutputStyle.failure + msg + StateOutputStyle.default)


def get_ros_param(param_name, default_value = None):
    if not rospy.has_param(param_name) and default_value is not None:
        warn_loginfo("Couldn't retrieve param: " + str(param_name) + " in ROS parameter server, set it to default value, please check if a correct default value is given.")
        return default_value
    elif not rospy.has_param(param_name) and default_value is None:
        failure_loginfo("Couldn't retrieve param: " + str(param_name) + " in ROS parameter server nor assign the value with default value.")
        return None
    else:
        param_value = rospy.get_param(param_name, default_value)
        return param_value

