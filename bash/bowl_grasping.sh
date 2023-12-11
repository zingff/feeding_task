#!/bin/bash

# Start a ROS master
roscore &
PID_ROSCORE=$!

# Wait a bit to ensure ROS master has started
sleep 1

# Starting kortex_driver
roslaunch kortex_driver gen3_connection.launch &
PID1=$!
sleep 1

# Starting kortex_motion_planning
#roslaunch anygrasp_generation anygrasp_generation_and_publish.launch &
#PID2=$!
#sleep 1

roslaunch kortex_motion_planning kortex_motion_planning.launch &
PID3=$!
sleep 1

roslaunch kortex_motion_planning simple_jcmpe.launch &
PID4=$!
sleep 1

# Wait for all the processes to finish
wait $PID1 $PID2 $PID3 $PID4

# Kill the ROS master
kill $PID_ROSCORE


