#!/bin/bash

roscore &
PID_ROSCORE=$!

sleep 1

roslaunch kortex_driver gen3_connection.launch &
PID1=$!
sleep 1

roslaunch feeding_task load_param.launch
PID8=$!

#roslaunch anygrasp_generation food_location_estimator.launch &
#PID2=$!
#sleep 1

roslaunch kortex_motion_planning simple_jcmpe.launch &
PID4=$!
sleep 1

roslaunch kortex_motion_planning collision_detection.launch &
PID5=$!
sleep 1

roslaunch feeding_task skewer_status_check.launch &
PID6=$!
sleep 1

wait $PID1 $PID8 $PID2 $PID3 $PID4 $PID5 $PID6 $PID7

kill $PID_ROSCORE


