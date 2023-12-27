#!/bin/bash

roscore &
PID_ROSCORE=$!

sleep 1

roslaunch kortex_driver gen3_connection.launch &
PID1=$!
sleep 1

roslaunch feeding_task load_param.launch
PID4=$!

roslaunch kortex_motion_planning simple_jcmpe.launch &
PID2=$!
sleep 1

roslaunch feeding_task skewer_status_check.launch &
PID3=$!
sleep 1

wait $PID1  $PID2 $PID3 $PID4
 
kill $PID_ROSCORE


