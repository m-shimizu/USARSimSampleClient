#/bin/bash

roslaunch start_robot_r.launch &
sleep 2
roslaunch start_robot_b.launch &
sleep 2
roslaunch start_robot_g.launch &
sleep 2
roslaunch start_robot_y.launch &
sleep 1
python usarimage2ros.pyc &
sleep 1
rviz -d usarsim_client_sample.rviz &

