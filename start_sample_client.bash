#/bin/bash

# SPAWN ROBOTS
# You can change followings in each launch file.
#  robot name
#  IP address of USARGazebo server
#  robot start point

roslaunch start_robot_r.launch &
sleep 2
roslaunch start_robot_b.launch &
sleep 2
roslaunch start_robot_g.launch &
sleep 2
roslaunch start_robot_y.launch &
sleep 1

# CONVERT ROBOT CAMERA IMAGES FROM USARGazebo jpeg image TO ROS TOPIC 
# You can change IP address of USARGazebo server in usarimage2ros.py .
# If you change usarimage2ros.py, you must recompile it.
#  $ python -m py_compile usarimage2ros.py
python usarimage2ros.pyc &
sleep 1

# VIEW ROBOT CAMERA IMAGES ON RVIZ
rviz -d usarsim_client_sample.rviz &

