#!/usr/bin/env bash


# start raspberry pi connection
#gnome-terminal -- sshpass -p "ubuntu" ssh ubuntu@192.168.5.3 "roslaunch arti_chasi_mark3 arti_chasi_mark3_upstart_with_teleop.launch"
gnome-terminal -- ssh ubuntu@192.168.5.3 "ROS_IP=192.168.5.3 ROS_MASTER_URI=http://192.168.48.41:11311 roslaunch arti_chasi_mark3 arti_chasi_mark3_upstart_with_teleop.launch"
sleep 15


# start the rviz tool with loaded map and properties
gnome-terminal -- sh -c "ROS_MASTER_URI=http://192.168.48.41:11311 roslaunch arti_navigation move_base.launch"
sleep 5


# start the ROS action server in python using rospy
gnome-terminal -- sh -c "ROS_MASTER_URI=http://192.168.48.41:11311 rosrun rxt_skills_chasi chasi_action_server.py"
sleep 5






