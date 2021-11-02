#!/usr/bin/env bash


# start the ROS action server in python using rospy
gnome-terminal -- rosrun rxt_skills_chasi chasi_action_server.py
sleep 5


# start raspberry pi connection
gnome-terminal -- sshpass -p "ubuntu" ssh ubuntu@192.168.5.3 "roslaunch arti_chasi_mark3 arti_chasi_mark3_upstart_with_teleop.launch"
sleep 50


# start the rviz tool with loaded map and properties
gnome-terminal -- roslaunch rxt_skills_chasi Chasi.launch






