#!/usr/bin/env bash

gnome-terminal -- killall roslaunch

gnome-terminal -- killall rosrun

gnome-terminal -- ssh ubuntu@192.168.5.3 "killall roslaunch"

