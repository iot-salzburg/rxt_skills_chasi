# rxt_skills_chasi
Implementation of ROBxTASK skills for ARTI Chasi robot as ROS Action Server

## Start and drive with the robot
Long press the left Button on the ARTI Robot to switch it on
Connect the computer to the ARTI Chasi Wi-Fi
Open a Terminal and connect to the ARTI Robot (Rpi4) via ssh ubuntu@192.168.5.3 - password: ubuntu
Start the ros nodes on the Robot with roslaunch arti_chasi_mark3 arti_chasi_mark3_upstart_with_teleop.launch
Open a second Terminal and start the OS1 lidar and Rviz with the launch file in this repository roslaunch arti_navigation arti_with_os1.launch
Switch on the Xbox 360 Controller
Drive - but be careful - the robot reacts very fast and has a high acceleration
Joystick Control
LB                     →  Deadman switch - has to be pressed all the time
Left Joystick    →  Drive
Right Joystick →  Steer


## Initial Setup - Creating a ROS Workspace
Install Ubuntu 16.04
Install ROS Kinetic (install ros-kinetic-desktop-full)
Install Gazebo 7.15+

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt upgrade

Create catkin workspace
Follow Instructions on https://github.com/ARTI-Robots/chasi
clone https://github.com/ARTI-Robots/ackermann-drive-teleop into catkin_workspace/src

sudo apt-get install ros-kinetic-controller-manager
sudo apt install ros-kinetic-gazebo-ros-control
sudo apt install ros-kinetic-effort-controllers
sudo apt install ros-kinetic-joint-state-controller

# (recommended) install the complete ros_control package including joint controllers (https://wiki.ros.org/ros_control)
sudo apt install ros-kinetic-ros-control ros-kinetic-ros-controllers
# (recommended) install moveit for motion planning (https://moveit.ros.org/install/)
sudo apt install ros-kinetic-moveit

catkin make inside catkin_workspace

##Simulation with a virtual Robot:
Connect Gamepad to PC via Cable
Start Rviz for visualization and Gazebo for simulation → roslaunch arti_chasi_gazebo gazebo_with_ouster_16.launch
Start the gamepad control node → roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch

##Network Configuration
Wifi-Configuration
see also: ARTI Herstellerinformation / Handbuch

SSID: "ARTI Chasi", Password: ARTIDefaultPW1!
Configuration: https://192.168.5.1/ (User & Password "ubnt")
IP-Configuration
EdgeMAX Router (DHCP Server): 192.168.5.1
Raspberry PI:    192.168.5.3
Ouster LIDAR:   192.168.5.4
Notebook:        192.168.5.5 


##Running the LIDAR (on the notebook):
Install
ROS Workspace should already be created on the notebook
Install the ouster-ros package into the workspace
cd ~/arti_ws/src
git clone https://github.com/ouster-lidar/ouster_example.git
export CMAKE_PREFIX_PATH=~/arti_ws/src/ouster_example
cd ~/arti_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
Run
Start the Ouster LIDAR via ROS → roslaunch ~/arti_ws/src/ouster_example/ouster_ros/os1.launch os1_hostname:=192.168.5.4 os1_udp_dest:=192.168.5.5 lidar_mode:=1024x20 viz:=false
For different configuration parameters see: Ouster GitHub
Visualize the Data with rviz → rviz -d ~/arti_ws/src/ouster_example/ouster_ros/viz.rviz

##Synchronize Time between Machines and LIDAR
This is necessary in order for the SLAM Algorithms to work properly.

Synchronisation between the ARTI Rpi and the Notebook
Install chrony on both machines
apt-get install chrony
Configure the NTP Server (Notebook)
sudo nano /etc/chrony/chrony.conf
Add the lines:
# make it serve time even if it is not synced (as it can't reach out)
local stratum 8
# allow the IP range of your peer to connect
allow 192.168.5
Configuire the NTP Client (ARTI Rpi)
sudo nano /etc/chrony/chrony.conf
Add the lines:
server 192.168.5.5 minpoll 0 maxpoll 5 maxdelay .05


Reboot both machines
Verify the synchronization on the client with:
chronyc tracking - should show the IP of the notebook and show its synchronized
Synchronization between the OS-Lidar and the Notebook
Important!
The USB-C Ethernet Card of the new notebook (no built in ethernet) is not capable of Hardware Timestamps. A possible solution would be to set the ARTI Rasberry PI as a time server.

However, we used a much quicker but dirty solution: Overwrite the timestamp of the ouster ros node with the system time of the notebook.

Steps:
Edit this file inside the ouster ros package: GitHub Link to Ouster ROS
Replace msg.header.stamp.fromNSec(timestamp.count()); with msg.header.stamp = ros::Time::now();
Rebuild the arti_ws ROS Workspace with catkin_make -DCMAKE_BUILD_TYPE=Release
Important!
The set_config_param timestamp_mode TIME_FROM_PTP_1588 is now automatically set via the arti_with_os1.launch file.

Furthermore there were massive Issues with the Gmapping package for creating a Map of the Environment. 

In arti_with_os1.launch, i added 3 static transform publishers for the os1_lidar.

It seems that the tf2 package for transforms in ROS has a bug (at least at ROS kinetic): The timestamp was 0 for each transformation and gmapping cannot handle that. 

That's also why the Lidar and Rpi have to be time synched to the notebook. In the arti os1 launch file are now 3 tf1 transform publishers with a polling rate of 100ms.

Connect to the OS1-Lidar via netcat

nc 192.168.5.4 7501

Get Sensor info and Timestamp

get_sensor_info

get_time_info 192.168.5.4 7501

Set the OS-1 Lidar Timesource to a PTP Master (IEEE 1588)

set_config_param timestamp_mode TIME_FROM_PTP_1588

Reinitialize the Lidar to let him change the settings

get_time_info 192.168.5.4 7501

reinitialize

Save the config

write_config_txt

Set up the PTP Master as told in [Ouster Software User Guide] Chapter 9
get_time_info 192.168.5.4 7501

Chapter 5 describes the PTP Setting of the OS1 



Map Making and Localization (SLAM) with Google Cartographer
Important!
Cartographer is not buildable now. First, the github repo states "build failed" for a long time. Second ROS Kinetic is not supported anymore by Cartographer, since its EOL.

I tried to create a own rosinstall file (see arti_ws/src/arti_navigation/scripts/install cartographer.sh) and replaced the "version master" with two older commits which did not fail building at the CI pipeline of cartographer, but that also doesnt work.

Only solution until now: Copy the cartographer_ws from the old notebook, delete the build_isolated and install_isolated folders and then run catkin_make_build --install --ninja only (see arti_ws/src/arti_navigation/scripts/install cartographer.sh).

A backup of the cartographer_ws is located at my backup drive. (and on the new and old ARTI Notebook)



GitHub:
https://github.com/nerovalerius/arti_navigation

##Prerequisites
Install google cartographer with rosrun arti_navigation install_cartographer.sh - this This installs as described "here"
Use rosrun arti_navigation configure_terminals.sh to add the necessary lines to bashrc which source the cartographer_ws and the arti_ws
Start the LIDAR with an visualization
Connect the computer to the ARTI Chasi Wi-Fi
Open a Terminal and connect to the ARTI Robot (Rpi4) via ssh ubuntu@192.168.5.3 - password: ubuntu
Start the ros nodes on the Robot with roslaunch arti_chasi_mark3 arti_chasi_mark3_upstart_with_teleop.launch
Open a second Terminal and start the OS1 lidar and Rviz with the launch file in this repository roslaunch arti_navigation arti_with_os1.launch
Switch on the Controller
Start the map making process with google cartographer
After the ros nodes on the robot and the os1_lidar with rviz is launched, start the cartograhper ROS node with roslaunch arti_navigation arti_cartographer.launch
Miscellaneous
Inside Rviz, the LIDAR data can be visualized with Add → By Topic → PointCloud2. Also the color scheme of the LIDAR data can be manipulated for easier interpretation.
