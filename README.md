# rxt_skills_chasi
Implementation of ROBxTASK skills for ARTI Chasi robot as ROS Action Server

## Start and drive with the robot
- Long press the left Button on the ARTI Robot to switch it on
- Connect the computer to the ARTI Chasi Wi-Fi
- Open a Terminal and connect to the ARTI Robot (Rpi4) via ssh ubuntu@192.168.5.3 - password: ubuntu
- Start the ros nodes on the Robot with roslaunch arti_chasi_mark3 arti_chasi_mark3_upstart_with_teleop.launch
- Open a second Terminal and start the OS1 lidar and Rviz with the launch file in this repository roslaunch arti_navigation arti_with_os1.launch
- Switch on the Xbox 360 Controller
- Driv, but be careful, the robot reacts very fast and has a high acceleration
- Joystick Control
LB  →  Deadman switch - has to be pressed all the time
Left Joystick  →  Drive
Right Joystick →  Steer


## Initial Setup - Creating a ROS Workspace
- Install Ubuntu 16.04
- Install ROS Kinetic (install ros-kinetic-desktop-full)
- Install Gazebo 7.15+
- Create catkin workspace
- Follow Instructions on https://github.com/ARTI-Robots/chasi
- Clone https://github.com/ARTI-Robots/ackermann-drive-teleop into catkin_workspace/src

Next we need to get some packages:
- sudo apt-get install ros-kinetic-controller-manager
- sudo apt install ros-kinetic-gazebo-ros-control
- sudo apt install ros-kinetic-effort-controllers
- sudo apt install ros-kinetic-joint-state-controller
- (recommended) install the complete ros_control package including joint controllers (https://wiki.ros.org/ros_control)
- sudo apt install ros-kinetic-ros-control ros-kinetic-ros-controllers
- (recommended) install moveit for motion planning (https://moveit.ros.org/install/)
- sudo apt install ros-kinetic-moveit
- catkin make inside catkin_workspace


##Simulation with a virtual Robot:
- Connect Gamepad to PC via Cable
- Start Rviz for visualization and Gazebo for simulation → roslaunch arti_chasi_gazebo gazebo_with_ouster_16.launch
- Start the gamepad control node → roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch


##Network Configuration
For Wifi-Configuration see also: ARTI Herstellerinformation / Handbuch

- SSID: "ARTI Chasi", Password: ARTIDefaultPW1!
- Configuration: https://192.168.5.1/ (User & Password "ubnt")
- EdgeMAX Router IP (DHCP Server): 192.168.5.1
- Raspberry PI IP:    192.168.5.3
- Ouster LIDAR IP:   192.168.5.4
- Notebook IP:        192.168.5.5 


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

For the synchronisation between the ARTI Rpi and the Notebook:
- Install chrony on both machines
- apt-get install chrony
- Configure the NTP Server (Notebook)
- sudo nano /etc/chrony/chrony.conf
- Add line: "local stratum 8"
- Add line: "allow 192.168.5"
- Configuire the NTP Client (ARTI Rpi)
- sudo nano /etc/chrony/chrony.conf
- Add the lines: server 192.168.5.5 minpoll 0 maxpoll 5 maxdelay .05

Reboot both machines
Verify the synchronization on the client with: chronyc tracking (should show the IP of the notebook and show its synchronized).
Synchronization between the OS-Lidar and the Notebook. The USB-C Ethernet Card of the new notebook (no built in ethernet) is not capable of Hardware Timestamps. A possible solution would be to set the ARTI Rasberry PI as a time server. However, we used a much quicker but dirty solution: Overwrite the timestamp of the ouster ros node with the system time of the notebook. For sull docu of steps see: https://secure.salzburgresearch.at/wiki/pages/viewpage.action?pageId=63804455


##Map Making and Localization (SLAM) with Google Cartographer:
Important! Cartographer is not buildable now. First, the github repo states "build failed" for a long time. Second ROS Kinetic is not supported anymore by Cartographer, since its EOL. I tried to create a own rosinstall file (see arti_ws/src/arti_navigation/scripts/install cartographer.sh) and replaced the "version master" with two older commits which did not fail building at the CI pipeline of cartographer, but that also doesnt work.

Only solution until now: Copy the cartographer_ws from the old notebook, delete the build_isolated and install_isolated folders and then run catkin_make_build --install --ninja only (see arti_ws/src/arti_navigation/scripts/install cartographer.sh). A backup of the cartographer_ws is located at my backup drive. (and on the new and old ARTI Notebook)

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
