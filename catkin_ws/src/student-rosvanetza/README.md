# ros_vanetza

ETSI ITS-G5 communication node for ROS. The first recommendation is to install everything first in a normal PC, i.e., not the Turtlebot directly.

## Installation of all dependencies for RosVanetza (Ubuntu 18.04)
This tutorial provides the steps to follow to install successfully RosVanetza

## Prerequisite
Install ROS melodic (http://wiki.ros.org/melodic/Installation/Ubuntu)  
Repositories that should have cloned and where they should be located:  
- [denm_client](https://bitbucket.org/QuentinDelooz/denm_client/src/master/): in your catkin workspace  
- [Student-fake_data_rosvanetza](https://bitbucket.org/QuentinDelooz/student-fake_data_rosvanetza/src/master/): in your catkin workspace  
- [Student-ros_etsi_msgs](https://bitbucket.org/QuentinDelooz/student-ros_etsi_msg/src/master/): in your catkin workspace  
- [Student-vanetza](https://bitbucket.org/QuentinDelooz/student-vanetza/src/master/): not in your catkin, put it somewhere else for the moment (e.g.: ~/Programs)

The following steps will help you to install everything.

## Installing Vanetza
### Creation of repository & requirements
- *optional, depends where you want to put Vanetza* mkdir ~/Programs && cd ~/Programs  
- sudo apt-get update
- sudo apt-get install libboost-dev libcrypto++-dev libgeographic-dev libssl-dev


### Installation
1. cd ~/Programs
2. *If not already done* git clone git@bitbucket.org:QuentinDelooz/student-vanetza.git
3. cd student-vanetza
4. mkdir build && cd build
5. cmake .. -DBUILD_SHARED_LIBS=ON
6. make -j2


## *Optional* Creation of a catkin workspace
1. mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
2. cd .. && catkin_make
3. source ~/catkin_ws/devel/setup.bash (to run for every new console or add it into your ~/.bashrc file)


## Installation of RosVanetza
1. Clone all repositories (except the Vanetza one) into your catkin workspace.
2. cd your_catkin_workspace && catkin_make
2. source ~/catkin_ws/devel/setup.bash  

If you managed to arrive here, good job, you are almost done :)


## Start RosVanetza

### -- no need, you can skip -- Start Cohda 
1. ssh user@ip_cohda   
2. sudo llc rosvanetza  

### Configuration
Check the roslaunch file of the rosvanetza package.   
You should set the correct ip of the Cohda (ping the Cohda before to be sure you can connect with it).  

### Start rosvanetza
1. roslaunch fake_data_rosvanetza fake_gps.launch  
2. roslaunch vanetza basic.launch  

### What after?
You can try to take a look at the roslaunch files of the different packages and try to generate DENM (check the denm_sender package).  


# Problems
Feel free to contact me (Quentin Delooz)

