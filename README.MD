__________________________________________________________________________________
# Turtlebot Autonomous Navigation Application 

The 'failsafe' package has been created as part of this work.
All other packages/nodes use existing sources, which are not own work. 
__________________________________________________________________________________
## PREREQUISITES  

Ubuntu          version: 16.04.7 LTS
ROS    	 version: kinetic
CoppeliaSim Edu version: 4.1.0  

Additionally an alias in bash.rc for CoppeliaSim is required:

export 
COPPELIASIM_ROOT_DIR=~/Programs/CoppeliaSim_Edu_V4_1_0_Ubuntu16_04
alias coppeliaSim="cd $COPPELIASIM_ROOT_DIR && ./coppeliaSim.sh"
source /opt/ros/kinetic/setup.bash
_________________________________________________________________________________
## INSTALLATION 
To install:

$ cd 10362555_autonomous_nav/autonomous_navigation_wss
$ cd turtlebot
$ catkin_make
$ cd ..
$ cd turtlebot_interaction
$ catkin_make
$ cd ..
$ cd turtlebot_simulation
$ catkin_make
$ cd ..
$ cd mapping
$ catkin_make
$ cd ..
$ cd navigation
$ catkin_make
$ cd ..
$ cd exploration
$ catkin_make

__________________________________________________________________________________
## RUNNING 

###To run autonomous navigation stack:
In a new terminal:
$ cd 10362555_autonomous_nav/autonomous_navigation_wss
$ source source_all.bash
$ roslaunch vrep_simulation init.launch 
 Copellia Scene, and rviz should open. Run Coppeliasim.

In a new terminal:
$ cd 10362555_autonomous_nav/autonomous_navigation_wss
$ source source_all.bash
$ roslaunch gmapping gmapping.launch  

In a new terminal:
$ cd 10362555_autonomous_nav/autonomous_navigation_wss
$ source source_all.bash
$ roslaunch roslaunch move_base stack.launch  

At this point it should be possible to command the robot by setting a nav goal in rviz. 
__________________________________________________________________
###To use the waypoint failsafe:

With the navigation stack already open, in a new terminal:
$ cd 10362555_autonomous_nav/autonomous_navigation_wss
$ source source_all.bash
$ rosrun failsafe failsafe

The failsafe will now monitor the robots progress towards active goals, and temporarily sets an easier goal if it has become stuck.
This should have the effect of improving navigation reliabilty by allowing the local planner to escape false minima. 
Stopping and starting the failsafe during navigation is OK
However- the failsafe will be unaware of a goal set before it was started.
       - stopping the failsafe when a waypoint goal is active will not return the original goal.
__________________________________________________________________
###To use autonomous exploration:
 
Close the waypoint failsafe if it is open.
$ cd 10362555_autonomous_nav/autonomous_navigation_wss
$ source source_all.bash
$ roslaunch explore_lite explore.launch

Goals are automatically set at borders between free space and unknown space 
__________________________________________________________________________________
## NOTES

vrep_simulation init.launch is located at autonomous_navigation_wss/turtlebot_simulation/src/vrep_simulation/launch

For convenience, it starts both Coppeliasim and Rviz using the two launch files below:
autonomous_navigation_wss/turtlebot_simulation/src/vrep_simulation/launch/vrep_simulation.launch
autonomous_navigation_wss/turtlebot_interaction/src/turtlebot_rviz_launchers/launch/turtlebot_rviz.launch
__________________________________________________________________

gmapping gmapping.launch is located at /home/ben/10362555_autonomous_nav/autonomous_navigation_wss/mapping/src/gmapping/launch
Parameters are set directly from the launch file.
Source: https://github.com/ros-perception/slam_gmapping
__________________________________________________________________

move_base stack.launch is located at autonomous_navigation_wss/navigation/src/move_base/launch
Parameters are loaded from yaml files in autonomous_navigation_wss/navigation/src/move_base/param
Source: https://github.com/ros-planning/navigation/tree/kinetic-devel
__________________________________________________________________

rosrun failsafe failsafe is located at autonomous_navigation_wss/navigation/src/failsafe
No parameters or launch files are used, node is created by source file: autonomous_navigation_wss/navigation/src/failsafe/src/failsafe.cpp
__________________________________________________________________

explore_lite explore.launch is located at autonomous_navigation_wss/exploration/src/explore/launch
Parameters are set directly from the launch file.
Source https://github.com/hrnr/m-explore/tree/kinetic-devel
__________________________________________________________________



