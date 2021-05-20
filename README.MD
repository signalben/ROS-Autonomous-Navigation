__________________________________________________________________________________
# Turtlebot2 Autonomous Navigation Application 

The 'failsafe' package has been created as part of this work.  
All other packages/nodes use existing sources.
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

**To run autonomous navigation stack:**  
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
**To use the waypoint failsafe:**

With the navigation stack already open, in a new terminal:  
$ cd 10362555_autonomous_nav/autonomous_navigation_wss  
$ source source_all.bash  
$ rosrun failsafe failsafe

The failsafe will now monitor the robots progress towards active goals, and temporarily sets an easier goal if it has become stuck.  
This should have the effect of improving navigation reliabilty by allowing the local planner to escape false minima.  
Stopping and starting the failsafe during navigation is OK  
However:    
- the failsafe will be unaware of a goal set before it was started.    
- stopping the failsafe when a waypoint goal is active will not return the original goal.
__________________________________________________________________
**To use autonomous exploration:**
 
Close the waypoint failsafe if it is open.  
$ cd 10362555_autonomous_nav/autonomous_navigation_wss  
$ source source_all.bash  
$ roslaunch explore_lite explore.launch

Goals are automatically set at borders between free space and unknown space 
__________________________________________________________________________________  
## TESTING AND DEBUGGING  
Multiple launch file are used so that nodes experiencing an error may be restarted without restarting the whole stack.  
For reliable operation nodes should be launched in the order listed in the previous section.   
Therefore if a node is restarted later nodes depending on it should also be restarted in sequence. 

An easy method of testing the whole navigation stack is to set an impossible goal outside the maze, forcing the robot to plan paths covering the entire maze.
Performace was judged based on the frequency of undesirable behaviors, such as getting stuck in local minima, traversing one path section in a loop, or clipping fatal areas of the costmap.    
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

The parameters for this application used existing .yaml files as a starting point:  
https://github.com/husky/husky/tree/noetic-devel/husky_navigation/config  
https://github.com/turtlebot/turtlebot_apps/tree/indigo/turtlebot_navigation/param  

Further tuning was done by experiment, and use the following guides:   
http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide  
K. Zheng, “Ros navigation tuning guide,” 2017, arXiv:1706.09068.  
__________________________________________________________________

rosrun failsafe failsafe is located at autonomous_navigation_wss/navigation/src/failsafe   
No parameters or launch files are used, node is created by source file: autonomous_navigation_wss/navigation/src/failsafe/src/failsafe.cpp  
The activation of a short (waypoint) goal and the return to the long (original) goal is output in the console.
Substantial existing code was used to create a tf listener from:
http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
__________________________________________________________________

explore_lite explore.launch is located at autonomous_navigation_wss/exploration/src/explore/launch    
Parameters are set directly from the launch file.  
Source https://github.com/hrnr/m-explore/tree/kinetic-devel
__________________________________________________________________



