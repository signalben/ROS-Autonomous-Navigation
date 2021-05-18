#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <math.h>

//Failsafe node monitors the local planners progress, and detects if it has stalled. 
//If the robot has stalled, a shorter goal is set as a waypoint, and the original goal restored once the waypoint is reached.

int stuck = 0;  //Tracks length of time the robot has been stuck for
float x_vel = 0; //robot forward velocity
bool longActive = false, shortActive = false, shortReached = false; //flags for original and shorter waypoint goals activity
geometry_msgs::PoseStamped shortGoal, longGoal, currentGoal;	//stores long and short goals

//called when a goal is set
void getGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
	currentGoal.pose = msg -> pose;		//current goal position used to detect when it has been reached
	currentGoal.header = msg -> header;

	stuck = 0;		//reset stuck count
	if(!shortActive){	//store as longgoal if it is not a waypoint	
		 longGoal = currentGoal;
		 longActive = true;	
	}
}

//called when robot cmd_vel is published, used to detect stalling
void getVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{
	x_vel = msg->linear.x;
}

//uses global path to produce short goal
void getPath(const nav_msgs::Path::ConstPtr& msg)
{
	if((!shortActive)&&(longActive)){ //only produce once
		shortGoal = msg->poses[35]; //found to be a suitable distance along path	
	}
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "failsafe");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/move_base/current_goal", 100, getGoal); //current robot goal
  ros::Subscriber sub3 = n.subscribe("cmd_vel", 100, getVelocity); //robot forward velocity
  ros::Subscriber sub4 = n.subscribe("/move_base/DWAPlannerROS/global_plan", 100, getPath); //global path	  
  ros::Publisher  pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100); //set robot goal
  tf::TransformListener listener; //used to obtain robot pose
  tf::StampedTransform transform;
  ros::Rate rate(10); //loop at 10Hz
	

  while(ros::ok()){
	
       try{
          listener.lookupTransform( "map", "base_footprint", ros::Time::now()-ros::Duration(0.1), transform); //get robots position in map
        }
        catch (tf::TransformException ex){ //catch if transform not published yet
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
        }


	if((longActive)&&(x_vel == 0)){stuck++;} //if goal set, but robot not moving forwards, then it is stuck

	if(x_vel != 0){ //if moving, then the robot is unstuck
		stuck = 0;
	}

	if((stuck > 7)&&(!shortActive)){ //if it has been stuck for a while, and is not already using a short goal
		pub.publish(shortGoal); //then set the shortgoal made from the global path as the robots current goal
		shortActive = true;
		ROS_INFO("Using Short Goal");
	}
		
	float goalDist_x = transform.getOrigin().x() - currentGoal.pose.position.x; //compute robot distance to goal 
	float goalDist_y = transform.getOrigin().y() - currentGoal.pose.position.y; 
	float dist = sqrt((goalDist_x*goalDist_x)+(goalDist_y*goalDist_y));
	
	if(dist < 0.45){ //has a goal been reached? smaller values cause the robot to decelerate when reaching a shortgoal
		stuck = 0;

		if(shortActive){ 	//short goal reached, resume using stored longgoal
			pub.publish(longGoal);
		 	shortActive = false;
			longActive = true;
			ROS_INFO("Using Long Goal");
		} 

		else if(longActive){	//long goal reached, deactivate until new goal set 
			longActive = false;
		}  
	}
	ros::spinOnce();
	rate.sleep();
  }
  return 0;
}


