#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <stdlib.h>     
#include <time.h>
#include <math.h>        


enum RobotReaction { WANDERING, STOP, FORWARD, ROTATE_LEFT, ROTATE_RIGHT};
ros::Publisher myPublisher;


//Callback with the Laser Scan sensor information
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

//Send the robot a movement command
	//moveForwardVel: (> 0: moving forward)      (< 0: moving backwards)
	//rotateVel: (> 0: rotate left)      (< 0: rotate right)
void sendRobotMoveCmd(float moveForwardVel, float rotateVel);

//Main robot logic processing, called in the "scanCallback"
void robotLogic(float leftDistVal, float frontDistVal, float rightDistVal);

//Receives information from the distance at the left, front and right the robot and acts based on that information
RobotReaction getReaction(float leftDistVal, float frontDistVal, float rightDistVal);

//true if is under the established limit
bool isCloseToWall(float dist);

//true if is the distance is bigger than the established limt
bool isTooFarFromWall(float dist);

//true if it is the special case of the inside V
bool isCenterOfVcase(float centerDIst, float rightDIst);






int main(int argc, char **argv)
{
  ros::init(argc, argv, "mylistener");
  ros::NodeHandle n;

  srand (time(NULL));

  
  myPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);
  
  //Subscribing to node "scan"
  ros::Subscriber sub = n.subscribe("scan", 10, scanCallback);

  ros::spin();

  return 0;
}



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	robotLogic(msg->ranges[msg->ranges.size()-1], msg->ranges[msg->ranges.size()/2], msg->ranges[0]);
}




void robotLogic(float leftDistVal, float frontDistVal, float rightDistVal)
{
	switch (getReaction(leftDistVal, frontDistVal, rightDistVal))
	{
		case WANDERING:
		{	
			std::cout << "\nWANDERING\n";
			sendRobotMoveCmd(0.3, 0.2);			
			break;
		}
		case FORWARD:
		{	
			std::cout << "\nFORWARD\n";
			std::cout << "leftDist: " << leftDistVal << " frontDIst: " << frontDistVal << " rightDIst: " <<  rightDistVal << "\n";
			sendRobotMoveCmd(0.3, 0.0);
			break;
		}
		case ROTATE_LEFT:
		{	
			std::cout << "\nROTATE_LEFT\n";
			std::cout << "leftDist: " << leftDistVal << " frontDIst: " << frontDistVal << " rightDIst: " <<  rightDistVal << "\n";
			sendRobotMoveCmd(0, 0.4);
			break;
		}
		case ROTATE_RIGHT:
		{	
			std::cout << "\nROTATE_RIGHT\n";
			std::cout << "leftDist: " << leftDistVal << " frontDIst: " << frontDistVal << " rightDIst: " <<  rightDistVal << "\n";
			sendRobotMoveCmd(0, -0.2);
			break;
		}
		default:
			break;
	}
	
}





RobotReaction getReaction(float leftDistVal, float frontDistVal, float rightDistVal)
{	
	if(!isnan(leftDistVal) || !isnan(frontDistVal) || !isnan(rightDistVal)) //   xxx
	{
		if(!isnan(frontDistVal) && isnan(leftDistVal) && isnan(rightDistVal)) //   x|x
			return FORWARD;
		else //   \ ou /, can or can't have |
		{
			if(isCenterOfVcase(frontDistVal, rightDistVal))
				return ROTATE_RIGHT;

			//Is it only \ or /
			if(isnan(leftDistVal)) //    /, can or can't have |
			{
				if(isCloseToWall(rightDistVal))
					return ROTATE_LEFT;
				else if(isTooFarFromWall(rightDistVal))
					return ROTATE_RIGHT;

				return FORWARD;
			}
			else if(isnan(rightDistVal)) //    \, can or can't have |
			{
				if(isCloseToWall(leftDistVal))
					return ROTATE_RIGHT;
				else if(isTooFarFromWall(leftDistVal))
					return ROTATE_LEFT;
				
				return FORWARD;
			}
				
	
			
			//   \ and / have values
			if(leftDistVal == rightDistVal) //particular case, so rotate to a side a little, side doesn't matter
				return ROTATE_LEFT;
			else if(leftDistVal < rightDistVal)
			{
				if(isCloseToWall(leftDistVal))
					return ROTATE_RIGHT;

				return FORWARD;
			}
			else 
			{
				if(isCloseToWall(rightDistVal))
					return ROTATE_LEFT;
				
				return FORWARD;
			}
		}
	}



	return WANDERING;
}



void sendRobotMoveCmd(float moveForwardVel, float rotateVel)
{
	geometry_msgs::Twist twist;
        twist.linear.x = moveForwardVel;
     	twist.linear.y = 0.0;
     	twist.linear.z = 0.0;
     	twist.angular.x = 0.0;
     	twist.angular.y = 0.0;
     	twist.angular.z = rotateVel;

	std::cout << "sendRobotMoveCmd ForwardVel: " << moveForwardVel << "   RotateVel: " << rotateVel << "\n";
	
	
	//Publish msg
	myPublisher.publish(twist);
}




bool isCloseToWall(float dist)
{
	if(dist < 0.85)
		return true;
	else return false;
}



bool isTooFarFromWall(float dist)
{
	if(dist > 1.4)
		return true;
	else return false;
}


bool isCenterOfVcase(float centerDIst, float rightDIst)
{
	if(centerDIst < 1.4 || rightDIst < 1.4)
		return true;
	else return false;
}
