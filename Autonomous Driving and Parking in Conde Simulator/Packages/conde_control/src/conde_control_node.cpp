#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <string>

#define FREQUENCY 20


double vlinear = 0;
double w = 0;
double VLINEARMAX = 1;
double WMAX = 0.012;
ros::Publisher vel_pub;
//ros::NodeHandle *vel_node;
geometry_msgs::Twist velocityToSend;

/// Callback References
void sensorCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{	
    vlinear = array->data[0]; //velocity to set
    w = array->data[1];       //angle to rotate
    std::cout << "[Sending to Motors] Velocity: " << vlinear << " Angle: " << w << std::endl;
    

    velocityToSend.angular.x = 0;
    velocityToSend.angular.y = 0;
    velocityToSend.angular.z = w;

    velocityToSend.linear.x = vlinear;
    velocityToSend.linear.y = 0;
    velocityToSend.linear.z = 0;
    ros::Rate loop_rate(1000/FREQUENCY);
    vel_pub.publish(velocityToSend);
		ros::spinOnce();
		loop_rate.sleep();
}



int main(int argc, char **argv) {
    std::cout << "\nStarting conde control\n" << std::endl;


    ros::init(argc,argv,"conde_control");
    
    ros::NodeHandle vel_node;

    std::string ref_topic = std::string("/conde_msg");
    ros::Subscriber sub1 = vel_node.subscribe(ref_topic, 50, sensorCallback);

    //vel_node = new ros::NodeHandle("~");
    vel_pub = vel_node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();

    return 0;

}
