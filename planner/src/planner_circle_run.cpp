#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Quaternion.h"
#include <iostream>

using namespace std;
using namespace ros;

#define LINEAR_COEFF 0.227
#define ANGULAR_COEFF 0.483

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for testing
  double R = 0.3;
    ros::Time current_time, init_time;
    init_time = ros::Time::now();

    double V = 0.1/LINEAR_COEFF;//sqrt(vx*vx+vy*vy);
    double W =(V*ANGULAR_COEFF)/(R);
    ros::Rate r(5.0);
  while(ros::ok())
  {

  	current_time = ros::Time::now();


  	double t = (current_time-init_time).toSec();

  		double rho = (-M_PI/2)+(W*t);
  		double theta = rho+(M_PI/2);
  		double x = R*cos(rho);
  		double y = R*sin(rho)+R; //(x,y) = (0,R)





	  //next, we'll publish the odometry message over ROS
	geometry_msgs::Twist odom;


	odom.linear.x = V;////vx;//
	odom.linear.y = 0;//vy;//0;
	odom.angular.z = W;

	odom_pub.publish(odom);
	ros::spinOnce();               // check for incoming messages
	r.sleep();

  }
}


