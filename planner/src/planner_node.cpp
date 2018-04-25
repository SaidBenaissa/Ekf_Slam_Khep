/*

 * planner_node.cpp
 *
 * 
*/


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Quaternion.h"
#include <iostream>


#define LINEAR_COEFF 0.227
#define ANGULAR_COEFF 0.483
#define dist 0.05
using namespace std;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/desired",1); //for control

  double R = 0.2;
  ros::Time current_time, init_time;
  init_time = ros::Time::now();

  double V = 0.1;//sqrt(vx*vx+vy*vy);
  double W =V/R;
  ros::Rate r(2.0);

  while(ros::ok())
  {

	current_time = ros::Time::now();

	double t = (current_time-init_time).toSec();

	double rho = (-M_PI/2)+(W*t);
	double theta = W*t;
	double x = R*sin(W*t);
	double y = -R*cos(W*t)+R ; //(x,y) = (0,R)


    nav_msgs::Odometry odom;



    odom.header.frame_id = "odom";
    odom.child_frame_id="base_link";
    odom.pose.pose.position.x = R*sin(W*t)+dist*cos(W*t);//for static
    odom.pose.pose.position.y = -R*cos(W*t)+R+dist*sin(W*t);
    odom.pose.pose.position.z = 0;

   // odom.pose.pose.orientation.z = theta;

   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.angular.z = W;
    odom.twist.twist.linear.x = V;
    odom.twist.twist.linear.y = R*cos(W*t)*W-dist*sin(W*t)*W; 
    odom.twist.twist.linear.z = R*sin(W*t)*W+dist*cos(W*t)*W ;
    odom.twist.twist.angular.x=0;
    odom.twist.twist.angular.y = 0;

    //publish the message
    odom_pub.publish(odom);

    ros::spinOnce(); // check for incoming messages
    r.sleep();
  }
}
