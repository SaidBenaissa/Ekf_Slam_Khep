//============================================================================
// Name        : player_ros.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


using namespace std;


ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

float MINIMUM_DISTANCE = 0.20;  //minimum distance to obstacle: 20cm
float minAngle;
double LINEAR_VEL = 0.3;        //move forward linear velocity: 0.3m/s
double ANGULAR_VEL = 1;         //turn right angular velocity: 1rad/s
ros::Time rotateTime;
int direction ;

// Send a velocity command
void move(double linearVelMPS, double angularVelRadPS) {
  geometry_msgs::Twist msg; // The default constructor will set all commands to 0
  msg.linear.x = linearVelMPS;
  msg.angular.z = angularVelRadPS;
  cmd_vel_pub.publish(msg);
};

// Callback Function for Laser Message
void laser_Callback(sensor_msgs::LaserScan msg)
{
  float minDist = msg.range_max;
  int minIndex;
  ros::Duration rotateDura = ros::Duration(0.0);
  direction = 0;
  for (int i=0; i<msg.ranges.size();i++){
    if (msg.ranges[i]<minDist){
      minIndex = i;
      minDist = msg.ranges[i];
      minAngle = i*msg.angle_increment;
    }
  }
  //ROS_INFO_STREAM("Minimum Angle: "<< minAngle);
  //ROS_INFO_STREAM("Minimum Distance: "<< minDist);
  if (minDist <= MINIMUM_DISTANCE && (minAngle>=30/180*M_PI && minAngle<210/180*M_PI)){
  //if (minDist <= MINIMUM_DISTANCE){
    //ROS_INFO("Rotate left. Min Distance: %f\n",minDist);
    ros::Time curTime = ros::Time::now();
    //ros::Duration rotateDura = ros::Duration((minAngle-30/180*M_PI)/4/ANGULAR_VEL);
    //ros::Duration rotateDura = ros::Duration((5/180*M_PI)/ANGULAR_VEL);
    //rotateTime = curTime + rotateDura;
    if (minAngle <= (float)120/180*M_PI){        //If obstacles in right side, turn left
      direction = 10;
      rotateDura = ros::Duration((minAngle-(float)30/180*M_PI)/ANGULAR_VEL);
    }
    else if (minAngle > (float)120/180*M_PI){    //If obstacles in left side, turn right
      direction = 20;
      rotateDura = ros::Duration(((float)210/180*M_PI-minAngle)/ANGULAR_VEL);
    }
    else {
      direction = 0;
      rotateDura = ros::Duration(0.0);
    }
    rotateTime = curTime + rotateDura;
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "random_walk");
  ROS_INFO("Node random walk");

  ros::NodeHandle nh_("~");
  //Subscribing
  ros::Subscriber laser_sub = nh_.subscribe <sensor_msgs::LaserScan> ("/base_scan",10,laser_Callback);
  //Publishing
  cmd_vel_pub = nh_.advertise <geometry_msgs::Twist> ("/cmd_vel", 10);

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    ros::Time now = ros::Time::now();
    //ROS_INFO_STREAM("Now: "<< now.toNSec());
    //ROS_INFO_STREAM("Rotate Time: "<< rotateTime);

    if (now<=rotateTime){
      switch (direction){
      case 10:
        //ROS_INFO_STREAM("Rotate left. Min Angle: " <<minAngle<<endl);
        ROS_INFO("Rotate left. Min Angle: %f\n",minAngle);
        move(0,ANGULAR_VEL);
        break;
      case 20:
        //ROS_INFO_STREAM("Rotate right. Min Angle: " <<minAngle<<endl);
        ROS_INFO("Rotate right. Min Angle: %f\n",minAngle);
        move(0,-ANGULAR_VEL);
        break;
      }
      direction = 0;
    }
    else {
      move(LINEAR_VEL,0);
      ROS_INFO("Run forward. Min Angle: %f\n",minAngle);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
