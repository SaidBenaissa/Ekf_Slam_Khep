/**
 * A simple driver to interface with a player robot (stage or other).
 * This was written as an exercise to learn ROS. Adapted from stageros.cpp.
 *
 * It is conform to the requirements of the navigation stack:
 * http://www.ros.org/wiki/navigation/Tutorials/RobotSetup
 *  - it publishes information about the relationships between coordinate
 *    frames using tf. Frames used are odom, base_link and base_laser.
 *  - sensor_msgs/LaserScan messages are published
 *  - odometry information are published using tf and the nav_msgs/Odometry
 *    message.
 *  - it accepts geometry_msgs/Twist messages in the base coordinate frame of
 *    the robot on the "cmd_vel" topic.
 *
 * To test (use one terminal per command):
 * - start a player robot (i.e. in stage)
 * - start roscore
 * - start this program
 * - to send vel cmd: rostopic pub cmd_vel geometry_msgs/Twist '[0.2,0,0]' '[0,0,0]'
 *   where the six digits are the linear vel (x,y,z) and the angular vel (x,y,z),
 *   or run the teleop node.
 * - echo odometry msgs: rostopic echo odom
 *   or visualize with rviz
 */

#include <iostream>
#include <string>

#include <boost/signal.hpp>
#include <boost/bind.hpp>


// roscpp
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

// libstage
#include <libplayerc++/playerc++.h>
#include "playerclient.h"
//#include <playerc++.h>
#include <playererror.h>
#include <clientproxy.h>

#include <assert.h>

#include "opt.c"


using namespace PlayerCc;
using namespace std;

class PlayerRosNode {
  public:
    PlayerRosNode(std::string host);
    void setClientRead();

  //private:
    //  Player objects
    PlayerCc::PlayerClient *client;
    PlayerCc::Position2dProxy *posProx;
    PlayerCc::LaserProxy *lasProx;
    pthread_mutex_t mutex;
  private:
    // ROS topics
    ros::NodeHandle n;
    ros::Publisher laser_pub;
    ros::Publisher odom_pub;
    ros::Subscriber vel_sub;
    tf::TransformBroadcaster tfb;

    void odomReceived(void);
    void laserReceived(void);
    void cmdvelReceived(const boost::shared_ptr<geometry_msgs::Twist const>& msg);
};

/// Connects to Player on the specified host and create the bridge between
/// Player and ROS.
PlayerRosNode::PlayerRosNode(std::string host)
{
  // Connect to player and get the proxies for position and laser
  try {
    client = new PlayerCc::PlayerClient(host);
    posProx = new PlayerCc::Position2dProxy(client);
    posProx->SetMotorEnable(true);
    lasProx = new PlayerCc::LaserProxy(client);
    //client->SetDataMode(2);
    //client->SetReplaceRule(false,-1,-1,-1);
  }
  catch( PlayerCc::PlayerError &e) {
    std::cerr << e << std::endl;
    abort();
  }

//  int update;
//  double min, max, range_res, scan_res, scanning_frequency, count;
//  bool  intensity;
//
//  //cout = lasProx->GetCount();
//  min = -M_PI/2;
//  max = M_PI/2;
//  range_res = 0.001;
//  scan_res = 100;
//  scanning_frequency = 10;
//  intensity = "true";
//  lasProx->Configure(min,max,scan_res,range_res,intensity,scanning_frequency);




  // A mutex to protect concurrent access to posProx, because it is accessed in
  // 2 different threads (odomReceived and cmdvelReceived)
  pthread_mutex_init(&mutex,NULL);

  // Connect the handlers to treat player updates
  posProx->ConnectReadSignal(boost::bind(&PlayerRosNode::odomReceived,this));
  lasProx->ConnectReadSignal(boost::bind(&PlayerRosNode::laserReceived,this));

  // Advertize the ROS topics for odometry and laser data
  odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
  laser_pub = n.advertise<sensor_msgs::LaserScan>("/base_scan", 10);

  // Subscribe to the topic to receive velocity commands
  vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
                        boost::bind(&PlayerRosNode::cmdvelReceived,this,_1));

  // Start the player client
  client->StartThread();

}

void PlayerRosNode::setClientRead(){
  client->Read();
}

/// A callback function called by player when an odometry packet is received.
/// Read the packet and create the corresponding ROS message, then publish it.
void PlayerRosNode::odomReceived() {
  // safety net
  assert(posProx);

  // lock access to posProx
  pthread_mutex_lock( &mutex );

  // Retrieve odo info from Player
  float x = posProx->GetXPos();
  float y = posProx->GetYPos();
  float th = posProx->GetYaw(); //theta
  float vx = posProx->GetXSpeed();
  float vy = posProx->GetYSpeed();
  float vth = posProx->GetYawSpeed();

  // release access to posProx
  pthread_mutex_unlock( &mutex );

  // current time
  ros::Time current_time = ros::Time::now();

  //first, we'll publish the transform over tf
  //position and velocity of frame base_link w.r.t. frame odom (fixed frame)
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
  tfb.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;//0
  odom.twist.twist.angular.z = vth;
  odom_pub.publish(odom);

}


/// A callback function called by player when a laser packet is received.
/// Read the packet and create the corresponding ROS message, then publish it.
void PlayerRosNode::laserReceived() {
  //safety net
  assert(lasProx);

  // retrieve laser data
  unsigned nPoints = lasProx->GetCount();
  float resolution = lasProx->GetScanRes();
  float fov = (nPoints-1)*resolution;

  // current time
  ros::Time now = ros::Time::now();

  // first publish the transform over tf. In our case, the position of the laser
  // is fixed.
  tfb.sendTransform( tf::StampedTransform(
      tf::Transform( tf::createIdentityQuaternion(), //orientation of the laser on the robot
                     tf::Vector3(0.0, 0.0, 0.0) ),   //Position of the laser on the robot
      now, "base_link", "base_laser"));

  // then send the laser message
  sensor_msgs::LaserScan laserMsg;
  laserMsg.header.frame_id = "/base_laser";
  laserMsg.header.stamp = now;
  laserMsg.angle_min = -fov/2.0;
  laserMsg.angle_max = +fov/2.0;
  laserMsg.angle_increment = resolution;
  laserMsg.range_min = 0.002;
  //laserMsg.range_max = 20; //arbitrary value (can we get the real value from Player?)
  laserMsg.range_max = lasProx->GetMaxRange();
  laserMsg.ranges.resize(nPoints);
  laserMsg.intensities.resize(nPoints);
  //laserMsg.intensities.resize(intensity);
  for(unsigned int i=0;i<nPoints;i++) {
    laserMsg.ranges[i] = lasProx->GetRange(i);
    laserMsg.intensities[i] = (float)(lasProx->GetIntensity(i));

  }
  laser_pub.publish(laserMsg);
}


/// A callback function called by ROS when a velocity command message is received.
/// Read the message and send the command to player.
void PlayerRosNode::cmdvelReceived(const boost::shared_ptr<geometry_msgs::Twist const>& msg) {
  // safety net
  assert(posProx);

  // lock access to posProx
  pthread_mutex_lock( &mutex );

  // Pass the velocity command to Player
  //posProx->SetSpeed(msg->linear.x,msg->linear.y, msg->angular.z);
  posProx->SetSpeed(msg->linear.x,0.0, msg->angular.z);
  // release access to posProx
  pthread_mutex_unlock( &mutex );
}



int main(int argc, char **argv)
{
  opt_t *opt;
  int port;
  const char *host;
  double rate;

  // Init ROS
  ros::init(argc, argv, "playerros");

  // Load program options
  opt = opt_init(argc, argv, NULL);
  if (!opt)
  {
    //print_usage();
    return -1;
  }

  // Pick out some important program options
  host = opt_get_string(opt, "", "host", NULL);
  if (!host)
    host = opt_get_string(opt, "", "h", "localhost");

  port = opt_get_int(opt, "", "port", -1);
  if (port < 0)
    port = opt_get_int(opt, "", "p", 6665);

  rate = opt_get_double(opt, "", "rate", 10.0);
  if(rate < 0.0)
    rate = 0.0;

  // Connect to the server
  printf("Connecting to [%s:%d]\n", host, port);


  // Create the PlayerROS node
  //PlayerRosNode p("localhost");
  PlayerRosNode p(host);

  // Start the ROS message loop
  ros::spin();

  return 0;

/*  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    //p.posProx->ReadSignal();
    //p.setClientRead();
    //p.client->ReadIfWaiting();
    //p.client->Read();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;*/
}
