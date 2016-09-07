//============================================================================
/* Control the robot from the keyboard.
Use arrow keys to steer the robot. Space to stop it.
Parameters scale_linear and scale_angular control the velocities sent to the robot.
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_SPACE 0x20




class Teleop {
public:
  Teleop();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher vel_pub_;
};


Teleop::Teleop():
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


void Teleop::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 2.0*M_PI/5.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -2.0*M_PI/5.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 0.3;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -0.3;
        dirty = true;
        break;
      case KEYCODE_SPACE:
        ROS_DEBUG("SPACE/STOP");
        linear_ = 0.0;
        angular_ = 0.0;
        dirty = true;
        break;
    }


    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular_;
    vel.linear.x = l_scale_*linear_*cos(a_scale_*angular_);
    vel.linear.y = l_scale_*linear_*sin(a_scale_*angular_);
    if( dirty )
    {
      vel_pub_.publish(vel);
      dirty = false;
    }
  }

  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  Teleop teleop;

  signal(SIGINT,quit);

  teleop.keyLoop();

  return(0);
}
