//============================================================================
// Name        : kalman_localize_node.cpp
// Author      : NGUYEN Dong Hai Phuong
// Email       : ph17dn@gmail.com
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "visualization_msgs/Marker.h"


//#include "Eigen/Dense"

#include "mobroEKF.h"
#include "mobroEKF.cpp"


#include <libxml/parser.h>
#include <libxml/tree.h>

#include "laserPoint.h"
#include "laserPoint.cpp"

//#define

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using Eigen::Vector2d;


// Global Variables
double sigmaX ;
double sigmaY ;
double sigmaTheta ;

double sigmaDecoder;
double sigmaMeasureDist;
double sigmaMeasureAngle;
double MahaDistThres;
double Period;
double L;       //Distance between 2 wheels
double PoX,PoY,PoTheta;
double X0,Y0,Theta0;
double prevX,prevY,prevTheta;
double X, Y,  Theta ;
double covX,covY,covTheta;
const double LINEAR_COEF = 0.227;
const double ANGLE_COEF = 0.483;

MatrixXd Qa;    //System noise covariance matrix
MatrixXd Qb;    //Input noise covariance matrix
MatrixXd Qg;    //Measurement noise covariance matrix
MatrixXd P;     //Covariance matrix

double linearVel,angleVel;

//Global variables
string fileNameFilteredPosition;
string fileNameInitPosition;
bool saveDataToFile;
double covLinSpeed;
double covHeading;
double covPitch;
MatrixXd Qk;//System noise covariance matrix

double covMeasurement;

mobroEKF::mobroEKF kh3Kalman;

vector<laserPoint::laserPoint> mapCorners;         //Vector of points at corners in Map
vector<laserPoint::laserPoint> mapTransformedCorners;         //Vector of points at corners in Map
vector<laserPoint::laserPoint> extractCorners;         //Vector of points at extracted corners from laser Data(intersection of 2 successive lines)
vector<double> cornersDist;                     //Vector of distance from extract Corner to robot
vector<double> cornersAngle;                     //Vector of angle between MC and Xm (M: center of robot, C: corner)

ros::Publisher odom_predict_pub, odom_estimate_pub ,pose_pub;
nav_msgs::Odometry odom_msg;
geometry_msgs::Twist vel_msg;


//Function declarations
double getDoubleXMLValue(xmlNodePtr node, xmlChar *prop);
string getStringXMLValue(xmlNodePtr node, xmlChar *prop);
void updateReceiverDataXML(string fileName);
bool loadConfigFromXML(string fileName);

bool loadConfigFromXML(string fileName)//Load configuration from an XML file (data/config.xml)
{
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL;

    cout << "Loading config file:" + fileName << endl;
    doc = xmlReadFile(fileName.c_str(), NULL, 0);

    if (doc!=NULL){
      root_element = xmlDocGetRootElement(doc);

      if (root_element->type == XML_ELEMENT_NODE) {
        if (!xmlStrcmp(root_element->name, xmlCharStrdup("config")))
        {
           xmlNodePtr cfgNode;

           cfgNode=root_element->xmlChildrenNode;

           while(cfgNode!=NULL)
           {
              xmlChar *prop;

              if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("DistanceWheel")))
              {
                 L=getDoubleXMLValue(cfgNode, xmlCharStrdup("value"));
              }
              else if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("Period")))
              {
                 Period=getDoubleXMLValue(cfgNode, xmlCharStrdup("value"));
              }
              else if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("systemCovariance")))
              {
                sigmaX=getDoubleXMLValue(cfgNode, xmlCharStrdup("sigmaX"));
                sigmaY=getDoubleXMLValue(cfgNode, xmlCharStrdup("sigmaY"));
                sigmaTheta=getDoubleXMLValue(cfgNode, xmlCharStrdup("sigmaTheta"))/180.0*M_PI;
                //cout << "P INIT:" << endl << P << endl;
              }
              else if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("inputCovariance")))
              {
                 sigmaDecoder=getDoubleXMLValue(cfgNode, xmlCharStrdup("sigmaDecoder"));
                 //cout << "P INIT:" << endl << P << endl;
              }
              else if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("measureCovariance")))
              {
                sigmaMeasureDist=getDoubleXMLValue(cfgNode, xmlCharStrdup("sigmaDist"));
                sigmaMeasureAngle=getDoubleXMLValue(cfgNode, xmlCharStrdup("sigmaAngle"))/180.0*M_PI;

                //cout << "P INIT:" << endl << P << endl;
              }
              else if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("MahalanobisDistance")))
              {
                MahaDistThres=getDoubleXMLValue(cfgNode, xmlCharStrdup("threshold"));
                //cout << "P INIT:" << endl << P << endl;
              }
              else if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("InitP")))
              {
                PoX=getDoubleXMLValue(cfgNode, xmlCharStrdup("PoX"));
                PoY=getDoubleXMLValue(cfgNode, xmlCharStrdup("PoY"));
                PoTheta=getDoubleXMLValue(cfgNode, xmlCharStrdup("PoTheta"))/180.0*M_PI;
                //cout << "P INIT:" << endl << P << endl;
              }
              else if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("InitPosition")))
              {
                X0=getDoubleXMLValue(cfgNode, xmlCharStrdup("X0"));
                Y0=getDoubleXMLValue(cfgNode, xmlCharStrdup("Y0"));
                Theta0=getDoubleXMLValue(cfgNode, xmlCharStrdup("Theta0"))/180.0*M_PI;
                //cout << "P INIT:" << endl << P << endl;
              }
              cfgNode=cfgNode->next;
           }
        }
      }

      xmlFreeDoc(doc);

      xmlCleanupParser();
      return true;
    }
    else
    {
      return false;
    }
}

bool loadMapFromXML(string fileName)//Load configuration from an XML file (data/config.xml)
{
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL;
    string cornerName;
    double posX, posY;
    mapCorners.clear();

    cout << "Loading Map file:" + fileName << endl;
    doc = xmlReadFile(fileName.c_str(), NULL, 0);

    if (doc!=NULL){
      root_element = xmlDocGetRootElement(doc);

      if (root_element->type == XML_ELEMENT_NODE) {
        if (!xmlStrcmp(root_element->name, xmlCharStrdup("map")))
        {
           xmlNodePtr cfgNode;

           cfgNode=root_element->xmlChildrenNode;

           while(cfgNode!=NULL)
           {
              xmlChar *prop;


              if (!xmlStrcmp(cfgNode->name, xmlCharStrdup("corner")))
              {
                laserPoint::laserPoint curCorner;
                cornerName=getDoubleXMLValue(cfgNode, xmlCharStrdup("name"));
                posX=getDoubleXMLValue(cfgNode, xmlCharStrdup("valueX"));
                posY=getDoubleXMLValue(cfgNode, xmlCharStrdup("valueY"));
                curCorner.setPoint(posX,posY);
                mapCorners.push_back(curCorner);
                //mapCorners.push_back(laserPoint::laserPoint(posX,posY));
                //cout << "Input cov INIT:" << endl << Qu << endl;
              }

              cfgNode=cfgNode->next;
             }
          }
       }

       xmlFreeDoc(doc);

       xmlCleanupParser();
       return true;
    }
    else
    {
        return false;
    }
}

void updateReceiverDataXML(string fileName, double tstamp, double latitude, double longitude)
{
    std::ofstream resultOutput;

    cout << "Updating receiver position data XML file:" << fileName << endl;
    resultOutput.open(fileName.c_str(), std::ofstream::out | std::ofstream::trunc);

    if (resultOutput.fail())
    {
      std::cout<<"ERROR!! Unable to open resultData output file!"<<std::endl;
    }
    else
    {
      cout << "Time stamp:" << tstamp << " Latitude:[deg]:" << latitude << " Longitude:[deg]:" << longitude << endl;

      resultOutput << std::setprecision(10);

      resultOutput << "<?xml version=\"1.0\"?>" << endl;
      resultOutput << "<state>" << endl;
      resultOutput << "  <tstamp>" << tstamp << "</tstamp>" << endl;
      resultOutput << "  <latitude>" << latitude << "</latitude>" << endl;
      resultOutput << "  <longitude>" << longitude << "</longitude>" << endl;
      resultOutput << "</state>" << endl;

      resultOutput.close();

    }
}

double getDoubleXMLValue(xmlNodePtr node, xmlChar *prop)
{
   std::istringstream stm;
   xmlChar *propVal;
   double ret;

   propVal = xmlGetProp(node, prop);
   stm.str(reinterpret_cast<char *>(propVal));
   stm >> ret;
   xmlFree(propVal);
   return ret;
}

string getStringXMLValue(xmlNodePtr node, xmlChar *prop)
{
   std::istringstream stm;
   xmlChar *propVal;
   string ret;

   propVal = xmlGetProp(node, prop);
   stm.str(reinterpret_cast<char *>(propVal));
   stm >> ret;
   xmlFree(propVal);
   return ret;
}

void publishOdom(double x, double y, double theta, double v, double w, MatrixXd CovErr, ros::Publisher odom_pub){
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "world";//"odom";//        //msg.header.frame_id;
  odom.child_frame_id = "base_link";    //msg.child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = w;
  odom.pose.covariance[0] = CovErr(0,0);
  odom.pose.covariance[1] = CovErr(0,1);
  odom.pose.covariance[5] = CovErr(0,2);
  odom.pose.covariance[6] = CovErr(1,0);
  odom.pose.covariance[7] = CovErr(1,1);
  odom.pose.covariance[11]= CovErr(1,2);
  odom.pose.covariance[30]= CovErr(2,0);
  odom.pose.covariance[31]= CovErr(2,1);
  odom.pose.covariance[35]= CovErr(2,2);
  odom_pub.publish(odom);
}

void publishPose(double x, double y, double theta, ros::Publisher pose_pub){
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "world";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  pose_pub.publish(pose);
}

visualization_msgs::Marker points;
ros::Publisher marker_pub;

void marker_init(){

  points.header.frame_id ="world";
  points.header.stamp = ros::Time::now();
  points.ns = "map_corners";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;

  // Points are green
  points.color.b = 1.0f;
  points.color.a = 1.0;
}

void marker_publish(){
  //ROS_INFO("Publish corners marker");
  points.points.clear();
  for (int i=0; i<mapCorners.size(); i++){
    geometry_msgs::Point p;
    p.x = mapCorners[i].getX();
    p.y = mapCorners[i].getY();
    p.z = 0;
    points.points.push_back(p);
  }
  marker_pub.publish(points);
}

void setCovariance(MatrixXd covMatrix){
  covX = covMatrix(0,0);
  covY = covMatrix(1,1);
  covTheta = covMatrix(2,2);
}

ros::Time curTime,lastTime;


// Callback Function for Odometry Message
void odom_Callback(nav_msgs::Odometry msg){
  //ROS_INFO("Odom Callback");

  double v = 0.0, w = 0.0;
  double dt = 0.0;

  curTime = ros::Time::now();
  dt = (curTime-lastTime).toSec();
  v = LINEAR_COEF*msg.twist.twist.linear.x;        //edit 06-22-22:15
  w = ANGLE_COEF*msg.twist.twist.angular.z;       //edit 06-22-22:15

  kh3Kalman.setPeriod(dt);
  kh3Kalman.setSpeed(v,w);
  kh3Kalman.setMatrixA();
  kh3Kalman.setMatrixB();
  kh3Kalman.prediction();
  kh3Kalman.propagateError();
  P = kh3Kalman.getCovariance();
  kh3Kalman.getPose(X,Y,Theta);

  publishOdom(X,Y,Theta,v,w,P,odom_predict_pub);
  publishPose(X,Y,Theta,pose_pub);
  setCovariance(P);

  lastTime = curTime;

}

// Callback Function for Corners Message
void extract_corners_Callback(geometry_msgs::PolygonStamped msg){
  //ROS_INFO("Extract Corner Callback");
  cornersDist.resize(msg.polygon.points.size());
  cornersAngle.resize(msg.polygon.points.size());
  for (int i=0;i<msg.polygon.points.size();i++){

    cornersDist[i]=(double)msg.polygon.points[i].x;
    cornersAngle[i]=(double)msg.polygon.points[i].y;
  }

  //ROS_INFO_STREAM("Number of extracted Corners from Laser data "<< extractCorners.size());
  /*for (int i=0;i<extractCorners.size();i++){
    ROS_INFO("Coordinate of Corner %d: %g, %g",i,extractCorners[i].getX(),extractCorners[i].getY());
  }*/

  //if (kh3Kalman.hasMeasure(extractCorners,mapCorners)){
  if (kh3Kalman.hasMeasure(cornersDist,cornersAngle,mapCorners)){
    ROS_INFO("Good Corner ===========>>>>>>>>> Correct Pose ");
    double v,w;
    kh3Kalman.getSpeed(v,w);
    kh3Kalman.getPose(X,Y,Theta);
    P = kh3Kalman.getCovariance();

    // Publish the filtered odometry message over ROS
    publishOdom(X,Y,Theta,v,w,P,odom_estimate_pub);
    publishPose(X,Y,Theta,pose_pub);
    setCovariance(P);
  }


}

void cmd_vel_Callback(geometry_msgs::Twist msg){
  vel_msg = msg;
}

laserPoint::laserPoint transform(laserPoint::laserPoint corner,double X, double Y, double Theta){
  laserPoint::laserPoint newCorner;
  double xC = corner.getX();
  double yC = corner.getY();
  double x = (xC-X)*cos(Theta)+(yC-Y)*sin(Theta);
  double y = -(xC-X)*sin(Theta)+(yC-Y)*cos(Theta);
  newCorner.setPoint(x,y);
  return newCorner;
}

int main(int argc, char **argv)
{

  //Read execution arguments
  for (int i=1;i<argc;i++)
  {
    if (!strcmp(argv[i],"-m")){//Load parameters file in XML format
      string fileNameMap;

      fileNameMap.assign(argv[i+1]);
      loadMapFromXML(fileNameMap);
    }
    if (!strcmp(argv[i],"-f")){//Load parameters file in XML format
      string fileNameConfig;

      fileNameConfig.assign(argv[i+1]);
      loadConfigFromXML(fileNameConfig);
    }
  }
  mapTransformedCorners.resize(mapCorners.size());
  for (int i=0; i<mapCorners.size(); i++){
    mapTransformedCorners[i] = transform(mapCorners[i],X0,Y0,Theta);
  }

  ros::init(argc, argv, "Kalman_Localize");
  ROS_INFO("Kalman Localize Node");

  ROS_INFO_STREAM("Number of Corners in Map file "<< mapCorners.size());
  for (int i=0;i<mapCorners.size();i++){
    ROS_INFO("Coordinate of Corner %d: %g, %g",i,mapCorners[i].getX(),mapCorners[i].getY());
  }

  ROS_INFO("Coordinate of Corner respected to initial position of robot:");
  for (int i=0;i<mapTransformedCorners.size();i++){
    ROS_INFO("New Corner %d: %g, %g",i,mapTransformedCorners[i].getX(),mapTransformedCorners[i].getY());
  }

  ROS_INFO("Parameters of Kalman Filter:\n Sigma X= %g, Sigma Y= %g, Sigma Theta= %g\n"
      " Sigma Decoder= %g\n Sigma Measure Distance= %g, Sigma Measure Angle= %g\n Mahalanobis distance threshold= %g"
      ,sigmaX,sigmaY,sigmaTheta,sigmaDecoder,sigmaMeasureDist,sigmaMeasureAngle,MahaDistThres);
  ROS_INFO("Initial Pose: (%g,%g,%g)",X0,Y0,Theta0);
  ROS_INFO("Period: %g",Period);
  ros::NodeHandle nh_("~");

  kh3Kalman = mobroEKF::mobroEKF(Period,L,
                                 sigmaX, sigmaY, sigmaTheta,
                                 sigmaDecoder,
                                 sigmaMeasureDist, sigmaMeasureAngle,
                                 MahaDistThres);
  kh3Kalman.setInitP(PoX,PoY,PoTheta);
  kh3Kalman.setPose(X0,Y0,Theta0);      // Set Initial Pose of robot    //un-comment on 06-24-16:35
  kh3Kalman.matrixInit();


  X = X0;
  Y = Y0;
  Theta = Theta0;

  //kh3Kalman.prediction();             //edit 06-21-1:04

  curTime = ros::Time::now();
  lastTime = ros::Time::now();

  //Subscribing
  ros::Subscriber odom_sub = nh_.subscribe <nav_msgs::Odometry> ("/odom", 1, odom_Callback);
  ros::Subscriber corners_sub = nh_.subscribe <geometry_msgs::PolygonStamped> ("/extract_corners",10, extract_corners_Callback);
  ros::Subscriber vel_sub = nh_.subscribe <geometry_msgs::Twist> ("/cmd_vel",1, cmd_vel_Callback);

  //Publishing
  //ros::Publisher odom_pub = nh_.advertise <nav_msgs::Odometry> ("/odom_kalman", 10);
  odom_predict_pub = nh_.advertise <nav_msgs::Odometry> ("/odom_kalman_predict", 10);
  odom_estimate_pub = nh_.advertise <nav_msgs::Odometry> ("/odom_kalman_estimate", 10);
  pose_pub = nh_.advertise <geometry_msgs::PoseStamped> ("/pose_kalman",10);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("/map_corners_marker", 10);

  tf::TransformBroadcaster tfb;

  ros::Rate loop_rate(1.0/Period);

  ofstream output("/home/towardthesea/catkin_ws/result.txt");
  if (output!=NULL)
            output.clear();



  while(ros::ok())
  {
    output<< X<<","<< Y <<","<< Theta <<","<< covX <<","<< covY<<","<< covTheta<<endl;
    //marker_publish();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
