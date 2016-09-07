//============================================================================
// Name        : line_extract_node.cpp
// Author      : NGUYEN Dong Hai Phuong
// Email       : ph17dn@gmail.com
// Version     :
// Copyright   : Your copyright notice
// Description : ROS node in C++, Ansi-style, to extract lines from laser data,
//              topic /laser_scan
//              find corners in the world and publish to the topic /extract_corners
//              as type PolygonStamped
//============================================================================

#include <iostream>
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PolygonStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_broadcaster.h"

#include <vector>
#include <math.h>
//#include "gnuplot_i.hpp"
#include "laserPoint.h"
#include "laserPoint.cpp"
#include "SegLasPts.h"
#include "SegLasPts.cpp"

//#include "gsl/gsl_fit.h"

using namespace std;

float MINIMUM_DISTANCE = 0.20;  //minimum distance to obstacle: 20cm
float minAngle;
double LINEAR_VEL = 0.3;        //move forward linear velocity: 0.3m/s
double ANGULAR_VEL = 1;         //turn right angular velocity: 1rad/s
ros::Time rotateTime;
int direction ;
const double p2pDistThres = 0.035;      //Maximum distance between 2 successive point
const double seg2pDistThres = 0.01;    //Maximum distance between a point of segment to the line segment (after fitting using linear regression)
const double cornerValid = 0.05;        //Maximum distance from a valid corner to lastPoint of previous segment and to firstPoint of next segment
const int numPtsSeg = 3;                //Maximum number of points in 1 segment
const int numPtsSeg_Re = 2;                //Maximum number of points in 1 segment in Re-Segmentation stage
vector<double> x,y;
//Gnuplot g1,g2;

vector<laserPoint::laserPoint> LaserPoints;     //Vector of Laser Points
vector<SegLasPts::SegLasPts> lineSegments;      //Vector of Segments
vector<SegLasPts::SegLasPts> filterLineSegments;      //Vector of Segments
vector<SegLasPts::SegLasPts> circleSegments;
vector<laserPoint::laserPoint> Corners;         //Vector of points at corner (intersection of 2 successive lines)
vector<double> cornersDist;                     //Vector of distance from extract Corner to robot
vector<double> cornersAngle;                     //Vector of angle between MC and Xm (M: center of robot, C: corner)
//laserPoint::laserPoint curLaserPoint;

visualization_msgs::Marker points, line_strip, line_list;
ros::Publisher corners_pub;
ros::Publisher marker_pub;

void marker_init(){

  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/base_laser";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.02;
  line_list.scale.x = 0.02;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is blue
  line_list.color.b = 1.0;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
}

void marker_publish(){
  ROS_INFO("Publish corners marker");
  points.points.clear();
  for (int i=0; i<Corners.size(); i++){
    geometry_msgs::Point p;
    p.x = Corners[i].getX();
    p.y = Corners[i].getY();
    p.z = 0;

    points.points.push_back(p);
  }

  ROS_INFO("Publish line segments marker");
  line_list.points.clear();
  for (int i=0; i<filterLineSegments.size(); i++){
    geometry_msgs::Point p1,p2;
    p1.x = filterLineSegments[i].getFirstPoint().getX();
    p1.y = filterLineSegments[i].getFirstPoint().getY();
    p1.z = 0;

    p2.x = filterLineSegments[i].getLastPoint().getX();
    p2.y = filterLineSegments[i].getLastPoint().getY();
    p2.z = 0;
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
  }
  marker_pub.publish(points);
  marker_pub.publish(line_list);
}

void publishCorners(vector <laserPoint::laserPoint> corners ){
  ROS_INFO("Publish corners coordinator");
  ros::Time current_time = ros::Time::now();
  geometry_msgs::PolygonStamped cornersStamped;
  cornersStamped.header.stamp = current_time;
  cornersStamped.header.frame_id = "/base_laser";
  cornersStamped.polygon.points.clear();
  cornersStamped.polygon.points.resize(corners.size());
  for (int i=0;i<corners.size();i++){
    cornersStamped.polygon.points[i].x = (float)corners[i].getX();
    cornersStamped.polygon.points[i].y = (float)corners[i].getY();
    cornersStamped.polygon.points[i].z = 0.0;
  }
  corners_pub.publish(cornersStamped);

}

void publishMeasure(vector <double> distances, vector<double> angles ){
  ROS_INFO("Publish corners distance and angle");
  ros::Time current_time = ros::Time::now();
  geometry_msgs::PolygonStamped cornersStamped;
  cornersStamped.header.stamp = current_time;
  cornersStamped.header.frame_id = "/base_laser";
  cornersStamped.polygon.points.clear();
  cornersStamped.polygon.points.resize(distances.size());
  for (int i=0;i<distances.size();i++){
    cornersStamped.polygon.points[i].x = (float)distances[i];   //This is NOT x coordinate
    cornersStamped.polygon.points[i].y = (float)angles[i];      //This is NOT y coordinate
    cornersStamped.polygon.points[i].z = 0.0;
  }
  corners_pub.publish(cornersStamped);

}

// Callback Function for Laser Message
void laser_Callback(sensor_msgs::LaserScan msg)
{
  ROS_INFO("Laser Callback");
  marker_init();

  unsigned nPoints = msg.ranges.size();
  double d,alpha;
  vector<double> ranges;
  vector<double> angle;
  ranges.resize(nPoints);
  angle.resize(nPoints);
  //int size_lineSegments = 1;

  x.resize(nPoints);
  y.resize(nPoints);
  LaserPoints.resize(nPoints);
  for (int i=0; i<nPoints; i++){
    d = msg.ranges[i];
    if (d>=msg.range_max){
      d = 0;
    }
    //alpha = msg.angle_increment*i-30/180*M_PI;
    //alpha = msg.angle_increment*i-120.0/180.0*M_PI;
    alpha = msg.angle_increment*i+(double)(msg.angle_min);//because x_m axis of robot lies at center of LRF which has maximum angle of (angle_max+angle_min), angle_min<0

    ranges[i] = d;
    angle[i] = alpha;

    x[i] = d*cos(alpha);
    y[i] = d*sin(alpha);
    LaserPoints[i] = laserPoint::laserPoint(x[i],y[i]);
  }
  //==============================================================================================

  // Todo:
  // Segmentation: divide all laser points into different clusters
  ROS_INFO("Segmentation");
  lineSegments.clear();
  lineSegments.resize(nPoints/3);
  lineSegments.push_back(SegLasPts::SegLasPts(0,0));
  SegLasPts::SegLasPts curSegment = lineSegments[0];


  int indexSeg=0;
  for (int k=0; k<nPoints-1;k++){
    // if distance of 2 successive points (k,k+1) <= threshold, set last index of the current segment as the index of point k+1
    double distance = LaserPoints[k].p2pDistance(LaserPoints[k+1]);
    if (distance <= p2pDistThres){
      curSegment.setLast(k+1);
    }
    // otherwise create new segment and set first and last index of new segment as the index of point k+1
    else {
      if (curSegment.getSize()>=numPtsSeg){
        indexSeg++;
      }
      curSegment.setFirst(k+1);
      curSegment.setLast(k+1);
    }
    if (curSegment.getSize()>=numPtsSeg){
      lineSegments[indexSeg] = curSegment;
    }
  }
  ROS_INFO("Number of segments %d: ",indexSeg+1);
  lineSegments.resize(indexSeg+1);

  //==============================================================================================

  // Todo:
  // Line fitting: using least square method to find parameters of line for each line Segments
  ROS_INFO("Line Fitting");

  double A,B;
  for (int i=0; i<lineSegments.size();i++){
    lineSegments[i].segmentFitting(x,y);
    lineSegments[i].compute(LaserPoints);
    //ROS_INFO("Info of segment %d: Length: %g, Size: %d, Coefficient: %g, %g ",i, lineSegments[i].getLength(), lineSegments[i].getSize(), lineSegments[i].getLineCoefA(), lineSegments[i].getLineCoefB());
    //gsl_fit_linear(posX,1,posY,1,sizeSegment,&c0,&c1,&cov00,&cov01,&cov11,&sumsq);
  }
  //==============================================================================================

  // Todo:
  // Re-Segmentation: Repeat until there is no new Segments
  // Find the maximum distance from points of Segment to line Segment: distance(Point(k),Segment)
  // This distance has to be smaller than a threshold
  // Otherwise  -Replace current Segment(i) by new Segment(firstPoint, Point(k))
  //            -Insert another new Segment(Point(k+1),lastPoint)(i+1)
  //            -Line Fitting for new Segments
  //            -hasNewSegment = 1;
  vector<SegLasPts::SegLasPts>::iterator it;
  it = lineSegments.begin();
  int hasNewSegment = 1;
  ROS_INFO("Re-Segmentation");

  int indexReSeg=0;
  int szLineSegments = lineSegments.size();
  while(indexReSeg<szLineSegments){
    int sizeSegment = lineSegments[indexReSeg].getSize();
    int startSegment = lineSegments[indexReSeg].getFirst();
    int endSegment = lineSegments[indexReSeg].getLast();
    double maxDistance = 0.0;
    int indexMaxDistance;
    int sizeNewSegment1,sizeNewSegment2;
    double posX[sizeSegment], posY[sizeSegment];
    vector<laserPoint::laserPoint> ptsCurSegment;
    ptsCurSegment.resize(sizeSegment);

    // Only do Re-Segmentation for segment longer than Threshold
    if (lineSegments[indexReSeg].getLength()>0.3){
      for (int j=0; j<sizeSegment; j++){

        ptsCurSegment[j].setPoint(x[startSegment+j],y[startSegment+j]);
        double curDistance = lineSegments[indexReSeg].segment2pDistance(ptsCurSegment[j]);
        if (curDistance > maxDistance){
          maxDistance = curDistance;
          indexMaxDistance = startSegment+j;
        }
      }
      //sizeNewSegment1 = (indexMaxDistance-1)-startSegment+1;
      sizeNewSegment1 = (indexMaxDistance)-startSegment+1;
      sizeNewSegment2 = endSegment-indexMaxDistance+1;
      //==============================================================================================
      // Todo:
      // If maxDistance >= threshold: have 1 new segment
      //          Increase szLineSegments 1
      //          Keep indexReSeg: to re-check new created Segments
      // Otherwise: have NO new segment
      //          Increase indexReSeg
      //          Keep szLineSegments
      //==============================================================================================
      if (maxDistance>=seg2pDistThres && sizeNewSegment1>= numPtsSeg_Re && sizeNewSegment2>=numPtsSeg_Re){
        //lineSegments[indexReSeg].setLast(indexMaxDistance-1);
        lineSegments[indexReSeg].setLast(indexMaxDistance);
        curSegment.setFirst(indexMaxDistance);
        curSegment.setLast(endSegment);
        lineSegments.insert(it+indexReSeg+1,curSegment);
        hasNewSegment = 1;
        szLineSegments++;
        // Todo: line-fitting for new segments
        lineSegments[indexReSeg].segmentFitting(x,y);
        lineSegments[indexReSeg].compute(LaserPoints);
        lineSegments[indexReSeg+1].segmentFitting(x,y);
        lineSegments[indexReSeg+1].compute(LaserPoints);
      }
      // If there are 1 new valid Segment: Segment1
      else if (maxDistance>=seg2pDistThres && sizeNewSegment1>=numPtsSeg_Re && sizeNewSegment2<numPtsSeg_Re){
        //lineSegments[indexReSeg].setLast(indexMaxDistance-1);
        lineSegments[indexReSeg].setLast(indexMaxDistance);
        lineSegments[indexReSeg].segmentFitting(x,y);
        lineSegments[indexReSeg].compute(LaserPoints);
        indexReSeg++;
      }
      // If there are 1 new valid Segment: Segment2
      else if (maxDistance>=seg2pDistThres && sizeNewSegment1<numPtsSeg_Re && sizeNewSegment2>=numPtsSeg_Re){
        lineSegments[indexReSeg].setFirst(indexMaxDistance);
        lineSegments[indexReSeg].segmentFitting(x,y);
        lineSegments[indexReSeg].compute(LaserPoints);
        indexReSeg++;
      }

      else {
        hasNewSegment = 0;
        indexReSeg++;
      }
    }
    else {
      indexReSeg++;
    }
  }
  //==============================================================================================

  // Todo:
  // Filter out line segments with length not longer than 0
  ROS_INFO("Filter Segments");
  filterLineSegments.resize(lineSegments.size());
  int indexFilter = 0;
  for (int i=0; i<lineSegments.size();i++){
    if (lineSegments[i].getLength()>0.0){
      filterLineSegments[indexFilter] = lineSegments[i];
      int first, last, size;
      double coef_a, coef_b;
      lineSegments[i].getParams(first, last, size, coef_a, coef_b);
      filterLineSegments[indexFilter].setParams(first,last,size,coef_a,coef_b);
      filterLineSegments[indexFilter].compute(LaserPoints);
      indexFilter++;
    }
  }
  filterLineSegments.resize(indexFilter);
  //==============================================================================================

  // Todo:
  // Find intersection of two successive line Segments (k,k+1) => Corner C(k)
  // Corner C(k) is available if and only if
  // distance(C(k),lastPoint(k))<=threshold) and // distance(C(k),firstPoint(k+1))<=threshold)
  ROS_INFO("Find Corners");
  int indexCorner;
  vector<double> xC,yC;
  Corners.clear();
  for (int i=0; i<filterLineSegments.size()-1; i++){
    double xCorner, yCorner;
    double a1,b1,a2,b2;
    double distance1, distance2;
    laserPoint::laserPoint curCorner, lastPoint1, firstPoint2;
    a1 = filterLineSegments[i].getLineCoefA();
    b1 = filterLineSegments[i].getLineCoefB();
    a2 = filterLineSegments[i+1].getLineCoefA();
    b2 = filterLineSegments[i+1].getLineCoefB();
    xCorner = (b2-b1)/(a1-a2);
    yCorner = (a1*b2-a2*b1)/(a1-a2);
    curCorner.setPoint(xCorner,yCorner);
    lastPoint1 = filterLineSegments[i].getLastPoint();
    firstPoint2 = filterLineSegments[i+1].getFirstPoint();
    distance1 = curCorner.p2pDistance(lastPoint1);
    distance2 = curCorner.p2pDistance(firstPoint2);
    if (distance1<=cornerValid && distance2<=cornerValid && abs(b2-b1)>=2*cornerValid){

      Corners.push_back(curCorner);
      xC.push_back(curCorner.getX());
      yC.push_back(curCorner.getY());
    }
  }
  ROS_INFO_STREAM("Numbers of Corners: "<<Corners.size());
  //==============================================================================================

  // Find the REAL MEASUREMENTS of the corners
  cornersDist.clear();
  cornersAngle.clear();
  for (int i=0; i<LaserPoints.size();i++){
    for (int j=0; j<Corners.size();j++){
      double distance = Corners[j].p2pDistance(LaserPoints[i]);
      if (distance<p2pDistThres){
        cornersDist.push_back(ranges[i]);
        cornersAngle.push_back(angle[i]);
      }
    }
  }
  for (int i=0; i<cornersDist.size();i++){
    ROS_INFO("Corners measurement: distance= %g, angle= %g",cornersDist[i],cornersAngle[i]);
  }
  //==============================================================================================

  // Publish message containing corner coordinators
  publishMeasure(cornersDist,cornersAngle);
  //==============================================================================================

  // Publish message containing corner coordinators
  //publishCorners(Corners);
  //==============================================================================================

  // Publish marker to visualize in RVIZ
  marker_publish();
  //==============================================================================================

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "line_extract");
  ROS_INFO("Node of line extraction");

  ros::NodeHandle nh_("~");
  //Subscribing
  ros::Subscriber laser_sub = nh_.subscribe <sensor_msgs::LaserScan> ("/base_scan",10,laser_Callback);
  //Publishing
  marker_pub = nh_.advertise<visualization_msgs::Marker>("/laser_marker", 10);
  corners_pub = nh_.advertise<geometry_msgs::PolygonStamped>("/extract_corners", 10);

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
/*    marker_pub.publish(points);
    marker_pub.publish(line_list);*/
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
