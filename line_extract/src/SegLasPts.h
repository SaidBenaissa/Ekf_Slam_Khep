/*
 * SegLasPts.h
 *  Class of Segment of Laser Points
 *  Created on: May 23, 2014
 *      Author: benaissa
 */

#ifndef SEGLASPTS_H_
#define SEGLASPTS_H_

#include "laserPoint.h"

namespace SegLasPts
{

class SegLasPts
{
public:
  //SegLasPts();
  SegLasPts(int start = 0,int end = 0);
  /*SegLasPts(int start = 0,int end = 0, int, laserPoint::laserPoint, laserPoint::laserPoint, laserPoint::laserPoint, double, double double,
            vector<double>, vector<double>);*/
  void setFirst(int);   //set index of first point in vector of laser Points
  void setLast(int);    //set index of last point in vector of laser Points

  int getFirst();
  int getLast();
  int getSize();

  void setFirstPoint(laserPoint::laserPoint);
  void setLastPoint(laserPoint::laserPoint);

  laserPoint::laserPoint getFirstPoint();
  laserPoint::laserPoint getLastPoint();

  void compute(vector<laserPoint::laserPoint>);       //calculate firstPoint and lastPoint of segment base on index First and Last and vector of laser Points
  double getLength();
  vector<double> getX();
  vector<double> getY();

  void setLineCoefA(double);
  void setLineCoefB(double);

  double getLineCoefA();
  double getLineCoefB();
  void getParams(int& ,int& ,int&, double&, double&) const;
  void setParams(int, int, int, double, double);

  double segment2pDistance(laserPoint::laserPoint&);
  void segmentFitting(vector<double>, vector<double>);

  /*const SegLasPts& operator=(const SegLasPts& otherSeg);        //Overload assignment operator*/

  virtual ~SegLasPts();
private:
  int First, Last, Size;
  laserPoint::laserPoint firstPoint, lastPoint, midPoint;
  double Length;
  double coefA, coefB;
  vector<double> X,Y;   //vector contains the coordinate of segment from firstPoint to lastPoint
};

} /* namespace SegLasPts */

#endif /* SEGLASPTS_H_ */
