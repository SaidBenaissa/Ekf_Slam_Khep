/*
 * laserPoint.h
 *  Class of Laser Points contains position (x,y) of every points detected by laser range finder
 *
 *  p2pDistance(laserPoint&); find the distance from current point to another point
 *  Created on: May 23, 2014
 *      Author: benaissa
 */

#ifndef LASERPOINT_H_
#define LASERPOINT_H_

namespace laserPoint
{

class laserPoint
{
public:
  /*laserPoint();*/
  laserPoint(double posX=double(0.0), double posY=double(0.0));
  void setX(double);
  void setY(double);
  void setPoint(double,double);
  double getX();
  double getY();
  double p2pDistance(laserPoint&);

  virtual ~laserPoint();
private:
  double x;
  double y;
};

} /* namespace laserPoint */

#endif /* LASERPOINT_H_ */
