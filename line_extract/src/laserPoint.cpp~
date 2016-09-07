/*
 * laserPoint.cpp
 *
 *  Created on: May 23, 2014
 *      Author: towardthesea
 */

#include "laserPoint.h"
#include <math.h>

using namespace std;

namespace laserPoint
{

/*laserPoint::laserPoint()
{
  // TODO Auto-generated constructor stub
  x = double(0.0);
  y = double(0.0);
}*/

laserPoint::laserPoint(double posX,double posY)
{
  // TODO Auto-generated constructor stub
  x = posX;
  y = posY;
}

void laserPoint::setX(double posX){
  x = posX;
}

void laserPoint::setY(double posY){
  y = posY;
}

void laserPoint::setPoint(double posX,double posY){
  x = posX;
  y = posY;
}

double laserPoint::getX(){
  return x;
}

double laserPoint::getY(){
  return y;
}

double laserPoint::p2pDistance(laserPoint& nextPoint){
  double dist = sqrt(pow(x-nextPoint.getX(),2) + pow(y-nextPoint.getY(),2));
  return dist;
}

laserPoint::~laserPoint()
{
  // TODO Auto-generated destructor stub
}

} /* namespace laserPoint */
