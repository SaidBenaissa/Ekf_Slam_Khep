/*
 * SegLasPts.cpp
 *
 *  Created on: May 23, 2014
 *      Author: benaissa
 */

#include "SegLasPts.h"
#include "linreg.h"
#include "linreg.cpp"

namespace SegLasPts
{

/*SegLasPts::SegLasPts()
{
  // TODO Auto-generated constructor stub
  First = 0;
  Last = 0;
  Size = 0;
}*/

SegLasPts::SegLasPts(int start,int end){
  First = start;
  Last = end;
  Size = end-start;
}

void SegLasPts::setFirst(int start){
  First = start;
}

void SegLasPts::setLast(int end){
  Last = end;
}

int SegLasPts::getFirst(){
  return First;
}

int SegLasPts::getLast(){
  return Last;
}
int SegLasPts::getSize(){
  Size = Last - First + 1;
  return Size;
}

void SegLasPts::setFirstPoint(laserPoint::laserPoint startPoint){
  firstPoint = startPoint;
}

void SegLasPts::setLastPoint(laserPoint::laserPoint endPoint){
  lastPoint = endPoint;
}


laserPoint::laserPoint SegLasPts::getFirstPoint(){
  return firstPoint;
}

laserPoint::laserPoint SegLasPts::getLastPoint(){
  return lastPoint;
}

// Calculate the coordinator of projection of point P on the line y=Ax+B
laserPoint::laserPoint projectPts2Line(laserPoint::laserPoint P,double A, double B){
  laserPoint::laserPoint projectionP;
  double X,Y;
  double xP = P.getX();
  double yP = P.getY();

  X = (A*yP + xP - A*B)/(A*A+1);
  Y = (A*A*yP + A*xP + B)/(A*A+1);

  projectionP.setX(X);
  projectionP.setY(Y);

  return projectionP;
}

double SegLasPts::segment2pDistance(laserPoint::laserPoint& pointP){

  laserPoint::laserPoint projP = projectPts2Line(pointP, coefA, coefB);
  double distance = pointP.p2pDistance(projP);
  return distance;
}

// Calculate firstPoint and lastPoint of segment base on index First and Last and vector of laser Points
void SegLasPts::compute(vector<laserPoint::laserPoint> laserPoints){
  laserPoint::laserPoint startPoint = laserPoints[First];
  laserPoint::laserPoint endPoint = laserPoints[Last];
  firstPoint = projectPts2Line(startPoint, coefA, coefB);
  lastPoint = projectPts2Line(endPoint, coefA, coefB);
  Length = firstPoint.p2pDistance(lastPoint);
  X.resize(3);
  Y.resize(3);
  X[0]=firstPoint.getX();
  X[2]=lastPoint.getX();
  Y[0]=firstPoint.getY();
  Y[2]=lastPoint.getY();

  midPoint.setX((firstPoint.getX()+lastPoint.getX())/2);
  midPoint.setY((firstPoint.getY()+lastPoint.getY())/2);
  X[1]=midPoint.getX();
  Y[1]=midPoint.getY();
}

void SegLasPts::segmentFitting(vector<double> x, vector<double> y){

  double posX[Size], posY[Size];
  double c0, c1, cov00, cov01, cov11, sumsq;

  for (int j=0; j<Size; j++){
    posX[j] = x[First+j];
    posY[j] = y[First+j];
  }
  LinearRegression lr(posX,posY,Size);
  coefA = lr.getB();
  coefB = lr.getA();

}

double SegLasPts::getLength(){
  return Length;
}

vector<double> SegLasPts::getX(){
  return X;
}

vector<double> SegLasPts::getY(){
  return Y;
}

void SegLasPts::setLineCoefA(double A){
  coefA = A;
}

void SegLasPts::setLineCoefB(double B){
  coefB = B;
}

double SegLasPts::getLineCoefA(){
  return coefA;
}

double SegLasPts::getLineCoefB(){
  return coefB;
}

void SegLasPts::getParams(int& first,int& last ,int& size, double& A, double& B) const{
  first = First;
  last = Last;
  size = Size;
  A = coefA;
  B = coefB;

}

void SegLasPts::setParams(int first, int last, int size, double A, double B){
  First = first;
  Last = last;
  Size = size;
  coefA = A;
  coefB = B;
}

/*const SegLasPts& SegLasPts::operator=(const SegLasPts& otherSeg){
  if (this != &otherSeg){

    First = otherSeg.First;
    Last = otherSeg.Last;
    Size = otherSeg.Size;
    firstPoint = otherSeg.firstPoint;
    lastPoint = otherSeg.lastPoint;
    midPoint = otherSeg.midPoint;
    Length = otherSeg.Length;
    coefA = otherSeg.coefA;
    coefB = otherSeg.coefB;
    for (int i=0;i<3;i++){
      X[i] = otherSeg.X[i];
      Y[i] = otherSeg.Y[i];
    }
    return *this;
  }
}*/

SegLasPts::~SegLasPts()
{
  // TODO Auto-generated destructor stub
}

} /* namespace SegLasPts */
