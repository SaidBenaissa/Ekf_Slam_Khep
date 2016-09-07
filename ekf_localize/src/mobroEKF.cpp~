/*
 * mobroEKF.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: towardthesea
 */

#include "mobroEKF.h"
//#include "Eigen/Dense"


namespace mobroEKF
{

mobroEKF::mobroEKF()
{
  // TODO Auto-generated constructor stub

}

mobroEKF::mobroEKF(double period, double l,
                   double sigmaX, double sigmaY, double sigmaTheta,
                   double sigmaWheel,
                   double sigmaMeaDist, double sigmaMeaAngle,
                   double distMahaThres){
  Period = period;
  L = l;
  sigX = sigmaX;
  sigY = sigmaY;
  sigTheta = sigmaTheta;
  sigWheel = sigmaWheel;
  sigMeaDist = sigmaMeaDist;
  sigMeaAngle = sigmaMeaAngle;
  DistMahaThres = distMahaThres;
}

void mobroEKF::setInitP(double x, double y, double theta){
  PoX = x;
  PoY = y;
  PoTheta = theta;
}

void mobroEKF::setSpeed(double v, double w){
  LinearSpeed = v;
  AngleSpeed = w;
  dD = LinearSpeed*Period;
  dTheta = AngleSpeed*Period;
}

void mobroEKF::setPeriod(double period){
  Period = period;
}

void mobroEKF::setPose(double x, double y, double theta){
  Xk = x;
  Yk = y;
  Thetak = theta;
}

laserPoint::laserPoint mobroEKF::transform(laserPoint::laserPoint corner){
  laserPoint::laserPoint newCorner;
  double xC = corner.getX();
  double yC = corner.getY();
  double x = Xk + xC*cos(Thetak)-yC*sin(Thetak);
  double y = Yk + xC*sin(Thetak)-yC*cos(Thetak);
  newCorner.setPoint(x,y);
  return newCorner;
}

void mobroEKF::setMatrixA(){
  A(0,0)= 1.0;
  A(0,1)= 0.0;
  A(0,2)= -dD*sin(Thetak);
  A(1,0)= 0.0;
  A(1,1)= 1.0;
  A(1,2)= dD*cos(Thetak);
  A(2,0)= 0.0;
  A(2,1)= 0.0;
  A(2,2)= 1.0;
}

void mobroEKF::setMatrixB(){
  B.resize(3,2);
  B(0,0) = Period*cos(Thetak);
  B(0,1) = 0.0;
  B(1,0) = Period*sin(Thetak);
  B(1,1) = 0.0;
  B(2,0) = 0.0;
  B(2,1) = Period;
}



MatrixXd mobroEKF::setMatrixC(laserPoint::laserPoint corner){
  laserPoint::laserPoint curPosition(Xk,Yk);
  double distance = corner.p2pDistance(curPosition);
  MatrixXd tempC;
  tempC.resize(1,3);

  tempC(0,0) = (Xk-corner.getX())/distance;
  tempC(0,1) = (Yk-corner.getY())/distance;
  tempC(0,2) = 0.0;
  return tempC;
}

void mobroEKF::matrixInit(){
  Vector3d vectorQa(sigX*sigX,sigY*sigY,sigTheta*sigTheta);
  Qa = vectorQa.asDiagonal();

  Qb = sigWheel*Matrix2d::Identity();

//  Vector2d vectorQg(sigMeaDist*sigMeaDist,sigMeaAngle*sigMeaAngle);
//  Qg = vectorQg.asDiagonal();

  // For the case of measure distance only
  Qg.resize(1,1);
  Qg(0,0) = sigMeaDist*sigMeaDist;

  Vector3d vectorPo(PoX*PoX, PoY*PoY,PoTheta*PoTheta);
  P = vectorPo.asDiagonal();

  predPose.resize(3,1);
  estPose.resize(3,1);
  K.resize(3,1);
}

void mobroEKF::prediction(){
  Xk = Xk + dD*cos(Thetak);                     //add at 06-25-15:15
  Yk = Yk + dD*sin(Thetak);
  Thetak = Thetak + dTheta;

  predPose << Xk,
              Yk,
              Thetak;
}

void mobroEKF::propagateError(){
  P = A*P*A.transpose()+B*Qb*B.transpose()+Qa;
}

void mobroEKF::setRealMeasure(vector<laserPoint::laserPoint> measureCorners){
  laserPoint::laserPoint curPosition(Xk,Yk);
  realMeasure.resize(measureCorners.size());
  for (int i=0;i<measureCorners.size();i++){
    laserPoint::laserPoint globCorner = this->transform(measureCorners[i]);
    // For the case of measure distance only
    realMeasure[i]= globCorner.p2pDistance(curPosition);              //edit 06-21-15:25

    //realMeasure[i]= measureCorners[i].p2pDistance(curPosition);         //edit 06-21-15:25
  }
}

void mobroEKF::setEstMeasure(vector<laserPoint::laserPoint> mapCorners){
  laserPoint::laserPoint curPosition(Xk,Yk);            //add at 06-25-17:15
  estMeasure.resize(mapCorners.size());
  for (int i=0;i<mapCorners.size();i++){
    // For the case of measure distance only
    estMeasure[i]= mapCorners[i].p2pDistance(curPosition);
  }
}

bool mobroEKF::hasMeasure(vector<double> measureDist, vector<double> measureAngle,
                          vector<laserPoint::laserPoint> mapCorners){
  if (measureDist.size()>0 && mapCorners.size()>0){

    int countGoodMeasure = 0;
    int countRejectedCorners =0;

    MatrixXd tempC(1,3),tempC_good(1,3);
    MatrixXd tempMeaCov(1,1),tempMeaCov_good(1,1);
    MatrixXd deltaMeasure(1,1),deltaMeasure_good(1,1);// = realMeasure[j]-estMeasure[i];

    this->setEstMeasure(mapCorners);
    for (int j=0;j<measureDist.size();j++){
      if (countGoodMeasure>1){
        countGoodMeasure = 0;
      }
      for (int i=0;i<mapCorners.size();i++){
        tempC = this->setMatrixC(mapCorners[i]);
        tempMeaCov = tempC*P*tempC.transpose() + Qg;        // Temporary Measure Covariance matrix
        deltaMeasure(0,0) = measureDist[j]-estMeasure[i];
        MatrixXd DistMaha = deltaMeasure.transpose()*tempMeaCov.inverse()*deltaMeasure;
        double distMahaSquare = DistMaha(0,0);
        if (sqrt(distMahaSquare)<DistMahaThres){
          countGoodMeasure++;
          tempC_good = tempC;
          tempMeaCov_good = tempMeaCov;
          deltaMeasure_good = deltaMeasure;
        }
        else {
          ROS_INFO("Reject corner: %dth", j);
          countRejectedCorners++;
        }
      }
      ROS_INFO("Numbers of rejected corners: %d", countRejectedCorners);
    }
    if (countGoodMeasure ==1){
      C=tempC_good;
      MeaCov = tempMeaCov_good;
      K = P*C.transpose()*MeaCov.inverse();
      estPose = predPose + K*deltaMeasure_good;
      P = (Matrix3d::Identity()-K*C)*P;

      //add 2014-06-25-15:00
      Xk = estPose(0,0);
      Yk = estPose(1,0);
      Thetak = estPose(2,0);

      return true;
    }
    else {
      //ROS_INFO("Numbers of bad corners: %d",countGoodMeasure);
      return false;
    }
  }
  else {
    return false;
  }
}

void mobroEKF::getSpeed(double& v,double& w){
  v = LinearSpeed;
  w = AngleSpeed;

}

void mobroEKF::getPose(double& x,double& y, double& theta){
  x = Xk;
  y = Yk;
  theta = Thetak;
}

void mobroEKF::getPredPose(double& x,double& y, double& theta){
  x = predX;
  y = predY;
  theta = predTheta;
}

void mobroEKF::getEstPose(double& x,double& y, double& theta){
  estX = estPose(0,0);
  estY = estPose(1,0);
  estTheta = estPose(2,0);
  x = estX;
  y = estY;
  theta = estTheta;
}

Matrix3d mobroEKF::getCovariance(){
  return P;
}

mobroEKF::~mobroEKF()
{
  // TODO Auto-generated destructor stub
}

} /* namespace mobroEKF */
