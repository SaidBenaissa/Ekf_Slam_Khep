/*
 * mobroEKF.h
 *
 *  Created on: Jun 18, 2014
 *      Author: towardthesea
 */

#ifndef MOBROEKF_H_
#define MOBROEKF_H_
#include "Eigen/Dense"
#include "laserPoint.h"

using namespace std;
using namespace Eigen;
namespace mobroEKF

{

class mobroEKF
{


public:
  mobroEKF();
  mobroEKF(double,double,double,double,double,double,double,double,double);
  void setInitP(double, double, double);
  void setSpeed(double, double);
  void setPeriod(double);
  void setPose(double,double,double);
  void setRealMeasure(vector<laserPoint::laserPoint>);
  void setEstMeasure(vector<laserPoint::laserPoint>);
  void setMatrixA();
  void setMatrixB();
  MatrixXd setMatrixC(laserPoint::laserPoint);
  void matrixInit();
  void prediction();
  void propagateError();

  laserPoint::laserPoint transform(laserPoint::laserPoint);
  //bool hasMeasure(vector<laserPoint::laserPoint>,vector<laserPoint::laserPoint>);
  bool hasMeasure(vector<double>,vector<double>,vector<laserPoint::laserPoint>);

  void getSpeed(double&,double&);
  void getPose(double&,double&,double&);
  void getPredPose(double&,double&,double&);
  void getEstPose(double&,double&,double&);
  Matrix3d getCovariance();
  virtual ~mobroEKF();
private:
  double Period, LinearSpeed, AngleSpeed,L, sigX, sigY, sigTheta, sigWheel, sigMeaDist, sigMeaAngle, dD,dTheta, DistMahaThres;
  double PoX,PoY,PoTheta;
  double Xk,Yk,Thetak;                  //posture of robot without kalman filter
  double predX,predY,predTheta;         //predict posture of robot
  double estX,estY,estTheta;            //estimated posture of robot
  vector<double> realMeasure, estMeasure;

  MatrixXd Qa;          //System noise covariance matrix
  MatrixXd Qb;          //Input noise covariance matrix
  MatrixXd Qg;          //Measurement noise covariance matrix
  Matrix3d P;           //Covariance matrix
  MatrixXd MeaCov;      //Measure Covariance matrix
  Matrix3d A;           //Jacobian matrix of system dX=f(U,X) over X
  MatrixXd B;           //Jacobian matrix of system dX=f(U,X) over U
  MatrixXd C;           //Jacobian matrix of measurement Y=g(X) over X
  MatrixXd K;           //Gain matrix
  MatrixXd predPose;    //prediction Pose matrix
  MatrixXd estPose;     //Estimated Pose matrix
};

} /* namespace mobroEKF */

#endif /* MOBROEKF_H_ */
