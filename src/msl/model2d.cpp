//----------------------------------------------------------------------
//               The Motion Strategy Library (MSL)
//----------------------------------------------------------------------
//
// Copyright (c) University of Illinois and Steven M. LaValle.
// All Rights Reserved.
//
// Permission to use, copy, and distribute this software and its
// documentation is hereby granted free of charge, provided that
// (1) it is not a component of a commercial product, and
// (2) this notice appears in all copies of the software and
//     related documentation.
//
// The University of Illinois and the author make no representations
// about the suitability or fitness of this software for any purpose.
// It is provided "as is" without express or implied warranty.
//----------------------------------------------------------------------

//#include <fstream.h>
#include <math.h>

#include "msl/model2d.h"

#include "msl/defs.h"

#define NUM_INPUTS_2DPOINT 50
#define NUM_INPUTS_2DRIGID 8


// *********************************************************************
// *********************************************************************
// CLASS:     Model2D
//
// *********************************************************************
// *********************************************************************



Model2D::Model2D(string path = ""):Model(path) {
}


MSLVector Model2D::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  q[0] = x[0]; q[1] = x[1]; q[2] = 0.0;

  return q;
}



// *********************************************************************
// *********************************************************************
// CLASS:     Model2DPoint
//
// *********************************************************************
// *********************************************************************


// Constructor
Model2DPoint::Model2DPoint(string path = ""):Model2D(path) {
  double theta;

  StateDim = 2;
  InputDim = 2;

  READ_PARAMETER_OR_DEFAULT(LowerState,MSLVector(0.0,0.0));
  READ_PARAMETER_OR_DEFAULT(UpperState,MSLVector(100.0,100.0));

  // Make the list of Inputs
  Inputs.clear();
  for (theta = 0.0; theta < 2.0*PI; theta += 2.0*PI/NUM_INPUTS_2DPOINT) {
    Inputs.push_back(MSLVector(cos(theta),sin(theta)));
  }

  READ_OPTIONAL_PARAMETER(Inputs);

}




MSLVector Model2DPoint::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(2);

  dx = u;
  return dx;
}



double Model2DPoint::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho;

  rho = (x1 - x2).length();

  return rho;
}





MSLVector Model2DPoint::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return EulerIntegrate(x,u,h);
}



// *********************************************************************
// *********************************************************************
// CLASS:     Model::Model2DPointCar
//
// *********************************************************************
// *********************************************************************


// Constructor
Model2DPointCar::Model2DPointCar(string path = ""):Model2DPoint(path) {
  double alpha;

  StateDim = 3;
  InputDim = 2;

  READ_PARAMETER_OR_DEFAULT(LowerState,MSLVector(0.0,0.0,0.0));
  READ_PARAMETER_OR_DEFAULT(UpperState,MSLVector(100.0,100.0,2.0*PI));

  MaxSteeringAngle = PI/6.0;
  CarLength = 4.0;

  // Make the list of Inputs
  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle;
       alpha += 2.0*MaxSteeringAngle/6.0) {
    Inputs.push_back(MSLVector(1.0,alpha));
    Inputs.push_back(MSLVector(-1.0,alpha));
  }

  READ_OPTIONAL_PARAMETER(Inputs);

}



MSLVector Model2DPointCar::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(3);

  dx[0] = u[0]*cos(x[2]);
  dx[1] = u[0]*sin(x[2]);
  dx[2] = u[0]*tan(u[1])/CarLength;
  return dx;
}



double Model2DPointCar::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho,dtheta;

  dtheta = min(fabs(x1[2]-x2[2]),2.0*PI - fabs(x1[2]-x2[2]));

  rho = sqrt(sqr(x1[0] - x2[0]) + sqr(x1[1] - x2[1]) + 50.0/PI*sqr(dtheta));

  return rho;
}



MSLVector Model2DPointCar::Integrate(const MSLVector &x, const MSLVector &u,
				  const double &h) {
  MSLVector nx(3);

  nx = RungeKuttaIntegrate(x,u,h);

  // Make sure the S^1 topology is preserved for 2D rotation
  if (nx[2] > 2.0*PI)
    nx[2] -= 2.0*PI;
  if (nx[2] < 0.0)
    nx[2] += 2.0*PI;

  return nx;
}



// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigid
//
// *********************************************************************
// *********************************************************************



// Constructor
Model2DRigid::Model2DRigid(string path = ""):Model2D(path) {
  double theta;

  StateDim = 3;
  InputDim = 3;

  READ_PARAMETER_OR_DEFAULT(LowerState,MSLVector(0.0,0.0,0.0));
  READ_PARAMETER_OR_DEFAULT(UpperState,MSLVector(100.0,100.0,2.0*PI));

  // Make the list of Inputs
  Inputs.clear();
  for (theta = 0.0; theta < 2.0*PI; theta += 2.0*PI/NUM_INPUTS_2DRIGID) {
    Inputs.push_back(MSLVector(cos(theta),sin(theta),0.0));
  }
  Inputs.push_back(MSLVector(0.0,0.0,-0.1));
  Inputs.push_back(MSLVector(0.0,0.0,0.1));

  READ_OPTIONAL_PARAMETER(Inputs);

}



MSLVector Model2DRigid::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(3);

  dx = u;
  return dx;
}



double Model2DRigid::Metric(const MSLVector &x1, const MSLVector &x2) {

  double fd = fabs(x1[2]-x2[2]);
  double dtheta = min(fd,2.0*PI - fd);

  return sqrt(sqr(x1[0] - x2[0]) + sqr(x1[1] - x2[1]) + sqr(50.0/PI*dtheta));

}



// Handle S^1 topology properly (for rotation)
MSLVector Model2DRigid::LinearInterpolate(const MSLVector &x1, const MSLVector &x2,
				       const double &a) {

  MSLVector v;

  v = (1.0-a)*x1 + a*x2;

  if (fabs(x2[2] - x1[2]) > PI) {
    if (x1[2] > x2[2])
      v[2] = (1.0-a)*x1[2] + a*(x2[2]+2.0*PI);
    else
      v[2] = (1.0-a)*(x1[2]+2.0*PI) + a*x2[2];
  }

  if (v[2] > 2.0*PI)
    v[2] -= 2.0*PI;

  return v;
}



MSLVector Model2DRigid::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  MSLVector nx(3);

  nx = RungeKuttaIntegrate(x,u,h);

  // Make sure the S^1 topology is preserved for 2D rotation
  if (nx[2] > 2.0*PI)
    nx[2] -= 2.0*PI;
  if (nx[2] < 0.0)
    nx[2] += 2.0*PI;

  return nx;
}


MSLVector Model2DRigid::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  q[0] = x[0]; q[1] = x[1]; q[2] = x[2];

  return q;
}


// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidCar
//
// *********************************************************************
// *********************************************************************


// Constructor
Model2DRigidCar::Model2DRigidCar(string path = ""):Model2DRigid(path) {
  double alpha;

  StateDim = 3;
  InputDim = 2;

  MaxSteeringAngle = PI/12.0;
  CarLength = 2.0;

  // Make the list of Inputs
  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle;
       alpha += 2.0*MaxSteeringAngle/6.0) {
    Inputs.push_back(MSLVector(1.0,alpha));
    Inputs.push_back(MSLVector(-1.0,alpha));
  }

  READ_OPTIONAL_PARAMETER(Inputs);

}


MSLVector Model2DRigidCar::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(3);
  dx[0] = u[0]*cos(x[2]);
  dx[1] = u[0]*sin(x[2]);
  dx[2] = u[0]*tan(u[1])/CarLength;
  return dx;
}


// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidCarForward
//
// *********************************************************************
// *********************************************************************

Model2DRigidCarForward::Model2DRigidCarForward(string path = ""):Model2DRigidCar(path) {
  double alpha;

  StateDim = 3;
  InputDim = 2;

  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle;
       alpha += 2.0*MaxSteeringAngle/6.0) {
    Inputs.push_back(MSLVector(1.0,alpha));
  }

  READ_OPTIONAL_PARAMETER(Inputs);

}


// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidCarSmooth
//  Smooth steering
// *********************************************************************
// *********************************************************************


Model2DRigidCarSmooth::Model2DRigidCarSmooth(string path = ""):Model2DRigidCar(path) {

  StateDim = 4;
  InputDim = 2;

  LowerState = MSLVector(4);
  UpperState = MSLVector(4);

  READ_PARAMETER_OR_DEFAULT(SteeringSpeed,0.05);


  LowerState[0] = 0.0; LowerState[1] = 0.0; LowerState[2] = 0.0;
  LowerState[3] = -MaxSteeringAngle;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 100.0; UpperState[1] = 100.0; UpperState[2] = 2.0*PI;
  UpperState[3] = MaxSteeringAngle;
  READ_OPTIONAL_PARAMETER(LowerState);

  // Make the list of Inputs
  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  Inputs.push_back(MSLVector(1.0,0.0)); // Keep the steering angle fixed
  Inputs.push_back(MSLVector(-1.0,0.0)); // Keep the steering angle fixed
  Inputs.push_back(MSLVector(1.0,SteeringSpeed));
  Inputs.push_back(MSLVector(-1.0,SteeringSpeed));
  Inputs.push_back(MSLVector(1.0,-SteeringSpeed));
  Inputs.push_back(MSLVector(-1.0,-SteeringSpeed));

  READ_OPTIONAL_PARAMETER(Inputs);

}



MSLVector Model2DRigidCarSmooth::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(4);

  dx[0] = u[0]*cos(x[2]);
  dx[1] = u[0]*sin(x[2]);
  dx[2] = u[0]*tan(x[3])/CarLength;
  dx[3] = u[1];

  //cout << "DX: " << dx << "\n";

  return dx;
}




MSLVector Model2DRigidCarSmooth::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return RungeKuttaIntegrate(x,u,h);
}


double Model2DRigidCarSmooth::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho,dphi,dtheta;

  dphi = min(fabs(x1[3]-x2[3]),2.0*PI - fabs(x1[3]-x2[3]));
  dtheta = min(fabs(x1[2]-x2[2]),2.0*PI - fabs(x1[2]-x2[2]));

  rho = sqrt(sqr(x1[0] - x2[0]) + sqr(x1[1] - x2[1]) +
    sqr(2.0/PI*dphi) +
    sqr(50.0/PI*dtheta));

  return rho;
}


MSLVector Model2DRigidCarSmooth::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  q[0] = x[0]; q[1] = x[1]; q[2] = x[2];

  return q;
}


bool Model2DRigidCarSmooth::Satisfied(const MSLVector &x)
{
  return ((x[3] < UpperState[3])&& // Steering is too sharp!
	  (x[3] > LowerState[3]));
}


// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidCarSmoothTrailer
//  Smooth steering
// *********************************************************************
// *********************************************************************


Model2DRigidCarSmoothTrailer::Model2DRigidCarSmoothTrailer(string path = ""):Model2DRigidCarSmooth(path) {

  StateDim = 5;
  InputDim = 2;

  LowerState = MSLVector(5);
  UpperState = MSLVector(5);

  READ_PARAMETER_OR_DEFAULT(HitchLength,10.0);
  READ_PARAMETER_OR_DEFAULT(HitchMaxAngle,PI/2.0); // From 0 to PI


  LowerState[0] = 0.0; LowerState[1] = 0.0; LowerState[2] = 0.0;
  LowerState[3] = -MaxSteeringAngle; LowerState[4] = 0.0;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 100.0; UpperState[1] = 100.0; UpperState[2] = 2.0*PI;
  UpperState[3] = MaxSteeringAngle; UpperState[4] = 2.0*PI;
  READ_OPTIONAL_PARAMETER(UpperState);

}



MSLVector Model2DRigidCarSmoothTrailer::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(5);

  dx[0] = u[0]*cos(x[2]);
  dx[1] = u[0]*sin(x[2]);
  dx[2] = u[0]*tan(x[3])/CarLength;
  dx[3] = u[1];
  dx[4] = u[0]*sin(x[2] - x[4])/HitchLength;
  return dx;
}




double Model2DRigidCarSmoothTrailer::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho,dphi,dtheta,dtheta1;

  dphi = min(fabs(x1[3]-x2[3]),2.0*PI - fabs(x1[3]-x2[3]));
  dtheta = min(fabs(x1[2]-x2[2]),2.0*PI - fabs(x1[2]-x2[2]));
  dtheta1 = min(fabs(x1[4]-x2[4]),2.0*PI - fabs(x1[4]-x2[4]));

  rho = sqrt(sqr(x1[0] - x2[0]) +
	     sqr(x1[1] - x2[1]) +
	     sqr(2.0/PI*dphi) +
	     sqr(5.0/PI*dtheta) +
	     sqr(5.0/PI*dtheta1));

  return rho;
}


MSLVector Model2DRigidCarSmoothTrailer::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(6);  // Two bodies

  // The car config
  q[0] = x[0]; q[1] = x[1]; q[2] = x[2];

  // The trailer config
  q[3] = -cos(x[4])*HitchLength+x[0];
  q[4] = -sin(x[4])*HitchLength+x[1];
  q[5] = x[4];

  return q;
}


bool Model2DRigidCarSmoothTrailer::Satisfied(const MSLVector &x)
{
  return ((x[3] < UpperState[3])&& // Steering is too sharp!
	  (x[3] > LowerState[3])&&
	  (cos(x[2]-x[4]) >= cos(HitchMaxAngle)));
}


// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidCarSmooth2Trailers
//  Smooth steering
// *********************************************************************
// *********************************************************************


Model2DRigidCarSmooth2Trailers::Model2DRigidCarSmooth2Trailers(string path = ""):Model2DRigidCarSmoothTrailer(path) {

  StateDim = 6;
  InputDim = 2;

  LowerState = MSLVector(6);
  UpperState = MSLVector(6);

  READ_PARAMETER_OR_DEFAULT(Hitch2Length,10.0);
  READ_PARAMETER_OR_DEFAULT(Hitch2MaxAngle,PI/2.0); // From 0 to PI

  LowerState[0] = 0.0; LowerState[1] = 0.0; LowerState[2] = 0.0;
  LowerState[3] = -MaxSteeringAngle; LowerState[4] = 0.0;
  LowerState[5] = 0.0;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 100.0; UpperState[1] = 100.0; UpperState[2] = 2.0*PI;
  UpperState[3] = MaxSteeringAngle; UpperState[4] = 2.0*PI;
  UpperState[5] = 2.0*PI;
  READ_OPTIONAL_PARAMETER(UpperState);

}



MSLVector Model2DRigidCarSmooth2Trailers::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(6);

  dx[0] = u[0]*cos(x[2]);
  dx[1] = u[0]*sin(x[2]);
  dx[2] = u[0]*tan(x[3])/CarLength;
  dx[3] = u[1];
  dx[4] = u[0]*sin(x[2] - x[4])/HitchLength;
  dx[5] = u[0]*cos(x[2] - x[4])*sin(x[4] - x[5])/Hitch2Length;
  return dx;
}



double Model2DRigidCarSmooth2Trailers::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho,dphi,dtheta,dtheta1,dtheta2;

  dphi = min(fabs(x1[3]-x2[3]),2.0*PI - fabs(x1[3]-x2[3]));
  dtheta = min(fabs(x1[2]-x2[2]),2.0*PI - fabs(x1[2]-x2[2]));
  dtheta1 = min(fabs(x1[4]-x2[4]),2.0*PI - fabs(x1[4]-x2[4]));
  dtheta2 = min(fabs(x1[5]-x2[5]),2.0*PI - fabs(x1[5]-x2[5]));

  rho = sqrt(sqr(x1[0] - x2[0]) + sqr(x1[1] - x2[1]) +
    sqr(2.0/PI*dphi) +
    sqr(5.0/PI*dtheta) +
    sqr(5.0/PI*dtheta1) +
    sqr(5.0/PI*dtheta2));

  return rho;
}


MSLVector Model2DRigidCarSmooth2Trailers::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(9);  // Three bodies

  // The car config
  q[0] = x[0]; q[1] = x[1]; q[2] = x[2];

  // The 1st trailer config
  q[3] = -cos(x[4])*HitchLength+x[0];
  q[4] = -sin(x[4])*HitchLength+x[1];
  q[5] = x[4];

  // The 2nd trailer config
  q[6] = -cos(x[5])*Hitch2Length+q[3];
  q[7] = -sin(x[5])*Hitch2Length+q[4];
  q[8] = x[5];

  return q;
}


bool Model2DRigidCarSmooth2Trailers::Satisfied(const MSLVector &x)
{
  return ((x[3] < UpperState[3])&& // Steering is too sharp!
	  (x[3] > LowerState[3])&&
	  (cos(x[2]-x[4]) >= cos(HitchMaxAngle))&&
	  (cos(x[4]-x[5]) >= cos(Hitch2MaxAngle)));
}


// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidCarSmooth3Trailers
//  Smooth steering
// *********************************************************************
// *********************************************************************


Model2DRigidCarSmooth3Trailers::Model2DRigidCarSmooth3Trailers(string path = ""):Model2DRigidCarSmooth2Trailers(path) {

  StateDim = 7;
  InputDim = 2;

  LowerState = MSLVector(7);
  UpperState = MSLVector(7);

  READ_PARAMETER_OR_DEFAULT(Hitch3Length,10.0);
  READ_PARAMETER_OR_DEFAULT(Hitch3MaxAngle,PI/2.0); // From 0 to PI

  LowerState[0] = 0.0; LowerState[1] = 0.0; LowerState[2] = 0.0;
  LowerState[3] = -MaxSteeringAngle; LowerState[4] = 0.0;
  LowerState[5] = 0.0; LowerState[6] = 0.0;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 100.0; UpperState[1] = 100.0; UpperState[2] = 2.0*PI;
  UpperState[3] = MaxSteeringAngle; UpperState[4] = 2.0*PI;
  UpperState[5] = 2.0*PI; UpperState[6] = 2.0*PI;
  READ_OPTIONAL_PARAMETER(UpperState);

}



MSLVector Model2DRigidCarSmooth3Trailers::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(7);

  dx[0] = u[0]*cos(x[2]);
  dx[1] = u[0]*sin(x[2]);
  dx[2] = u[0]*tan(x[3])/CarLength;
  dx[3] = u[1];
  dx[4] = u[0]*sin(x[2] - x[4])/HitchLength;
  dx[5] = u[0]*cos(x[2] - x[4])*sin(x[4] - x[5])/Hitch2Length;
  dx[6] = u[0]*cos(x[2] - x[4])*cos(x[4] - x[5])*sin(x[5]-x[6])/Hitch3Length;
  return dx;
}




double Model2DRigidCarSmooth3Trailers::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho,dphi,dtheta,dtheta1,dtheta2,dtheta3;

  dphi = min(fabs(x1[3]-x2[3]),2.0*PI - fabs(x1[3]-x2[3]));
  dtheta = min(fabs(x1[2]-x2[2]),2.0*PI - fabs(x1[2]-x2[2]));
  dtheta1 = min(fabs(x1[4]-x2[4]),2.0*PI - fabs(x1[4]-x2[4]));
  dtheta2 = min(fabs(x1[5]-x2[5]),2.0*PI - fabs(x1[5]-x2[5]));
  dtheta3 = min(fabs(x1[6]-x2[6]),2.0*PI - fabs(x1[6]-x2[6]));

  rho = sqrt(sqr(x1[0] - x2[0]) + sqr(x1[1] - x2[1]) +
    sqr(2.0/PI*dphi) +
    sqr(5.0/PI*dtheta) +
    sqr(5.0/PI*dtheta1) +
    sqr(5.0/PI*dtheta2) +
    sqr(5.0/PI*dtheta3));

  return rho;
}



MSLVector Model2DRigidCarSmooth3Trailers::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(12);  // Four bodies

  // The car config
  q[0] = x[0]; q[1] = x[1]; q[2] = x[2];

  // The 1st trailer config
  q[3] = -cos(x[4])*HitchLength+x[0];
  q[4] = -sin(x[4])*HitchLength+x[1];
  q[5] = x[4];

  // The 2nd trailer config
  q[6] = -cos(x[5])*Hitch2Length+q[3];
  q[7] = -sin(x[5])*Hitch2Length+q[4];
  q[8] = x[5];

  // The 3rd trailer config
  q[9]  = -cos(x[6])*Hitch3Length+q[6];
  q[10] = -sin(x[6])*Hitch3Length+q[7];
  q[11] = x[6];

  return q;
}


bool Model2DRigidCarSmooth3Trailers::Satisfied(const MSLVector &x)
{
  return ((x[3] < UpperState[3])&& // Steering is too sharp!
	  (x[3] > LowerState[3])&&
	  (cos(x[2]-x[4]) >= cos(HitchMaxAngle))&&
	  (cos(x[4]-x[5]) >= cos(Hitch2MaxAngle))&&
	  (cos(x[5]-x[6]) >= cos(Hitch3MaxAngle)));
}


// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidDyncar
//
// *********************************************************************
// *********************************************************************

// Constructor
Model2DRigidDyncar::Model2DRigidDyncar(string path = ""):Model2DRigid(path) {
  double alpha;
  MSLVector v;

  StateDim = 5;
  InputDim = 1;
  Mass = 100.0;
  CAF = 17000.0;
  CAR = 20000.0;
  Adist = 4.0;
  Bdist = 5.0;
  Izz = 1600.0;

  WorldScale = 0.1;
  Speed = 88.0;   // Feet per sec

  LowerState = MSLVector(5);
  UpperState = MSLVector(5);

  LowerState[0] = -50.0; LowerState[1] = -5.0; LowerState[2] = 0.0;
  LowerState[3] = -1000.0; LowerState[4] = 0.0;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 50.0; UpperState[1] = 5.0; UpperState[2] = 1000.0;
  UpperState[3] = 0.0; UpperState[4] = 2.0*PI;
  READ_OPTIONAL_PARAMETER(UpperState);

  MaxSteeringAngle = 0.6;

  // Make the list of 1D Inputs
  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle;
       alpha += 2.0*MaxSteeringAngle/24.0) {
    v = MSLVector(1); v[0] = alpha;
    Inputs.push_back(v);
  }

  READ_OPTIONAL_PARAMETER(Inputs);

}


// This is xdot = f(x,u) for a 5dof state-space, and 1dof input
//  Model taken from Jim Bernard...
// alphaf = (v + a*r)/u - deltaf
// alphar = (v-b*r)/u
// FYF = -CAF*alphaf
// FYR = -CAR*alphar
// vdot = -u*r  + (FYF + FYR)/m
// rdot = (FYF*a - FYR*b)/Izz
// Xdot = u*cos(psi) - v*sin(psi)
// Ydot = u*sin(spi) + v*cos(psi)
// psidot = r
//
// m = mass of car, say about 100 slugs
// CAF = front cornering stiffness in pounds per radiaof the tires, say about
// 17000
// CAR = rear cornering stiffness, say about 20000
// a is dist from mass center to front tires, say 4 feet
// b is dist from mass center to rear tires, say 5 feet
// Izz is yaw moment of intertia, say about 1600 slug ft**2
// u is forward speed which is assumed constant, input is in feet/sec
// delta is your input, it is the steer angle of the tires in radians.

MSLVector Model2DRigidDyncar::StateTransitionEquation(const MSLVector &x, const MSLVector &u)
{
  double alphaf,alphar,fyf,fyr,v,r,psi;
  MSLVector dx(5);

  v = x[0]; r = x[1]; psi = x[4];

  alphaf = (v + Adist * r) / Speed - u[0];
  alphar = (v - Bdist * r) / Speed;
  fyf = -CAF * alphaf;
  fyr = -CAR * alphar;

  /* Transfer the velocity */
  dx[0] = -Speed * r  + (fyf + fyr) / Mass;
  dx[1] = (fyf * Adist - fyr * Bdist) / Izz;
  dx[2] = Speed * cos(psi) - v * sin(psi);
  dx[3] = Speed * sin(psi) + v * cos(psi);
  dx[4] = r;

  //cout << "x: " << x << "  Dx: " << dx << "  u: " << u[0] << "\n";

  return dx;
}



double Model2DRigidDyncar::Metric(const MSLVector &x1, const MSLVector &x2) {
  double d;

  // Position difference
  d =  sqr((x1[2] - x2[2]) / (UpperState[2] - LowerState[2]));
  d += sqr((x1[3] - x2[3]) / (UpperState[3] - LowerState[3]));

  // Orientation difference
  d += sqr(min(fabs(x1[4]-x2[4]),2.0*PI - fabs(x1[4]-x2[4]))/2.0/PI);

  // Velocities
  d += sqr((x1[0] - x2[0]) / (UpperState[0] - LowerState[0]));
  d += sqr((x1[1] - x2[1]) / (UpperState[1] - LowerState[1]));


  return sqrt(d);
}



MSLVector Model2DRigidDyncar::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return RungeKuttaIntegrate(x,u,h);
}


// NOTE: This neglects the S^1 effect of orientation!!
MSLVector Model2DRigidDyncar::StateToConfiguration(const MSLVector &x) {
  return MSLVector(x[2]*WorldScale,-x[3]*WorldScale,2.0*PI-x[4]);
}


MSLVector Model2DRigidDyncar::LinearInterpolate(const MSLVector &x1,
					     const MSLVector &x2,
					     const double &a) {

  MSLVector v;

  v = (1.0-a)*x1 + a*x2;

  if (fabs(x2[4] - x1[4]) > PI) {
    if (x1[4] > x2[4])
      v[4] = (1.0-a)*x1[4] + a*(x2[4]+2.0*PI);
    else
      v[4] = (1.0-a)*(x1[4]+2.0*PI) + a*x2[4];
  }

  if (v[4] > 2.0*PI)
    v[4] -= 2.0*PI;

  return v;
}



// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidDyncarNtire
//
// *********************************************************************
// *********************************************************************

// Constructor
Model2DRigidDyncarNtire::Model2DRigidDyncarNtire(string path = ""):Model2DRigidDyncar(path) {
  double alpha;
  MSLVector v;

  StateDim = 5;
  InputDim = 2;

  // These are the exact parameters from Ric's model
  Mass = 3518.0/32.2;
  Adist = 0.45*100.5/12.0;
  Bdist = 0.55*100.5/12.0;
  Izz = 0.25*Mass*(Adist+Bdist)*(Adist+Bdist);

  // Constants for the nonlinear tire model
  Mu = 0.85;
  Nf = Mass*32.2*0.55;
  Nr = Mass*32.2*0.45;

  // Make the list of 2D Inputs
  // Apparently, this models allows rear-wheel steering
  //    This option is set to zero for now...hence v[1]=0.0
  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle;
       alpha += 2.0*MaxSteeringAngle/24.0) {
    v = MSLVector(2); v[0] = alpha; v[1] = 0.0;
    Inputs.push_back(v);
  }

  READ_OPTIONAL_PARAMETER(Inputs);

}


// The is the model from Jim Bernard and Ric Menendez
MSLVector Model2DRigidDyncarNtire::StateTransitionEquation(const MSLVector &x, const MSLVector &u)
{
  double alphaf,alphar,fyf,fyr,v,r,psi;
  double talff,talfr,xiblf,xiblr;
  MSLVector dx(5);


  v = x[0]; r = x[1]; psi = x[4];

  alphaf = (v + Adist * r) / Speed - u[0];
  alphar = (v - Bdist * r) / Speed - u[1];

  // Here is where the nonlinear tire model is used
  //  The result is new values for fyf and fyr
  talff = tan(fabs(alphaf));
  talfr = tan(fabs(alphar));
  xiblf = (CAF*talff == 0) ?
    INFINITY :
    Mu*Nf/(2.0*CAF*talff);
  xiblr = (CAR*talfr == 0) ?
    INFINITY :
    Mu*Nr/(2.0*CAR*talfr);
  fyf = (xiblf >= 1.0) ?
    CAF*talff :
    Mu*Nf*(1.0-0.5*xiblf);
  fyr = (xiblr >= 1.0) ?
    CAR*talfr :
    Mu*Nr*(1.0-0.5*xiblr);
  fyf = (alphaf > 0) ? -1.0*fabs(fyf) : fabs(fyf);
  fyr = (alphar > 0) ? -1.0*fabs(fyr) : fabs(fyr);

  //cout << "talff: " << talff << "  xiblf: " << xiblf << " fyf: " << fyf << "\n";

  /* Transfer the velocity */
  dx[0] = -Speed * r  + (fyf + fyr) / Mass;
  dx[1] = (fyf * Adist - fyr * Bdist) / Izz;
  dx[2] = Speed * cos(psi) - v * sin(psi);
  dx[3] = Speed * sin(psi) + v * cos(psi);
  dx[4] = r;

  //cout << "x: " << x << "  Dx: " << dx << "  u: " << u[0] << "\n";

  return dx;
}







// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidLander
//
// *********************************************************************
// *********************************************************************

// Constructor
Model2DRigidLander::Model2DRigidLander(string path = ""):Model2DRigid(path) {
  MSLVector v;

  StateDim = 4;
  InputDim = 1;
  Mass = 1.0;
  G = 1.568;  // Accel of gravity on moon (use 9.8 for earth)
  Fs = 10.0;
  Fu = 20.0;

  LowerState = MSLVector(4);
  UpperState = MSLVector(4);

  LowerState[0] = 0.0; LowerState[1] = 0.0; LowerState[2] = -10.0;
  LowerState[3] = -10.0;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 100.0; UpperState[1] = 100.0; UpperState[2] = 10.0;
  UpperState[3] = 10.0;
  READ_OPTIONAL_PARAMETER(UpperState);

  // Make the list of 1D Inputs
  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  v = MSLVector(1); v[0] = 0;
  Inputs.push_back(v);
  v = MSLVector(1); v[0] = 1;
  Inputs.push_back(v);
  v = MSLVector(1); v[0] = 2;
  Inputs.push_back(v);
  v = MSLVector(1); v[0] = 3;
  Inputs.push_back(v);

  READ_OPTIONAL_PARAMETER(Inputs);
}



MSLVector Model2DRigidLander::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return RungeKuttaIntegrate(x,u,h);
}



MSLVector Model2DRigidLander::StateToConfiguration(const MSLVector &x) {
  return MSLVector(x[0],x[1],0.0); // Yield a zero rotation every time
}


MSLVector Model2DRigidLander::StateTransitionEquation(const MSLVector &x, const MSLVector &u)
{
  MSLVector dx(4);

  /* Transfer the velocity */
  dx[0] = x[2];
  dx[1] = x[3];
  dx[2] = 0.0;
  if (u[0] == 1)
    dx[2] = Fs;
  if (u[0] == 3)
    dx[2] = -Fs;
  dx[3] = -Mass*G;
  if (u[0] == 2)
    dx[3] += Fu;

  //cout << "x: " << x << "  Dx: " << dx << "  u: " << u[0] << "\n";

  return dx;
}





double Model2DRigidLander::Metric(const MSLVector &x1, const MSLVector &x2) {
  double d = 0.0;
  int i;

  for (i = 0; i < 4; i++) {
    d += sqrt(sqr(x1[i] - x2[i]) / (UpperState[i] - LowerState[i]));
  }

  //cout << "x1: " << x1 << "  x2: " << x2 << "   Metric: " << d << "\n";

  return d;
}




// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidMulti
//
// *********************************************************************
// *********************************************************************


// Constructor
Model2DRigidMulti::Model2DRigidMulti(string path = ""):Model2DRigid(path) {
  MSLVector u;
  int i,j;

  READ_PARAMETER_OR_ERROR(NumBodies);

  StateDim = 3*NumBodies;
  InputDim = 3*NumBodies;

  READ_PARAMETER_OR_DEFAULT(LowerState,MSLVector(StateDim));
  READ_PARAMETER_OR_DEFAULT(UpperState,MSLVector(StateDim));

  u = MSLVector(StateDim);
  Inputs.clear();
  for (i = 0; i < StateDim; i++) {
    for (j = 0; j < StateDim; j++)
      u[j] = (i==j) ? 1.0 : 0.0;
    Inputs.push_back(u);
    for (j = 0; j < StateDim; j++)
      u[j] = (i==j) ? -1.0 : 0.0;
    Inputs.push_back(u);
  }
}




double Model2DRigidMulti::Metric(const MSLVector &x1, const MSLVector &x2) {

  double d,fd,dtheta;
  int i;

  d = 0.0;

  for (i = 0; i < NumBodies; i++) {
    fd = fabs(x1[3*i+2]-x2[3*i+2]);
    dtheta = min(fd,2.0*PI - fd);
    d += sqr(x1[3*i] - x2[3*i]);
    d += sqr(x1[3*i+1] - x2[3*i+1]);
    d += sqr(dtheta);
  }

  return sqrt(d);
}



MSLVector Model2DRigidMulti::LinearInterpolate(const MSLVector &x1, const MSLVector &x2, const double &a){
  MSLVector v;
  int i;

  v = (1.0-a)*x1 + a*x2;

  for (i = 0; i < NumBodies; i++) {

    if (fabs(x2[3*i+2] - x1[3*i+2]) > PI) {
      if (x1[3*i+2] > x2[3*i+2])
	v[3*i+2] = (1.0-a)*x1[3*i+2] + a*(x2[3*i+2]+2.0*PI);
      else
	v[3*i+2] = (1.0-a)*(x1[3*i+2]+2.0*PI) + a*x2[3*i+2];
    }

    if (v[3*i+2] > PI)
      v[3*i+2] -= 2.0*PI;

  }

  return v;

}



MSLVector Model2DRigidMulti::StateToConfiguration(const MSLVector &x)
{
  return x;
}




// *********************************************************************
// *********************************************************************
// CLASS:     Model2DRigidChain
//
// *********************************************************************
// *********************************************************************

Model2DRigidChain::Model2DRigidChain(string path = ""):Model2DRigid(path) {

  int i;

  READ_PARAMETER_OR_ERROR(NumBodies);

  StopAngle = PI/1.5;

  StateDim = NumBodies+2;
  InputDim = NumBodies+2;

  READ_PARAMETER_OR_ERROR(A);

  LowerState = MSLVector(NumBodies+2);
  UpperState = MSLVector(NumBodies+2);

  LowerState[0] = 0.0;
  LowerState[1] = 0.0;
  for (i = 0; i < NumBodies; i++) {
    LowerState[i+2] = -StopAngle;
  }
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 100.0;
  UpperState[1] = 100.0;
  for (i = 0; i < NumBodies; i++) {
    UpperState[i+2] = StopAngle;
  }
  READ_OPTIONAL_PARAMETER(LowerState);


  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  // NEED A DEFAULT HERE!!!!

  READ_OPTIONAL_PARAMETER(Inputs);

}



MSLVector Model2DRigidChain::LinearInterpolate(const MSLVector &x1,
					    const MSLVector &x2,
					    const double &a) {
  return (1.0-a)*x1 + a*x2;
}



MSLVector Model2DRigidChain::StateToConfiguration(const MSLVector &x) {
  MSLVector q;
  int i;
  double lx,ly,ltheta;

  q = MSLVector(3*NumBodies);
  q[0] = x[0];
  q[1] = x[1];
  q[2] = x[2];

  for (i = 1; i < NumBodies; i++) {
    lx = q[3*(i-1)];
    ly = q[3*(i-1)+1];
    ltheta = q[3*(i-1)+2];

    q[3*i] = cos(ltheta)*A[i-1]+lx;
    q[3*i+1] = sin(ltheta)*A[i-1]+ly;
    q[3*i+2] = atan2(cos(ltheta)*sin(x[2+i])+sin(ltheta)*cos(x[2+i]),
		     cos(ltheta)*cos(x[2+i])-sin(ltheta)*sin(x[2+i]));
    if (q[3*i+2] < 0.0)
      q[3*i+2] += 2.0*PI; // Force the orientation into [0,2pi)
  }

  return q;
}




MSLVector Model2DRigidChain::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(StateDim);

  dx = u;
  return dx;
}



double Model2DRigidChain::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho;
  MSLVector dtheta(StateDim);
  int i;

  rho = sqr(x1[0] - x2[0]) + sqr(x1[1] - x2[1]);

  for (i = 2; i < StateDim; i++) {
    dtheta[i] = min(fabs(x1[i]-x2[i]),2.0*PI - fabs(x1[i]-x2[i]));
    rho += sqr(50.0/PI*dtheta[i]);
  }

  rho = sqrt(rho);

  return rho;
}


// Make sure the joint position and limits are respected
bool Model2DRigidChain::Satisfied(const MSLVector &x)
{
  int i;

  for (i = 0; i < StateDim; i++)
    if ((x[i] > UpperState[i]) || (x[i] < LowerState[i]))
      return false;

  return true;
}
