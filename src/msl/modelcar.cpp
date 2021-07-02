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

#include <fstream>
#include <math.h>

#include "msl/vector.h"
#include "msl/modelcar.h"
#include "msl/defs.h"

// *********************************************************************
// *********************************************************************
// CLASS:     ModelCar
//
// Just a class wrapper around Model2DRigidCar
// *********************************************************************
// *********************************************************************


// Constructor
ModelCar::ModelCar(string path = ""):Model2DRigidCar(path) {
  Speed = 3.0;
}


MSLVector ModelCar::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  q[0] = x[0];
  q[1] = x[1];
  q[2] = x[2];

  return q;
}

bool ModelCar::Satisfied(const MSLVector &state)
{
  int i;

  for (i = 0; i < StateDim; i++)
    if ((state[i] > UpperState[i]) || (state[i] < LowerState[i]))
      return false;

  return true;
}

// *********************************************************************
// *********************************************************************
// CLASS:     ModelCarSmooth
//
// A class wrapper around Model2DRigidCarSmooth
// *********************************************************************
// *********************************************************************


// Constructor
ModelCarSmooth::ModelCarSmooth(string path = ""):Model2DRigidCarSmooth(path) {
}


MSLVector ModelCarSmooth::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  q[0] = x[0];
  q[1] = x[1];
  q[2] = x[2];

  return q;
}

// *********************************************************************
// *********************************************************************
// CLASS:     ModelCarDyn
//
//  A class wrapper around Model2DRigidDyncar
// *********************************************************************
// *********************************************************************


// Constructor
ModelCarDyn::ModelCarDyn(string path = ""):Model2DRigidDyncar(path) {

  Mass = 1360.0;
  CAF = 61170.0;
  CAR = 76470.0;
  Adist = 1.113;
  Bdist = 1.427;
  Izz = 2500.0;

  Speed = 20.0;
  WorldScale = 1.0;

  LowerState = MSLVector(5);
  UpperState = MSLVector(5);

  LowerState[0] = -50.0; LowerState[1] = -5.0; LowerState[2] = 0.0;
  LowerState[3] = 0.0; LowerState[4] = 0.0;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 50.0; UpperState[1] = 5.0; UpperState[2] = 100.0;
  UpperState[3] = 100.0; UpperState[4] = 2.0*PI;
  READ_OPTIONAL_PARAMETER(UpperState);

}


MSLVector ModelCarDyn::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  q[0] = x[2];
  q[1] = x[3];
  q[2] = x[4];

  return q;
}


double ModelCarDyn::Metric(const MSLVector &x1, const MSLVector &x2) {

  double d;

  // Position difference
  d =  sqr((x1[2] - x2[2]) / (UpperState[2] - LowerState[2])) * 16.0;
  d += sqr((x1[3] - x2[3]) / (UpperState[3] - LowerState[3])) * 16.0;

  // Orientation difference
  d += sqr(min(fabs(x1[4]-x2[4]),2.0*PI - fabs(x1[4]-x2[4]))/2.0/PI) * 16.0;

  // Velocities
  d += sqr((x1[0] - x2[0]) / (UpperState[0] - LowerState[0]));
  d += sqr((x1[1] - x2[1]) / (UpperState[1] - LowerState[1]));


  return sqrt(d);
}

// *********************************************************************
// *********************************************************************
// CLASS:     ModelCarDynNtire
//
//  A class wrapper around Model2DRigidDyncarNtire
// *********************************************************************
// *********************************************************************


// Constructor
ModelCarDynNtire::ModelCarDynNtire(string path = ""):Model2DRigidDyncarNtire(path) {

  Mass = 1360.0;
  CAF = 61170.0;
  CAR = 76470.0;
  Adist = 1.113;
  Bdist = 1.427;
  Izz = 2500.0;

  Mu = 0.6;
  Nf = Mass*9.8*0.55;
  Nr = Mass*9.8*0.45;

  Speed = 15.0;
  WorldScale = 1.0;
}


MSLVector ModelCarDynNtire::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  q[0] = x[2];
  q[1] = x[3];
  q[2] = x[4];

  return q;
}


double ModelCarDynNtire::Metric(const MSLVector &x1, const MSLVector &x2) {

  double d;

  // Position difference
  d =  sqr((x1[2] - x2[2]) / (UpperState[2] - LowerState[2])) * 16.0;
  d += sqr((x1[3] - x2[3]) / (UpperState[3] - LowerState[3])) * 16.0;

  // Orientation difference
  d += sqr(min(fabs(x1[4]-x2[4]),2.0*PI - fabs(x1[4]-x2[4]))/2.0/PI) * 16.0;

  // Velocities
  d += sqr((x1[0] - x2[0]) / (UpperState[0] - LowerState[0]));
  d += sqr((x1[1] - x2[1]) / (UpperState[1] - LowerState[1]));


  return sqrt(d);
}

// *********************************************************************
// *********************************************************************
// CLASS:     ModelCarDynRollover
//
//  A car model considering the rolling effect
// *********************************************************************
// *********************************************************************


// Constructor
ModelCarDynRollover::ModelCarDynRollover(string path = ""):ModelCarDynNtire(path) {

  MSLVector v;

  StateDim = 12;
  InputDim = 2;

  CAR = 76470;
  CAF = 61170;

  Mass = 1360;
  Adist = 1.113;
  Bdist = 1.427;
  Izz = 0.25*Mass*(Adist+Bdist)*(Adist+Bdist);

  T = 1.4;
  H2 = 0.44;
  H = 0.14;
  Fai = 6.15*3.1415926/180.0;
  Wn = 1.75;
  x = Bdist/(Adist + Bdist);

  Speed = 15.0;

  IsRollOver = false;

  K = Mass*9.8*H2*(1+Fai)/Fai;
  Ixx = (K-Mass*9.8*H2)/sqr(Wn*2*3.1415926);
  c = Wn*2*sqrt((K-Mass*9.8*H2)*Ixx);

  Mu = 0.6;

  LowerState = MSLVector(StateDim);
  UpperState = MSLVector(StateDim);

  LowerState[0] = -100.0; LowerState[1] = -5.0; LowerState[2] = 0.0;
  LowerState[3] = -1000.0; LowerState[4] = 0.0;LowerState[5] = -30.0*PI/180.0;
  LowerState[6] = LowerState[5]*3.0; LowerState[7] = 0.0;
  LowerState[8] = 0.0; LowerState[9] = 0.0;
  LowerState[10] = 0.0; LowerState[11] = 0.0;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 50.0; UpperState[1] = 5.0; UpperState[2] = 1000.0;
  UpperState[3] = 0.0; UpperState[4] = 2.0*PI;UpperState[5] = 30.0*PI/180.0;
  UpperState[6] = UpperState[5]*3.0; UpperState[7] = 160.0;
  UpperState[8] = Mass * 9.8; UpperState[9] = Mass * 9.8;
  UpperState[10] = Mass * 9.8; UpperState[11] = Mass * 9.8;
  READ_OPTIONAL_PARAMETER(UpperState);

  MaxSteeringAngle = 0.6;

  v = MSLVector(2);
  v[0] = -0.6-MaxSteeringAngle/6.0;
  v[1] = 0.0;
  for(int i = 1; i <= 13; i++) {
    v[0] = v[0] + MaxSteeringAngle/6.0;
    Inputs.push_back(v);
  }

  READ_OPTIONAL_PARAMETER(Inputs);

}


MSLVector ModelCarDynRollover::StateTransitionEquation(const MSLVector &x1,
						       const MSLVector &u)
{
  MSLVector dx(StateDim);

  double alphaf,alphar,fyf,fyr,v,r,psi,fyfl,fyrl,fyfr,fyrr;
  double talff,talfr,xiblfl,xiblrl,xiblfr,xiblrr;
  double roll, rollrate;
  double rollaccel;
  double deltan, nf, nr;
  double Nfl, Nfr, Nrl, Nrr;

  Nfl = x1[8];  Nfr = x1[9];
  Nrl = x1[10];  Nrr = x1[11];

  v = x1[0]; r = x1[1]; psi = x1[4];
  roll = x1[5]; rollrate = x1[6]; Speed = x1[7];

  alphaf = atan((v + Adist * r) / Speed) - u[0];
  alphar = atan((v - Bdist * r) / Speed);

  talff = tan(fabs(alphaf));
  talfr = tan(fabs(alphar));
  xiblfl = (CAF*talff == 0) ?
    INFINITY :
    Mu*Nfl/(CAF*talff/2.0);
  xiblrl = (CAR*talfr == 0) ?
    INFINITY :
    Mu*Nrl/(CAR*talfr/2.0);
  xiblfr = (CAF*talff == 0) ?
    INFINITY :
    Mu*Nfr/(CAF*talff/2.0);
  xiblrr = (CAR*talfr == 0) ?
    INFINITY :
    Mu*Nrr/(CAR*talfr/2.0);
  fyfl = (xiblfl >= 1.0) ?
    -1.0*CAF*alphaf/2.0:
    -Mu*Nfl*sgn(alphaf)*(1.0-0.5*xiblfl);
  fyrl = (xiblrl >= 1.0) ?
    -1*CAR*alphar/2.0:
    -Mu*Nrl*sgn(alphar)*(1.0-0.5*xiblrl);
  fyfr = (xiblfr >= 1.0) ?
    -1.0*CAF*alphaf/2.0:
    -Mu*Nfr*sgn(alphaf)*(1.0-0.5*xiblfr);
  fyrr = (xiblrr >= 1.0) ?
    -1*CAR*alphar/2.0:
    -Mu*Nrr*sgn(alphar)*(1.0-0.5*xiblrr);

  fyf = fyfl + fyfr;
  fyr = fyrl + fyrr;

  rollaccel = (-(fyf+fyr)*H2-(K-Mass*9.8*H2)*roll-c*rollrate)/Ixx;

  deltan = (-K*roll-c*rollrate+(fyf+fyr)*H)*2/T;
  nf = Mass * 9.8 * x;
  nr = Mass * 9.8 * (1-x);

  dx[8] = nf/2.0 + deltan*x/2.0;
  dx[9] = nf/2.0 - deltan*x/2.0;
  dx[10] = nr/2.0 + deltan*(1-x)/2.0;
  dx[11] = nr/2.0 - deltan*(1-x)/2.0;

  /* Transfer the velocity */
  dx[0] = -Speed * r  + (fyf + fyr) / Mass - rollaccel*H2;
  dx[1] = (fyf * Adist - fyr * Bdist) / Izz;
  dx[2] = Speed * cos(psi) - v * sin(psi);
  dx[3] = Speed * sin(psi) + v * cos(psi);
  dx[4] = r;
  dx[5] = rollrate;
  dx[6] = rollaccel;
  dx[7] = u[1];

  return dx;
}


//!!!!!!!!! It has both the state and the uncontrolled state
MSLVector ModelCarDynRollover::Integrate(const MSLVector &x, const MSLVector &u,
					 const double &h)
{
  int s,i,k;
  double c;
  MSLVector nx, x1;

  s = (h > 0) ? 1 : -1;

  c = s*h/ModelDeltaT;  // Number of iterations (as a double)
  k = (int) c;

  nx = x;
  for (i = 0; i < k; i++) {
    x1 = StateTransitionEquation(nx,u);
    nx += s * ModelDeltaT * x1;

    nx[8] = x1[8];    nx[9] = x1[9];
    nx[10] = x1[10];  nx[11] = x1[11];
  }

  // Integrate the last step for the remaining time
  x1 = StateTransitionEquation(nx,u);
  nx += s * (c - k) * ModelDeltaT * x1;
  nx[8] = x1[8];    nx[9] = x1[9];
  nx[10] = x1[10];  nx[11] = x1[11];

  return nx;
}

MSLVector ModelCarDynRollover::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  q[0] = x[2];
  q[1] = x[3];
  q[2] = x[4];

  return q;
}


double ModelCarDynRollover::Metric(const MSLVector &x1, const MSLVector &x2) {

  double d;

  // Position difference
  d =  sqr((x1[2] - x2[2]) / (UpperState[2] - LowerState[2])) * 25.0;
  d += sqr((x1[3] - x2[3]) / (UpperState[3] - LowerState[3])) * 25.0;

  // Orientation difference
  d += sqr(min(fabs(x1[4]-x2[4]),2.0*PI - fabs(x1[4]-x2[4]))/2.0/PI) * 36.0;

  // Velocities
  d += sqr((x1[0] - x2[0]) / (UpperState[0] - LowerState[0]));
  d += sqr((x1[1] - x2[1]) / (UpperState[1] - LowerState[1]));

  d += sqr((x1[5] - x2[5]) / (UpperState[5] - LowerState[5]));
  d += sqr((x1[6] - x2[6]) / (UpperState[6] - LowerState[6]));
  d += sqr((x1[7] - x2[7]) / (UpperState[7] - LowerState[7]));

  return sqrt(d);
}


bool ModelCarDynRollover::RollOverFree(const MSLVector &x)
{
  return !(x[8]<=0.0 || x[9]<=0.0 || x[10]<=0.0 || x[11]<=0.0);
}

bool ModelCarDynRollover::Satisfied(const MSLVector &state)
{
  int i;
  bool tb = true;

  if(RollOverFree(state)) {
    for(i=0; i<StateDim; i++)
      if(state[i]>UpperState[i] || state[i]<LowerState[i]) {
	tb = false;
      }
  }
  else {
    tb = false;
  }

  return tb;
}


int ModelCarDynRollover::sgn(double x)
{
  int m;

  if(x>0.0) m = 1;
  else
    if(x==0.0) m = 0;
    else m = -1;

  return m;
}





// *********************************************************************
// *********************************************************************
// CLASS:     ModelCarDynSmoothRollover
//
// A simple car to be used in the town
// use a 2d rigid car to simulate the 3d model to drive in the town
// *********************************************************************
// *********************************************************************


// Constructor
ModelCarDynSmoothRollover::ModelCarDynSmoothRollover(string path = ""):ModelCarDynRollover(path) {

  MSLVector v;

  StateDim = 13;

  Speed = 10.0;

  LowerState = MSLVector(StateDim);
  UpperState = MSLVector(StateDim);

  LowerState[0] = -100.0; LowerState[1] = -5.0; LowerState[2] = 0.0;
  LowerState[3] = -1000.0; LowerState[4] = 0.0;LowerState[5] = -30.0*PI/180.0;
  LowerState[6] = LowerState[5]*3.0; LowerState[7] = 0.0; LowerState[8] = -0.6;
  LowerState[9] = 0.0; LowerState[10] = 0.0;
  LowerState[11] = 0.0; LowerState[12] = 0.0;
  READ_OPTIONAL_PARAMETER(LowerState);

  UpperState[0] = 50.0; UpperState[1] = 5.0; UpperState[2] = 1000.0;
  UpperState[3] = 0.0; UpperState[4] = 2.0*PI;UpperState[5] = 30.0*PI/180.0;
  UpperState[6] = UpperState[5]*3.0; UpperState[7] = 160.0; UpperState[8] = 0.6;
  UpperState[9] = Mass * 9.8; UpperState[10] = Mass * 9.8;
  UpperState[11] = Mass * 9.8; UpperState[12] = Mass * 9.8;
  READ_OPTIONAL_PARAMETER(UpperState);

  MaxSteeringAngle = 0.6;
  Inputs.clear();

  v = MSLVector(2);
  v[0] = -0.6-MaxSteeringAngle/6.0;
  v[1] = 0.0;
  for(int i = 1; i <= 13; i++) {
    v[0] = v[0] + MaxSteeringAngle/6.0;
    Inputs.push_back(v);
  }

  READ_OPTIONAL_PARAMETER(Inputs);

}


MSLVector ModelCarDynSmoothRollover::StateTransitionEquation(const MSLVector &x1,
							     const MSLVector &u)
{
  MSLVector dx(StateDim);
  double alphaf,alphar,fyf,fyr;
  double v,r,psi,fyfl,fyrl,fyfr,fyrr;
  double talff,talfr,xiblfl,xiblrl;
  double xiblfr,xiblrr,deltan,nf,nr;
  double roll, rollrate;
  double rollaccel, beta;
  double Nfl, Nfr, Nrl, Nrr;

  Nfl = x1[9];  Nfr = x1[10];
  Nrl = x1[11];  Nrr = x1[12];

  v = x1[0]; r = x1[1]; psi = x1[4];
  roll = x1[5]; rollrate = x1[6];
  Speed = x1[7]; beta = x1[8];

  alphaf = atan((v + Adist * r) / Speed) - beta;
  alphar = atan((v - Bdist * r) / Speed);

  talff = tan(fabs(alphaf));
  talfr = tan(fabs(alphar));
  xiblfl = (CAF*talff == 0) ?
    INFINITY :
    Mu*Nfl/(CAF*talff/2.0);
  xiblrl = (CAR*talfr == 0) ?
    INFINITY :
    Mu*Nrl/(CAR*talfr/2.0);
  xiblfr = (CAF*talff == 0) ?
    INFINITY :
    Mu*Nfr/(CAF*talff/2.0);
  xiblrr = (CAR*talfr == 0) ?
    INFINITY :
    Mu*Nrr/(CAR*talfr/2.0);
  fyfl = (xiblfl >= 1.0) ?
    -1.0*CAF*alphaf/2.0:
    -Mu*Nfl*sgn(alphaf)*(1.0-0.5*xiblfl);
  fyrl = (xiblrl >= 1.0) ?
    -1*CAR*alphar/2.0:
    -Mu*Nrl*sgn(alphar)*(1.0-0.5*xiblrl);
  fyfr = (xiblfr >= 1.0) ?
    -1.0*CAF*alphaf/2.0:
    -Mu*Nfr*sgn(alphaf)*(1.0-0.5*xiblfr);
  fyrr = (xiblrr >= 1.0) ?
    -1*CAR*alphar/2.0:
    -Mu*Nrr*sgn(alphar)*(1.0-0.5*xiblrr);

  fyf = fyfl + fyfr;
  fyr = fyrl + fyrr;

  deltan = (-K*roll-c*rollrate+(fyf+fyr)*H)*2/T;
  nf = Mass * 9.8 * x;
  nr = Mass * 9.8 * (1-x);

  dx[9] = nf/2.0 + deltan*x/2.0;
  dx[10] = nf/2.0 - deltan*x/2.0;
  dx[11] = nr/2.0 + deltan*(1-x)/2.0;
  dx[12] = nr/2.0 - deltan*(1-x)/2.0;

  rollaccel = (-(fyf+fyr)*H2-(K-Mass*9.8*H2)*roll-c*rollrate)/Ixx;

  dx[0] = -Speed * r  + (fyf + fyr) / Mass - rollaccel*H2;
  dx[1] = (fyf * Adist - fyr * Bdist) / Izz;
  dx[2] = Speed * cos(psi) - v * sin(psi);
  dx[3] = Speed * sin(psi) + v * cos(psi);
  dx[4] = r;
  dx[5] = rollrate;
  dx[6] = rollaccel;
  dx[7] = u[1];
  dx[8] = u[0];

  return dx;
}


MSLVector ModelCarDynSmoothRollover::StateToConfiguration(const MSLVector &x)
{
  MSLVector q(3);

  // adjust the value when the obstacles are not about the origin.
  q[0] = x[2]; // + 2390.0;
  q[1] = x[3]; // + 2300.0;
  q[2] = x[4];

  return q;
}


double ModelCarDynSmoothRollover::Metric(const MSLVector &x1, const MSLVector &x2) {

  double d;

  // Position difference
  d =  sqr((x1[2] - x2[2]) / (UpperState[2] - LowerState[2])); // * 25.0;
  d += sqr((x1[3] - x2[3]) / (UpperState[3] - LowerState[3])); // * 25.0;

  // Orientation difference
  d += sqr(min(fabs(x1[4]-x2[4]),2.0*PI - fabs(x1[4]-x2[4]))/2.0/PI);// * 36.0;

  // Velocities
  d += sqr((x1[0] - x2[0]) / (UpperState[0] - LowerState[0]));
  d += sqr((x1[1] - x2[1]) / (UpperState[1] - LowerState[1]));

  d += sqr((x1[5] - x2[5]) / (UpperState[5] - LowerState[5]));
  d += sqr((x1[6] - x2[6]) / (UpperState[6] - LowerState[6]));
  d += sqr((x1[7] - x2[7]) / (UpperState[7] - LowerState[7]));
  d += sqr((x1[8] - x2[8]) / (UpperState[8] - LowerState[8]));

  return sqrt(d);
}



MSLVector ModelCarDynSmoothRollover::LinearInterpolate(const MSLVector &x1,
						       const MSLVector &x2,
						       const double &a)
{

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
