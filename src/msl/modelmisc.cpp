//----------------------------------------------------------------------
//               The Motion Strategy Library (MSL)
//----------------------------------------------------------------------
//
// Copyright (c) University of Illinois and Steve LaValle.
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

#include "msl/modelmisc.h"
#include "msl/defs.h"


// *********************************************************************
// *********************************************************************
// CLASS:     Model1D
//
// *********************************************************************
// *********************************************************************

Model1D::Model1D(string path = ""):Model(path) {

  MSLVector v;

  Force = 1.0;

  StateDim = 1;
  InputDim = 1;


  READ_PARAMETER_OR_DEFAULT(LowerState,MSLVector(-50.0,-10.0));
  READ_PARAMETER_OR_DEFAULT(UpperState,MSLVector(50.0,-10.0));

  // Make the list of 1D Inputs
  Inputs.clear();  // Just to be safe
  v = MSLVector(1); v[0] = 0;  // Float
  Inputs.push_back(v);
  v = MSLVector(1); v[0] = 1;  // Accel in +x
  Inputs.push_back(v);
  v = MSLVector(1); v[0] = -1; // Accel in -x
  Inputs.push_back(v);

  READ_OPTIONAL_PARAMETER(Inputs);

}



MSLVector Model1D::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return RungeKuttaIntegrate(x,u,h);
}


MSLVector Model1D::StateTransitionEquation(const MSLVector &x, const MSLVector &u)
{
  MSLVector dx(2);

  dx[0] = x[1];
  dx[1] = u[0]*Force;

  return dx;
}


MSLVector Model1D::StateToConfiguration(const MSLVector &x) {
  return MSLVector(x[0]+50,x[1]*(100.0/(UpperState[1]-LowerState[1]))+50.0,0.0); // Yield a zero rotation every time
}

double Model1D::Metric(const MSLVector &x1, const MSLVector &x2) {
  double d = 0.0;
  int i;

  for (i = 0; i < 2; i++) {
    d += sqr(x1[i] - x2[i]) / (UpperState[i] - LowerState[i]);
  }

  //cout << "x1: " << x1 << "  x2: " << x2 << "   Metric: " << d << "\n";

  return d;
}



// *********************************************************************
// *********************************************************************
// CLASS:     ModelLinear
//
// *********************************************************************
// *********************************************************************

ModelLinear::ModelLinear(string path = ""):Model(path) {

  MSLVector v;
  int i;

  READ_PARAMETER_OR_ERROR(A);
  READ_PARAMETER_OR_ERROR(B);

  StateDim = A.dim1();
  InputDim = B.dim2();

  LowerState = MSLVector(StateDim);
  for (i = 0; i < StateDim; i++)
    LowerState[i] = -10.0;
  READ_OPTIONAL_PARAMETER(LowerState);


  UpperState = MSLVector(StateDim);
  for (i = 0; i < StateDim; i++)
    UpperState[i] = 10.0;
  READ_OPTIONAL_PARAMETER(UpperState);

  Inputs.clear();  // Just to be safe
  v = MSLVector(InputDim);  // Initializes to the zero MSLVector
  Inputs.push_front(v);
  for (i = 0; i < StateDim; i++) {
    v = MSLVector(InputDim);
    v[i] = 1.0;
    Inputs.push_front(v);
    v[i] = -1.0;
    Inputs.push_front(v);
  }
  READ_OPTIONAL_PARAMETER(Inputs);

}


MSLVector ModelLinear::StateToConfiguration(const MSLVector &x) {
  return MSLVector(5.0*(x[0]+10.0),5.0*(x[1]+10.0),0.0);
}


MSLVector ModelLinear::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return RungeKuttaIntegrate(x,u,h);
}


MSLVector ModelLinear::StateTransitionEquation(const MSLVector &x, const MSLVector &u)
{
  MSLVector dx(StateDim);

  dx = A * x + B * u; // This looks too easy!

  return dx;
}



// *********************************************************************
// *********************************************************************
// CLASS:     ModelND
//
// *********************************************************************
// *********************************************************************

ModelND::ModelND(string path = ""):Model(path) {

  MSLVector v;
  int i;

  if ((path.length() > 0)&&(path[path.length()-1] != '/'))
    path += "/";

  FilePath = path;

  READ_PARAMETER_OR_DEFAULT(StateDim,2);
  READ_PARAMETER_OR_DEFAULT(CorridorWidth,6.0*StateDim);

  LowerState = MSLVector(StateDim);
  for (i = 0; i < StateDim; i++)
    LowerState[i] = 0.0;
  READ_OPTIONAL_PARAMETER(LowerState);


  UpperState = MSLVector(StateDim);
  for (i = 0; i < StateDim; i++)
    UpperState[i] = 100.0;
  READ_OPTIONAL_PARAMETER(UpperState);


  Inputs.clear();  // Just to be safe
  v = MSLVector(InputDim);  // Initializes to the zero MSLVector
  Inputs.push_front(v);
  for (i = 0; i < StateDim; i++) {
    v = MSLVector(InputDim);
    v[i] = 1.0;
    Inputs.push_front(v);
    v[i] = -1.0;
    Inputs.push_front(v);
  }

  READ_OPTIONAL_PARAMETER(Inputs);
}



MSLVector ModelND::StateToConfiguration(const MSLVector &x) {
  return MSLVector(x[0],x[1],0.0);
}


MSLVector ModelND::StateTransitionEquation(const MSLVector &x, const MSLVector &u)
{
  MSLVector dx(StateDim);

  dx = u;

  return dx;
}


MSLVector ModelND::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return EulerIntegrate(x,u,h);
}


// This makes a narrow circular corridor
/*
bool ModelND::Satisfied(const MSLVector &x) {
  bool sat;
  MSLVector sx,cx;  // cx is the center
  int i;

  sx = MSLVector(x.dim()-1);  // a MSLVector of the last n-1 components
  cx = MSLVector(x.dim()-1);
  for (i = 0; i < sx.dim(); i++) {
    sx[i] = x[i+1];
    cx[i] = (UpperState[i+1] - LowerState[i+1])/2.0;
  }
  sat = ((x[0] < 30.0) || (x[0] > 70.0) ||
	 ((cx - sx).length() < CorridorWidth));

  return sat;
}
*/

// This makes a narrow square corridor
/*
bool ModelND::Satisfied(const MSLVector &x) {
  int i;
  double maxdiff;

  if ((x[0] < 30)||(x[0] > 70.0))
    return true;

  maxdiff = 0.0;
  for (i = 1; i < x.dim(); i++) {
    if (fabs(x[i]-50.0) > maxdiff)
      maxdiff = fabs(x[i]-50.0);
  }

  return (maxdiff < CorridorWidth);
}
*/

/*
// This makes a narrow square corridor with two bends
bool ModelND::Satisfied(const MSLVector &x) {
  int i;

  if ((x[0] < 30)||(x[0] > 70.0))
    return true;

  // Check the proper slice for last n-2 coords
  if (x.dim() > 2) {
    for (i = 2; i < x.dim(); i++) {
      if (fabs(x[i]-50.0) > CorridorWidth)
	return false;
    }
  }

  // Check upper left corridor piece
  if ((x[0] < 50) && (fabs(x[1]-70.0) < CorridorWidth))
    return true;

  // Check lower right corridor piece
  if ((x[0] >= 50)&&(fabs(x[1]-30.0) < CorridorWidth))
    return true;

  // Final check for center piece
  if ((x[0] < 50.0 + CorridorWidth)&&
      (x[0] > 50.0 - CorridorWidth)&&
      (x[1] < 70.0 + CorridorWidth)&&
      (x[1] > 30.0 - CorridorWidth))
    return true;

  // Finally, the point is in collision
  return false;
}

*/

// *********************************************************************
// *********************************************************************
// CLASS:     ModelNintegrator
//
// *********************************************************************
// *********************************************************************


ModelNintegrator::ModelNintegrator(string path = ""):Model(path) {

  MSLVector v;
  int i;
  double alpha,beta;

  UBound = 0.5;
  VBound = 0.5;

  StateDim = 3;
  InputDim = 2;

  LowerState = MSLVector(StateDim);
  for (i = 0; i < StateDim; i++)
    LowerState[i] = -20.0;
  READ_OPTIONAL_PARAMETER(LowerState);


  UpperState = MSLVector(StateDim);
  for (i = 0; i < StateDim; i++)
    UpperState[i] = 20.0;
  READ_OPTIONAL_PARAMETER(UpperState);

  Inputs.clear();  // Just to be safe
  for (alpha = -UBound; alpha <= UBound;
       alpha += 2.0*UBound/4.0) {
    for (beta = -VBound; beta <= VBound;
	 beta += 2.0*VBound/4.0) {
      Inputs.push_back(MSLVector(alpha,beta));
      Inputs.push_back(MSLVector(alpha,beta));
    }
  }
  READ_OPTIONAL_PARAMETER(Inputs);

}


MSLVector ModelNintegrator::StateToConfiguration(const MSLVector &x) {
  return MSLVector(2.5*(x[0]+20.0),2.5*(x[1]+20.0),0.0);
}


MSLVector ModelNintegrator::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return RungeKuttaIntegrate(x,u,h);
}


MSLVector ModelNintegrator::StateTransitionEquation(const MSLVector &x, const MSLVector &u)
{
  MSLVector dx(StateDim);

  dx[0] = u[0];
  dx[1] = u[1];
  dx[2] = x[0]*u[1] - x[1]*u[0];

  return dx;
}
