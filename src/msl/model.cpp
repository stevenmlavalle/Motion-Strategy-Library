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

#include "msl/model.h"
#include "msl/defs.h"

Model::Model(string path) {

  if ((path.length() > 0)&&(path[path.length()-1] != '/'))
    path += "/";

  FilePath = path;

  READ_PARAMETER_OR_DEFAULT(ModelDeltaT,1.0);

  StateDim = 2;
  InputDim = 2;

}



MSLVector Model::EulerIntegrate(const MSLVector &x, const MSLVector &u,
		  const double &h)
{
  int s,i,k;
  double c;
  MSLVector nx;

  s = (h > 0) ? 1 : -1;

  c = s*h/ModelDeltaT;  // Number of iterations (as a double)
  k = (int) c;

  nx = x;
  for (i = 0; i < k; i++) {
    nx += s * ModelDeltaT * StateTransitionEquation(nx,u);
  }

  // Integrate the last step for the remaining time
  nx += s * (c - k) * ModelDeltaT * StateTransitionEquation(nx,u);

  return nx;
}



MSLVector Model::RungeKuttaIntegrate(const MSLVector &x, const MSLVector &u,
		  const double &h)
{
  MSLVector k1,k2,k3,k4;
  int s,i,k;
  double c,deltat;
  MSLVector nx;

  s = (h > 0) ? 1 : -1;

  c = s*h/ModelDeltaT;  // Number of iterations (as a double)
  k = (int) c;
  deltat = s * ModelDeltaT;

  nx = x;
  for (i = 0; i < k; i++) {
    k1 = StateTransitionEquation(nx,u);
    k2 = StateTransitionEquation(nx + (0.5*deltat)*k1,u);
    k3 = StateTransitionEquation(nx + (0.5*deltat)*k2,u);
    k4 = StateTransitionEquation(nx + deltat*k3,u);
    nx += deltat / 6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4);
  }

  // Integrate the last step for the remaining time
  deltat = s * (c - k) * ModelDeltaT;
  k1 = StateTransitionEquation(nx,u);
  k2 = StateTransitionEquation(nx + (0.5*deltat)*k1,u);
  k3 = StateTransitionEquation(nx + (0.5*deltat)*k2,u);
  k4 = StateTransitionEquation(nx + deltat*k3,u);
  nx += deltat / 6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4);

  return nx;
}



list<MSLVector> Model::GetInputs(const MSLVector &x) {
  return Inputs;
}



// By default, don't change anything
MSLVector Model::StateToConfiguration(const MSLVector &x) {
  return x;
}


bool Model::Satisfied(const MSLVector &x) {
  return true;
}


// Default is to use the standard Euclidean metric
double Model::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho;

  rho = (x1 - x2).length();

  return rho;
}


// Some models will interpolate differently because of
// topology (e.g., S^1, P^3)
MSLVector Model::LinearInterpolate(const MSLVector &x1, const MSLVector &x2,
				const double &a) {
  return (1.0-a)*x1 + a*x2;
}


// Ignore topology in the base class
MSLVector Model::StateDifference(const MSLVector &x1, const MSLVector &x2) {
  return (x2 - x1);
}
