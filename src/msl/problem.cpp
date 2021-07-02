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

#include "msl/problem.h"
#include "msl/defs.h"

// Constructor
Problem::Problem(Geom *geom, Model *model, string path = "") {

  SetGeom(geom);
  SetModel(model);

  if ((path.length() > 0)&&(path[path.length()-1] != '/'))
    path += "/";

  FilePath = path;

  READ_PARAMETER_OR_DEFAULT(InitialState,M->LowerState + \
			    0.5*(M->UpperState - M->LowerState));

  READ_PARAMETER_OR_DEFAULT(GoalState,M->LowerState);

  StateDim = M->StateDim;
  InputDim = M->InputDim;
  LowerState = M->LowerState;
  UpperState = M->UpperState;

  NumBodies = G->NumBodies;
  MaxDeviates = G->MaxDeviates;
}


void Problem::SetGeom(Geom *geom) {
  G = geom;
  NumBodies = G->NumBodies;
  MaxDeviates = G->MaxDeviates;
  GeomDim = G->GeomDim;
}


void Problem::SetModel(Model *model) {
  M = model;
  StateDim = M->StateDim;
  InputDim = M->InputDim;
  LowerState = M->LowerState;
  UpperState = M->UpperState;

  READ_PARAMETER_OR_DEFAULT(InitialState,M->LowerState + \
			    0.5*(M->UpperState - M->LowerState));

  READ_PARAMETER_OR_DEFAULT(GoalState,M->LowerState);

}


// In the base class, steal the following methods from Model

list<MSLVector> Problem::GetInputs(const MSLVector &x) {
  return M->GetInputs(x);
}

list<MSLVector> Problem::GetInputs() {
  MSLVector x(StateDim);

  return M->GetInputs(x);
}


MSLVector Problem::InterpolateState(const MSLVector &x1, const MSLVector &x2,
				 const double &a) {
  return M->LinearInterpolate(x1,x2,a);
}

// By default, don't change anything
MSLVector Problem::StateToConfiguration(const MSLVector &x) {
  return M->StateToConfiguration(x);
}

// Default metric: use the metric from the model
double Problem::Metric(const MSLVector &x1, const MSLVector &x2) {
  return M->Metric(x1,x2);
}

MSLVector Problem::StateDifference(const MSLVector &x1,
				const MSLVector &x2) {
  return M->StateDifference(x1,x2);
}

bool Problem::Satisfied(const MSLVector &x) {
  return ((G->CollisionFree(StateToConfiguration(x)))&&
	  (M->Satisfied(x)));
}

MSLVector Problem::Integrate(const MSLVector &x, const MSLVector &u,
			  const double &deltat) {
  return M->Integrate(x,u,deltat);
}

// In the base class, steal the following methods from Geom

bool Problem::CollisionFree(const MSLVector &q) {
  return G->CollisionFree(q);
}


double Problem::DistanceComp(const MSLVector &q) {
  return G->DistanceComp(q);
}


MSLVector Problem::ConfigurationDifference(const MSLVector &q1,
				    const MSLVector &q2) {
  return G->ConfigurationDifference(q1,q2);
}
