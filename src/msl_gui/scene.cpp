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

#include "msl/defs.h"

#include "msl_gui/scene.h"

// Constructor
Scene::Scene(Problem *problem, string path = "") {

  SetProblem(problem);

  if ((path.length() > 0)&&(path[path.length()-1] != '/'))
    path += "/";

  FilePath = path;

  NumBodies = P->NumBodies;

  READ_PARAMETER_OR_DEFAULT(LowerWorld,MSLVector(-50.0,-50.0,-50.0));
  READ_PARAMETER_OR_DEFAULT(UpperWorld,MSLVector(50.0,50.0,50.0));
  READ_PARAMETER_OR_DEFAULT(GlobalCameraPosition,
			    MSLVector((UpperWorld[0] + LowerWorld[0])/2.0,
				      (UpperWorld[1] + LowerWorld[1])/2.0,
				      UpperWorld[2] +
				      (UpperWorld[1]-
				       LowerWorld[1])/tan(38.0/180.0*PI)));
  READ_PARAMETER_OR_DEFAULT(GlobalCameraDirection,MSLVector(0.0,0.0,-1.0));
  READ_PARAMETER_OR_DEFAULT(GlobalCameraZenith,MSLVector(0.0,1.0,0.0));
  READ_PARAMETER_OR_DEFAULT(AttachedCameraPosition,MSLVector(0.0,0.0,6.0));
  READ_PARAMETER_OR_DEFAULT(AttachedCameraDirection,MSLVector(1.0,0.0,0.0));
  READ_PARAMETER_OR_DEFAULT(AttachedCameraZenith,MSLVector(0.0,0.0,1.0));
  READ_PARAMETER_OR_DEFAULT(AttachedCameraBody,0);

  GlobalCameraDirection = GlobalCameraDirection.norm();
  GlobalCameraZenith = GlobalCameraZenith.norm();
  AttachedCameraDirection = AttachedCameraDirection.norm();
  AttachedCameraZenith = AttachedCameraZenith.norm();

}



void Scene::SetProblem(Problem *problem) {
  P = problem;
  NumBodies = P->NumBodies;
  GeomDim = P->GeomDim;
  SceneConfigurationDim = NumBodies * 6;  // 6 DOF for each body
}


// By default, don't change anything
MSLVector Scene::StateToSceneConfiguration(const MSLVector &x) {
  MSLVector sc,q;
  int i;
  if (GeomDim == 3)
    return P->StateToConfiguration(x);
  else { // GeomDim == 2
    q = P->StateToConfiguration(x); // Three parameters per body
    // Blow out the MSLVector to make 6 parameters per body
    sc = MSLVector(NumBodies*6);
    for (i = 0; i < NumBodies; i++) {
      sc[6*i] = q[3*i];
      sc[6*i+1] = q[3*i+1];
      sc[6*i+2] = 0.0;
      sc[6*i+3] = 0.0;
      sc[6*i+4] = 0.0;
      sc[6*i+5] = q[3*i+2];
    }
    return sc;
  }
}


MSLVector Scene::InterpolatedSceneConfiguration(const MSLVector &x1,
						const MSLVector &x2,
						const double &a) {

  return StateToSceneConfiguration(P->InterpolateState(x1,x2,a));
}
