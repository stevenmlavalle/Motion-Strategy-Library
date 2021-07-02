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

#ifndef MSL_SCENE_H
#define MSL_SCENE_H

#include "problem.h"
#include "util.h"

//! An interface class that gives Problem information to Render.
//! It tells the renderer how the "scene" appears for rendering
//! purposes, as opposed to collision-detection purposes.

/*!
This is an interface class that computes
configurations of all bodies to be displayed by a rendering method.
Currently, configuration information is passed through an instance of
Problem.  Thus, the base class for Scene is simply a wrapper to Problem.
*/

class Scene {
 protected:
  //! The Problem instance used by the planner/solver
  Problem *P;

 public:
  //! A common filepath
  string FilePath;

  //! The geometry dimension, 2 or 3
  int GeomDim;

  //! The number of bodies in the scene
  int NumBodies;

  //! Total degrees of freedom of the scene
  int SceneConfigurationDim;

  //! The smallest x,y,z world coordinates for the environment
  MSLVector LowerWorld;
  
  //! The largest x,y,z world coordinated for the environment
  MSLVector UpperWorld;

  //! The location of the default camera in x,y,z
  MSLVector GlobalCameraPosition;

  //! The direction the default camera is pointing with respect to the model
  MSLVector GlobalCameraDirection;

  //! The up direction of the default camera 
  MSLVector GlobalCameraZenith;

  //! The location of the body-attached camera in x,y,z
  MSLVector AttachedCameraPosition;

  //! The direction the body-attached camera is pointing
  MSLVector AttachedCameraDirection;

  //! The up direction of the body-attached camera 
  MSLVector AttachedCameraZenith;

  //! The index of the body to which the camera is attached (default = 0)
  int AttachedCameraBody;

  //! Scene must be initialized with the Problem that was passed to the planner
  Scene(Problem *problem, string path);
  
  //! Empty destructor
  virtual ~Scene() {};

  //! A method for changing the associated Problem
  void SetProblem(Problem *P);

  //! Convert the state into configurations for each body (a long MSLVector)
  virtual MSLVector StateToSceneConfiguration(const MSLVector &x);

  //! Interpolate the states; convert the result to a SceneConfiguratrion
  virtual MSLVector InterpolatedSceneConfiguration(const MSLVector &x1, 
						   const MSLVector &x2, 
						   const double &a);
};

#endif
