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

#ifndef MSL_PROBLEM_H
#define MSL_PROBLEM_H

//#include <list.h>
//#include <string>

#include "model.h"
#include "geom.h"
#include "util.h"

//! An interface class that provides the primary input to a planner.

/*! This interface class contains protected instances of Geom and
Model.  Wrappers to methods from Geom provide collision detection and
distance computation.  Wrappers to methods from Model provide
incremental simulation of a kinematic or dynamical system.  It is
expected a planner can get all (or nearly all) of the information it
needs from Problem.
*/

class Problem {
 protected:
  //! Need to define a geometry for collision detection
  Geom *G;
  //! xdot = f(x,u), integration technique, state bounds
  Model *M;
 public:
  //! The directory in which all files for a problem will be stored
  string FilePath;

  //! The number of rigid bodies
  int NumBodies;

  //! The dimenson of the state space
  int StateDim;

  //! The dimension of the input space
  int InputDim;

  //! The dimension of the geometric model
  int GeomDim;

  //! MSLVector of minimum values for each state variable
  MSLVector LowerState;

  //! MSLVector of maximum values for each state variable
  MSLVector UpperState;

  //! The starting state for a planner
  MSLVector InitialState;

  //! The goal state for a planner
  MSLVector GoalState;

  //! Problem must be given any instance of Geom and any instance of
  //! Model from each of their class hierarchies
  Problem(Geom *geom, Model *model, string path);

  //! Empty destructor
  virtual ~Problem() {};

  //! Change the instance of Geom
  void SetGeom(Geom *geom);

  //! Change the instance of Model
  void SetModel(Model *model);

  //! Return a list of possible inputs, which may depend on state
  virtual list<MSLVector> GetInputs(const MSLVector &x);

  //! Return a list of possible inputs
  virtual list<MSLVector> GetInputs();

  //! Perform integration from state x, using input u, over time deltat
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, 
			   const double &deltat);

  //! Linearly interpolate two states while respecting topology.
  /*! If a=0, then x1 is returned; if a=1, then x2 is returned.  All
      intermediate values of $a \in [0,1]$ yield intermediate states.
      This method is defined by Model.
  */
  virtual MSLVector InterpolateState(const MSLVector &x1, const MSLVector &x2, 
				   const double &a);  // Depends on topology

  //! A distance metric defined in Model.
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);

  //! A method that converts a Model state in to a Geom configuration
  virtual MSLVector StateToConfiguration(const MSLVector &x);

  //! Compute a MSLVector based on x2-x1.  In R^n, the states are simply
  //! subtracted to make the MSLVector.  This method exists to make things
  //! work correctly for other state-space topologies.
  virtual MSLVector StateDifference(const MSLVector &x1, 
				 const MSLVector &x2); 

  //! This takes the logical AND of CollisionFree from Geom, and Satisfied from
  //!  Model.
  virtual bool Satisfied(const MSLVector &x);

  //! The collision checker passed in from Geom
  virtual bool CollisionFree(const MSLVector &q);

  //! The distance computation algorithm from Geom
  virtual double DistanceComp(const MSLVector &q); 

  //! Maximum displacement of geometry with respect to change in each variable
  MSLVector MaxDeviates; 

  //! Compute a MSLVector based on q2-q1.  In R^n, the configurations are simply
  //! subtracted to make the MSLVector.  This method exists to make things
  //! work correctly for other configuration-space topologies.
  virtual MSLVector ConfigurationDifference(const MSLVector &q1, 
					 const MSLVector &q2); 
};

#endif


