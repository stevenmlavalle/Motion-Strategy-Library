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

#ifndef MSL_MODEL_H
#define MSL_MODEL_H

#include <list>
#include <string>

#include "vector.h"
#include "matrix.h"

//! The incremental simulator model

/*!  The Model classes contain incremental simulators that model the
kinematics and dynamics of a variety of mechanical systems.  The
methods allow planning algorithms to compute the future system state,
given the current state, an interval of time, and a control input
applied over that interval.  (The planning algorithms select the
appropriate inputs using this information.)  Using object-oriented
class derivations, a wide variety of simulators can be included.  */

class Model {
 protected:
  //! The time interval to use for numerical integration (affects accuracy)
  double ModelDeltaT;

  //! The complete set of inputs
  list<MSLVector> Inputs;

  //! Integrate xdot using 4th-order Runge-Kutta
  MSLVector RungeKuttaIntegrate(const MSLVector &x, const MSLVector &u, const double &h);

  //! Integrate xdot using Euler integration
  MSLVector EulerIntegrate(const MSLVector &x, const MSLVector &u, const double &h);
 public:

  //! This file path is used for all file reads
  string FilePath;

  //! MSLVector of minimum values for each state variable
  MSLVector LowerState; 

  //! MSLVector of maximum values for each state variable
  MSLVector UpperState;

  //! MSLVector of minimum values for each input variable
  MSLVector LowerInput;

  //! MSLVector of maximum values for each input variable
  MSLVector UpperInput;

  //! The dimension of the state space
  int StateDim;
  
  //! The dimension of the input space
  int InputDim;

  //! Empty constructor
  Model(string path);

  //! Empty destructor
  virtual ~Model() {};

  //! Return a list of inputs, which may depend on state
  virtual list<MSLVector> GetInputs(const MSLVector &x);

  //! The state transition equation, or equations of motion, xdot=f(x,u)
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u) = 0;

  //! Test whether global state-space constraints are satisfied
  virtual bool Satisfied(const MSLVector &x);

  //! Perform integration from state x, using input u, over time step h
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, 
			   const double &h) = 0; 

  //! Linearly interpolate two state while respecting topology.
  /*! If a=0, then x1 is returned; if a=1, then x2 is returned.  All
      intermediate values of $a \in [0,1]$ yield intermediate states.
      This method is defined by Model.
  */
  virtual MSLVector LinearInterpolate(const MSLVector &x1, const MSLVector &x2, 
				   const double &a);  // Depends on topology

  //! Compute a MSLVector based on x2-x1.  In R^n, the states are simply
  //! subtracted to make the MSLVector.  This method exists to make things
  //! work correctly for other state-space topologies.
  virtual MSLVector StateDifference(const MSLVector &x1, const MSLVector &x2); 

  // Conversions
  //! A method that converts a Model state in to a Geom configuration
  virtual MSLVector StateToConfiguration(const MSLVector &x);

  //! A distance metric, which is Euclidean in the base class
  virtual double Metric(const MSLVector &x1, const MSLVector &x2); 

  // The following are used by optimization methods.  They are empty by
  // default because regular planners don't need them.  These could later
  // go in a derived class for optimization problems, but are left here
  // so that "regular" models can be converted to optimization models
  // by overriding these methods.

  //! Partial with respect to x of the state transition equation
  virtual void Partialf_x(const MSLVector &x, const MSLVector &u, MSLMatrix & m) {};

  //! Partial with respect to u of the state transition equation
  virtual void Partialf_u(const MSLVector &x, const MSLVector &u, MSLMatrix & m) {};

  //! A cost or loss function, to be used in optimization problems
  virtual void L(const MSLVector &x, const MSLVector &u, double &l) {};

  //! Partial of the loss with respect to x
  virtual void PartialL_x(const MSLVector &x, const MSLVector &u, MSLMatrix & m) {};

  //! Partial of the loss with respect to u
  virtual void PartialL_u(const MSLVector &x, const MSLVector &u, MSLMatrix & m) {};

  //! The final-state cost or loss
  virtual void Phi(const MSLVector &x, const MSLVector &u, 
		   const MSLVector &goalstate, double &phi) {};

  //! Partial of the final-state loss with respect to x
  virtual void PartialPhi_x(const MSLVector &x, const MSLVector &u, 
			    const MSLVector &goalstate, 
			    MSLMatrix & m) {};

  //! Partial of the final-state loss with respect to u
  virtual void PartialPhi_t(const MSLVector &x, const MSLVector &u, 
			    const MSLVector &goalstate, 
			    MSLMatrix & m) {};

  //! An error MSLVector that compares a goal state to a given state
  virtual void Psi(const MSLVector &x, const MSLVector &goalstate, MSLVector& psi) {};

  //! Partial of the error MSLVector with respect to x
  virtual void PartialPsi_x(const MSLVector &x, const MSLVector &u, MSLMatrix & m) {};

  //! Partial of the error MSLVector with respect to time
  virtual void PartialPsi_t(const MSLVector &x, const MSLVector &u, MSLMatrix & m) {};

};

#endif
