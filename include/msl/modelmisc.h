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

#ifndef MSL_MODELMISC_H
#define MSL_MODELMISC_H

#ifdef WIN32
	#include <string>
	using namespace std;
#else
	#include <string>
#endif

#include "model.h"
#include "vector.h"
#include "matrix.h"

//! A simple one-dimensional model for dynamics studies
class Model1D: public Model {
 public:
  double Force;
  Model1D(string path);
  virtual ~Model1D() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
};


//! A linear systems model:   xdot = Ax + Bu
class ModelLinear: public Model {
 public:
  MSLMatrix A;
  MSLMatrix B;
  ModelLinear(string path);
  virtual ~ModelLinear() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, 
			      const double &h);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, 
					    const MSLVector &u);
};



//! Simple axis-parallel motions in an N-dimensional space 
class ModelND: public Model {
 public:
  double CorridorWidth;
  ModelND(string path);
  virtual ~ModelND() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  //virtual bool Satisfied(const MSLVector &x);
};


//! The "nonholonomic integrator", used by R. Brockett and many others.
class ModelNintegrator: public Model {
 public:
  double UBound;
  double VBound;
  ModelNintegrator(string path);
  virtual ~ModelNintegrator() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
};


#endif
