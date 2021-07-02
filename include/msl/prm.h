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

#ifndef MSL_PRM_H
#define MSL_PRM_H

#include <queue>

#include "planner.h"
#include "util.h"

/*! The base class for planners based on the Probabilistic Roadmap Planner (PRM)
  framework of Kavraki, Svestka, Latombe, Overmars, 1994.  In the base class,
  only Holonomic planning problems can be solved (i.e., standard path planning,
  without differential constraints).

*/

//! A probabilistic roadmap planner, proposed by Kavraki, Svestka, Latombe, Overmars, 1994
class PRM: public RoadmapPlanner {
 protected:
  virtual list<MSLVertex*> NeighboringVertices(const MSLVector &x);
  virtual bool Connect(const MSLVector &x1, const MSLVector &x2, MSLVector &u);
  virtual MSLVector ChooseState(int i, int maxnum, int dim);
  MSLVector QuasiRandomStateHammersley(int i, int maxnum, int dim);
  MSLVector QuasiRandomStateHalton(int i, int dim);
  //! Computed from DeltaT using the model
  double StepSize;  // Derived from DeltaT using the model
  int MaxNeighbors;
  int MaxEdgesPerVertex;
 public:

  //! Used for deciding on which neighbors to choose
  double Radius;

  //! Number of times the collision checker has been called
  int SatisfiedCount;

  //! If true, then quasirandom sampling is used (make a file named QuasiRandom)
  bool QuasiRandom;

  //! Choose Hammersley, over Halton sequence
  bool QuasiRandomHammersley;

  //! A constructor that initializes data members.
  PRM(Problem *problem);

  //! Empty destructor
  virtual ~PRM() {};

  //! Build a PRM
  virtual void Construct();

  //! Try to solve a planning query using an existing PRM
  virtual bool Plan();
};


#endif

