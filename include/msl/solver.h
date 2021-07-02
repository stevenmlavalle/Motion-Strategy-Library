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

#ifndef MSL_SOLVER_H
#define MSL_SOLVER_H

#include "problem.h"
#include "util.h"

//! The base class for all path planners

class Solver {
 public:
  //! The directory in which all files for a problem will be stored
  string FilePath;

  /** An instance of problem, which defines all of the problem-specific
      methods needed for solvers.  This includes incremental simulation
      and collision detection. */
  Problem *P;

  //! A constructor that initializes data members.
  Solver(Problem *problem);

  //! Empty destructor
  virtual ~Solver() {};

};


#endif










