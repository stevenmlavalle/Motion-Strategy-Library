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

// This file is used to put the classes holding the extra information in
// the node in the search tree.
// Written by Peng Cheng (pcheng1@cs.uiuc.edu) 1/14/2002
 
#
#ifndef MSL_NODEINFO_H
#define MSL_NODEINFO_H

#include "vector.h" 

//! The information holded in this class is explained in "Reducing
//! Metric Sensitivity in Randomized Trajectory Design" in IEEE/RSJ
//! International Conference on Intelligent Robots and Systems, 2001

class MSLNodeInfo{

 private:
  //! Exploration information 
  MSLVector explorationinfo;

  //! Path collision tendency 
  double collisiontendency;

 public:

  //! Get the exploration information
  MSLVector GetExplorationInfo() {return explorationinfo; };

  //! Set the exploration information
  void SetExplorationInfo(MSLVector& exploreinfo) {
    explorationinfo = exploreinfo;
  };

  //! Get the collision tendency information
  double GetCollisionTendency() {return collisiontendency; };

  //! Set the collision tendency information
  void SetCollisionTendency(double& collisioninfo) {
    collisiontendency = collisioninfo;
  };

  MSLNodeInfo() { };
  MSLNodeInfo(const MSLVector& exploreinfo, const double& collisioninfo ){
    explorationinfo = exploreinfo;
    collisiontendency = collisioninfo;
  };
  ~MSLNodeInfo();

};

#endif


