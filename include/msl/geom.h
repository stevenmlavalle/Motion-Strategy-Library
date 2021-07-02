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

#ifndef MSL_GEOM_H
#define MSL_GEOM_H


// The world places objects using configuration (model uses state).
// Each derived class can define Obst and Robot however it likes.
// Methods such as "GeomToPolygons" can give output in whatever 
//  format is requested.  So far, only list<polygon> is shown,
//  but we could imagine an OpenInventor model, Geomview model, etc.

//! Geometric models and collision detection methods.

/*!  These classes define the geometric representations of all obstacles in
the world, and of each part of the robot.  The methods allow planning
algorithms to determine whether any of the robot parts are in
collision with each other or with obstacles in the world.  

A configuration vector specifies the positions and orientation of each
rigid body.
*/

//#include <list.h>
#include <string>

#include "vector.h"
#include "util.h"

class Geom {
 protected:
  string FilePath;
 public:
  //! Empty constructor in base class
  Geom(string path);

  //! Empty destructor
  virtual ~Geom() {};

  //! The number of rigid bodies in the geometry model
  int NumBodies;

  //! The dimension of the world geometry: 2 or 3
  int GeomDim;

  //! Return true if the robot(s) and obstacles are not in collision
  virtual bool CollisionFree(const MSLVector &q) = 0; // Input is configuration

  //! Compute the distance of the closest point on the robot to the
  //! obstacle region.
  virtual double DistanceComp(const MSLVector &q) = 0;  // Distance in world

  //! Maximum displacement of geometry with respect to change in each variable
  MSLVector MaxDeviates;

  //! Compute a MSLVector based on q2-q1.  In R^n, the configurations are simply
  //! subtracted to make the MSLVector.  This method exists to make things
  //! work correctly for other configuration-space topologies.
  virtual MSLVector ConfigurationDifference(const MSLVector &q1, 
					    const MSLVector &q2); 
};


//! A class with no geometry -- a collision never happens
class GeomNone: public Geom {
 public:
  GeomNone(string path);
  virtual ~GeomNone() {};
  virtual bool CollisionFree(const MSLVector &q){return true;} 
  virtual double DistanceComp(const MSLVector &q){return 10000.0;}
};

#endif
