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

#ifndef MSL_GEOMPQP_H
#define MSL_GEOMPQP_H

//#include <list.h>
//#include <vector.h>
#include <PQP.h>

#include <stdlib.h>
#include <stdio.h>

#include "defs.h"
#include "geom.h"
#include "point.h"  // This includes the MSLPolygon class
#include "triangle.h"

// OK, this is bad.  The maximum number should be dynamic, but PQP
// really seems to want a static matrix.
#define MAXBODIES 100

//! Parent class PQP-based list of Triangle models

class GeomPQP: public Geom {
 protected:
  PQP_REAL RR[3][3],RO[3][3];
  PQP_REAL TR[3],TO[3];
 public:
  list<MSLTriangle> Obst;
  list<MSLTriangle> Robot;
  PQP_Model Ro, Ob;
  GeomPQP(string path);
  virtual ~GeomPQP() {};
  virtual void LoadEnvironment(string path);
  virtual void LoadRobot(string path);
  virtual bool CollisionFree(const MSLVector &q){return true;}
  virtual double DistanceComp(const MSLVector &q){return 10000.0;}
};

//! A parent class for 2D PQP geometries

class GeomPQP2D: public GeomPQP {
 public:
  list<MSLPolygon> ObstPolygons;
  list<MSLPolygon> RobotPolygons;
  GeomPQP2D(string path);
  virtual ~GeomPQP2D() {};
  virtual void LoadEnvironment(string path);
  virtual void LoadRobot(string path);
};


//! 2D rigid body

class GeomPQP2DRigid: public GeomPQP2D {
 public:
  GeomPQP2DRigid(string path);
  virtual ~GeomPQP2DRigid() {};
  virtual bool CollisionFree(const MSLVector &q); // Input is configuration
  virtual double DistanceComp(const MSLVector &q);  // Distance in world
  virtual MSLVector ConfigurationDifference(const MSLVector &q1,
					      const MSLVector &q2);
  void SetTransformation(const MSLVector &q); // Input is configuration
};


//! A collection of 2D rigid bodies
class GeomPQP2DRigidMulti: public GeomPQP2DRigid {
 private:
  vector<list<MSLTriangle> > Robot;
  vector<PQP_Model> Ro;
  list<MSLVector> CollisionPairs; // Index pairs to check for collision
  PQP_REAL TR[MAXBODIES][3];  // Robot translation
  PQP_REAL RR[MAXBODIES][3][3];  // Robot rotation
 public:
  bool SelfCollisionCheck;
  GeomPQP2DRigidMulti(string path);
  virtual ~GeomPQP2DRigidMulti() {};
  virtual bool CollisionFree(const MSLVector &q); // Input is configuration
  virtual double DistanceComp(const MSLVector &q);  // Distance in world
  virtual void LoadRobot(string path); // Load multiple robots
  void SetTransformation(const MSLVector &q); // Input is configuration
};


//! 3D rigid body
class GeomPQP3DRigid: public GeomPQP {
 public:
  GeomPQP3DRigid(string path);
  virtual ~GeomPQP3DRigid() {};
  virtual bool CollisionFree(const MSLVector &q); // Input is configuration
  virtual double DistanceComp(const MSLVector &q);  // Distance in world
  virtual MSLVector ConfigurationDifference(const MSLVector &q1,
					      const MSLVector &q2);
  void SetTransformation(const MSLVector &q); // Input is configuration
};


//! A collection of 3D rigid modies
class GeomPQP3DRigidMulti: public GeomPQP3DRigid {
 private:
  vector<list<MSLTriangle> > Robot;
  vector<PQP_Model> Ro;
  list<MSLVector> CollisionPairs; // Index pairs to check for collision
  PQP_REAL TR[MAXBODIES][3];  // Robot translation
  PQP_REAL RR[MAXBODIES][3][3];  // Robot rotation
 public:
  bool SelfCollisionCheck;
  GeomPQP3DRigidMulti(string path);
  virtual ~GeomPQP3DRigidMulti() {};
  virtual bool CollisionFree(const MSLVector &q); // Input is configuration
  virtual double DistanceComp(const MSLVector &q);  // Distance in world
  virtual void LoadRobot(string path); // Load multiple robots
  void SetTransformation(const MSLVector &q); // Input is configuration
};

#endif
