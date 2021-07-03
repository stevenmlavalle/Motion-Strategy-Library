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

#include <math.h>
#include <stdlib.h>

// Include all models
#include "msl/model.h"
#include "msl/modelmisc.h"
#include "msl/model2d.h"
#include "msl/model3d.h"
#include "msl/modelcar.h"

// Include all geometries
#include "msl/geom.h"
#include "msl/geom_pqp.h"

#include "msl/util.h"

#include "msl/defs.h"

#include "msl_gui/setup.h"

#define MAKE_MODEL(_m)  if (is_file(path+""#_m"")) m = new _m(path);
#define MAKE_GEOM(_g)  if (is_file(path+""#_g"")) g = new _g(path);

void SetupProblem(Model *&m, Geom *&g, string path) {
  // Make them null initially
  m = NULL;
  g = NULL;

  // Models from modelmisc.h
  MAKE_MODEL(Model1D);
  MAKE_MODEL(ModelND);
  MAKE_MODEL(ModelLinear);
  MAKE_MODEL(ModelNintegrator);

  // Models from model2d.h
  MAKE_MODEL(Model2DPoint);
  MAKE_MODEL(Model2DPointCar);
  MAKE_MODEL(Model2DRigid);
  MAKE_MODEL(Model2DRigidCar);
  MAKE_MODEL(Model2DRigidCarForward);
  MAKE_MODEL(Model2DRigidCarSmooth);
  MAKE_MODEL(Model2DRigidCarSmoothTrailer);
  MAKE_MODEL(Model2DRigidCarSmooth2Trailers);
  MAKE_MODEL(Model2DRigidCarSmooth3Trailers);
  MAKE_MODEL(Model2DRigidDyncar);
  MAKE_MODEL(Model2DRigidDyncarNtire);
  MAKE_MODEL(Model2DRigidMulti);
  MAKE_MODEL(Model2DRigidChain);

  // Models from model3d.h
  MAKE_MODEL(Model3DRigid);
  MAKE_MODEL(Model3DRigidMulti);
  MAKE_MODEL(Model3DRigidChain);
  MAKE_MODEL(Model3DRigidTree);

  // Models from modelcar.h
  MAKE_MODEL(ModelCar);
  MAKE_MODEL(ModelCarSmooth);
  MAKE_MODEL(ModelCarDyn);
  MAKE_MODEL(ModelCarDynNtire);
  MAKE_MODEL(ModelCarDynRollover);
  MAKE_MODEL(ModelCarDynSmoothRollover);

  // Geoms from geom.h
  MAKE_GEOM(GeomNone);

  // Geoms from geomPQP.h
  MAKE_GEOM(GeomPQP2DRigid);
  MAKE_GEOM(GeomPQP2DRigidMulti);
  MAKE_GEOM(GeomPQP3DRigid);
  MAKE_GEOM(GeomPQP3DRigidMulti);

  if (m == NULL) // Make a default Model
    m = new Model2DPoint(path);
  if (g == NULL) // Make a default Geom
    g = new GeomPQP2DRigid(path);

}
