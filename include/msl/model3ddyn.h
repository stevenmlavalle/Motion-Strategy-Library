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

#ifndef MSL_MODEL3DDYN_H
#define MSL_MODEL3DDYN_H

#include <vector>

#include "vector.h"
#include "model3d.h"
#include "matrix.h"


//! A spacecraft model with three thrusters providing both tranlation
//! force and rotation torque
class Model3DDyn : public Model3DRigid {
 public:
  Model3DDyn(string path);
  virtual ~Model3DDyn() {};
  
  virtual MSLVector StateTransitionEquation(const MSLVector &x, 
					    const MSLVector &u);
  virtual bool Satisfied(const MSLVector &state);
  
  double m;
  MSLMatrix I;
};


#endif

