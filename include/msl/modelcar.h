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

#ifndef MSL_MODELCAR_H
#define MSL_MODELCAR_H

#include <list>
#include <string>

#include "model2d.h"
#include "vector.h"
#include "matrix.h"


//! The same model as Model2DRigidCar
class ModelCar: public Model2DRigidCar {
 public:
  double Speed;

  ModelCar(string path);
  virtual ~ModelCar() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual bool Satisfied(const MSLVector &state); 
};


//! The same model as Model2DRigidCarSmooth
class ModelCarSmooth: public Model2DRigidCarSmooth {
 public:
  ModelCarSmooth(string path);
  virtual ~ModelCarSmooth() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
};


//! The same model as Model2DRigidDyncar
class ModelCarDyn: public Model2DRigidDyncar {
 public:
  ModelCarDyn(string path);
  virtual ~ModelCarDyn() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
};

//! The same model as Model2DRigidDyncarNtire 
class ModelCarDynNtire: public Model2DRigidDyncarNtire {
 public:
  ModelCarDynNtire(string path);
  virtual ~ModelCarDynNtire() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
};


//! A car model considering the rolling effect and the pressure on
//! different tires of the car is different. If the pressure on one tire
//! is 0, the car is considered rolling over.  
//! The pressure model of the tire is rigid such that pressure can
//! change at instant time, which means: 
//! (1) It might be the reason that only forward RRT tree works. 
//! (2) In the SelectInput function, pressure has to be restored when to
//! test new inputs.
class ModelCarDynRollover: public ModelCarDynNtire{  
 public:
  double K, c, Ixx;
  double T;
  double H, H2;
  double Ms;
  double Wn;
  double Fai;
  double x; 
  
  bool IsRollOver;

  ModelCarDynRollover(string path);
  virtual ~ModelCarDynRollover() {};

  int sgn(double x);

  virtual MSLVector StateTransitionEquation(const MSLVector &x1, const MSLVector &u);

  virtual MSLVector StateToConfiguration(const MSLVector &x);
  
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);   

  virtual double Metric(const MSLVector &x1, const MSLVector &x2);

  bool RollOverFree(const MSLVector &x);

  bool Satisfied(const MSLVector &x);
};


//! One more dimension than ModelCarDynRollover considering the
//! steering angle can only change continuously.
class ModelCarDynSmoothRollover: public ModelCarDynRollover {
 public:
 
  ModelCarDynSmoothRollover(string path);
  virtual ~ModelCarDynSmoothRollover() {};

  virtual MSLVector StateTransitionEquation(const MSLVector &x1, const MSLVector &u);

  virtual MSLVector StateToConfiguration(const MSLVector &x);

  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  
  virtual MSLVector LinearInterpolate(const MSLVector &x1, 
				      const MSLVector &x2, 
				      const double &a);
  
};

#endif








