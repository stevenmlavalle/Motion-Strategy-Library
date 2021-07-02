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

#ifndef MSL_MODEL2D_H
#define MSL_MODEL2D_H

#include "model.h"

//! Base for all 2D models
class Model2D: public Model {
 public:
  Model2D(string path);
  virtual ~Model2D() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
};


//! A point robot in a 2D world
class Model2DPoint: public Model2D {
 public:
  Model2DPoint(string path);
  virtual ~Model2DPoint() {};
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
};


//! A point car-like robot in a 2D world
class Model2DPointCar: public Model2DPoint {
 public:
  double MaxSteeringAngle;
  double CarLength;
  Model2DPointCar(string path);
  virtual ~Model2DPointCar() {};
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
};


//! A holonomic rigid robot in a 2D world.  
class Model2DRigid: public Model2D {
 public:
  Model2DRigid(string path);
  virtual ~Model2DRigid() {};
  virtual MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  MSLVector LinearInterpolate(const MSLVector &x1, const MSLVector &x2, 
			   const double &a);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  virtual MSLVector StateToConfiguration(const MSLVector &x);
};


//! A rigid car-like robot in a 2D world
class Model2DRigidCar: public Model2DRigid {
 public:
  double MaxSteeringAngle;
  double CarLength;
  Model2DRigidCar(string path);
  virtual ~Model2DRigidCar() {};
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
};


//! A rigid car-like robot that can only go forward in a 2D world
class Model2DRigidCarForward: public Model2DRigidCar {
 public:
  Model2DRigidCarForward(string path);
  virtual ~Model2DRigidCarForward() {};
};


//! A rigid car-like robot with continuous steering angles
//! This model is used by Th. Fraichard, Scheuer, Laugier
class Model2DRigidCarSmooth: public Model2DRigidCar {
 public:
  double SteeringSpeed;
  Model2DRigidCarSmooth(string path);
  virtual ~Model2DRigidCarSmooth() {};
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual bool Satisfied(const MSLVector &x);
};

//! A rigid car-like robot with continuous steering angles and a trailer.
//! The trailer models are taken from Murray and Sastry, Trans. 
//!  Automatic Control, Vol 38, No 5, 1993, pp. 700-716
class Model2DRigidCarSmoothTrailer: public Model2DRigidCarSmooth {
 public:
  double HitchLength;
  double HitchMaxAngle;
  Model2DRigidCarSmoothTrailer(string path);
  virtual ~Model2DRigidCarSmoothTrailer() {};
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual bool Satisfied(const MSLVector &x);
};


//! A rigid car-like robot with continuous steering angles and two trailers.
class Model2DRigidCarSmooth2Trailers: public Model2DRigidCarSmoothTrailer {
 public:
  double Hitch2Length;
  double Hitch2MaxAngle;
  Model2DRigidCarSmooth2Trailers(string path);
  virtual ~Model2DRigidCarSmooth2Trailers() {};
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual bool Satisfied(const MSLVector &x);
};


//! A rigid car-like robot with continuous steering angles and three trailers.
class Model2DRigidCarSmooth3Trailers: public Model2DRigidCarSmooth2Trailers {
 public:
  double Hitch3Length;
  double Hitch3MaxAngle;
  Model2DRigidCarSmooth3Trailers(string path);
  virtual ~Model2DRigidCarSmooth3Trailers() {};
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual bool Satisfied(const MSLVector &x);
};



//! A 5DOF dynamical model of a rigid car.  This model uses a linear
//! tire model, which is far from reality.  The model was donated by
//! Jim Bernard.
class Model2DRigidDyncar: public Model2DRigid {
 public:
  //! Mass in slugs (yuck! American customary units...)
  double Mass;

  //! Front cornering stiffness
  double CAF;

  //! Rear cornering stiffness
  double CAR;

  //! Mass center to front tires - feet
  double Adist;

  //! Mass center to rear tires - feet
  double Bdist;

  //! Yaw moment of interia - ft slugs^2
  double Izz;

  //! Feet per world unit (100x100 world)
  double WorldScale;

  //! Maximum steering angle in radians
  double MaxSteeringAngle;

  //! A forward speed of the car
  double Speed;

  Model2DRigidDyncar(string path);
  virtual ~Model2DRigidDyncar() {};
  MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  virtual MSLVector LinearInterpolate(const MSLVector &x1, const MSLVector &x2, 
				   const double &a);
};


//! A 5DOF dynamical model of a rigid car.  This model uses a nonlinear
//! tire model.  The model was donated by Jim Bernard.
class Model2DRigidDyncarNtire: public Model2DRigidDyncar {
 public:

  //! Nonlinear tire model constant
  double Mu;

  //! Load on the front tires
  double Nf;

  //! Load on the rear tires
  double Nr;

  Model2DRigidDyncarNtire(string path);
  virtual ~Model2DRigidDyncarNtire() {};
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
};


//! A rigid body with two small side thrusters, and a larger lower thruster.
//! The goal is to navigate and softly "land" the craft by
//! firing thrusters, in spite of gravity.
class Model2DRigidLander: public Model2DRigid {
 public:
  //! Mass in kg
  double Mass;

  //! Accel of gravity (m/s^2)
  double G;

  //! Side thruster force
  double Fs;         

  //! Upward thruster force
  double Fu;

  Model2DRigidLander(string path);
  virtual ~Model2DRigidLander() {};
  MSLVector Integrate(const MSLVector &x, const MSLVector &u, const double &h);
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
};



//! A collection of free-floating bodies in a 2D world
class Model2DRigidMulti: public Model2DRigid {
 public:
  //! Number of independent rigid bodies
  int NumBodies;

  Model2DRigidMulti(string path);
  virtual ~Model2DRigidMulti() {}
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual MSLVector LinearInterpolate(const MSLVector &x1, const MSLVector &x2, 
				   const double &a);  // Depends on topology
};


//! A 2D kinematic chain of bodies.
class Model2DRigidChain: public Model2DRigid {
 public:
  //! Number of bodies in the chain
  int NumBodies;

  //! The distances between joints ("a" parameters in kinematics)
  MSLVector A;

  //! The default joint limits (must be in 0,PI)
  double StopAngle;  

  Model2DRigidChain(string path);
  virtual ~Model2DRigidChain() {};
  virtual MSLVector StateToConfiguration(const MSLVector &x);
  virtual MSLVector StateTransitionEquation(const MSLVector &x, const MSLVector &u);
  virtual MSLVector LinearInterpolate(const MSLVector &x1, const MSLVector &x2, 
				   const double &a);
  virtual double Metric(const MSLVector &x1, const MSLVector &x2);
  virtual bool Satisfied(const MSLVector &x);
};


#endif


