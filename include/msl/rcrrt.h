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

#ifndef MSL_RRTDYN_H
#define MSL_RRTDYN_H

#include "rrt.h"


//! Resolution Complete Rapidly-Exploring Random Trees , by Peng Cheng
//! and Steven M. LaValle, submitted to 2002 IEEE International
//! Conference on Robotics and Automation.
//! Techniques applied to improve the performance: 
//!  (1) Combining systematic search with random search such that it has
//!      both the completeness of the systematic search and fast searching 
//!      of the random search.
//!  (2) Constraint violation tendency to avoid obstacles
//! This basic planner is used to do the experiment with dynamic car
//! model in the virtual town.  The rolling effect of the car and the
//! nonlinear tire model are considered in the model.
class RCRRT: public RRTGoalBias {
public:
  //! the number of the inputs
  int inputnum;

  //! input set
  list<MSLVector> inputset;

  //! true, solution possibly exists; 
  //! false, solution does not exist
  bool issolutionexist;

  //! Initial exploration information initialized with 0
  MSLVector initexploreinfo;

  //! Initial collision tendency
  double initcoltend;


  //! Check if this node has been explored (all inputs are expanded)
  //! and return the collision tendency information
  //! Used in SelectNode function
  bool IsNodeExpanded(MSLNode* x, double& biasvalue, 
		      bool forward);

  //! Check the ith input is failed or not
  //! Used in SelectInput function 
  virtual bool IsInputApplied(const int& inputindex, 
			      const MSLVector& expolreinfo);


  //! back ward along the tree to set the bias value
  virtual void BackWardBiasSet(MSLNode* n, MSLTree* t);

  //! bias value function, 
  double BiasValue(int backstep);

  RCRRT(Problem *problem);
  virtual ~RCRRT() {};

  virtual MSLNode* SelectNode(const MSLVector& x, MSLTree* t, 
			      bool forward);

  virtual bool Extend(const MSLVector& x, MSLTree* t, 
		      MSLNode*& nn,bool forward);

  //! The first parameter is node because the node will be used in the
  //! function, xu_new is the new value for the uncontrolled state
  virtual MSLVector SelectInput(MSLNode* n1, const MSLVector& x2, 
				    MSLVector& nx_best, bool& success, 
				    bool forward);
  
  virtual bool Connect(const MSLVector& x, MSLTree* t, 
		       MSLNode*& nn, bool forward);  
  
  virtual bool Plan();  
};


//! Basic dual tree version of RCRRT
class RCRRTDual: public RCRRT
{
 protected:
  void RecoverSolution(MSLNode* n1, MSLNode* n2);

 public:

  RCRRTDual(Problem *p);
  virtual ~RCRRTDual() {};

  virtual bool Plan();
  virtual bool GetConnected(MSLNode* n1, MSLNode* n2);
};


//! Dual tree version of RCRRT with ExtExt method, which is used to do
//! experiments with spacecraft model in 3D grid environment considering
//! the dynamic constraints .
class RCRRTExtExt: public RCRRTDual
{
 public:
  RCRRTExtExt(Problem *p);
  virtual ~RCRRTExtExt() {};
  virtual bool Plan();
};


//! RCRRT planner using ball neighborhood to exclude the repeated
//! states

class RCRRTBall: public RCRRT
{
 public:

  //! the radius of the balls surrounding the nodes  
  double BallRadius;  

  //! the termination number, if FailNum random points are in the balls
  //! in row the algorithm terminates
  int FailNumTh;

  //! the number of times of the random points in the balls in row
  int FailNum;


  RCRRTBall(Problem *p);
  virtual ~RCRRTBall() {};

  //! check the termination condition: n points in the balls in row
  virtual MSLNode* SelectNode(const MSLVector &x, MSLTree* t, 
				   bool forward);

  virtual bool Extend(const MSLVector &x, MSLTree* t, 
		      MSLNode*& nn, bool forward);

  //! extend the best input to a new state and check if the new state is in some balls
  virtual bool Connect(const MSLVector& x, MSLTree* t, MSLNode*& nn,
		       bool forward);  
  
  virtual bool Plan();
};

//! Basic dual tree version of RCRRTBall
class RCRRTBallDual: public RCRRTBall
{
 protected:
  void RecoverSolution(MSLNode* n1, MSLNode* n2);

 public:

  RCRRTBallDual(Problem *p);
  virtual ~RCRRTBallDual() {};

  virtual bool Plan();
  virtual bool GetConnected(MSLNode* n1, MSLNode* n2);
};


//! Dual tree version of RCRRTBall with ExtExt method 
class RCRRTBallExtExt: public RCRRTBallDual
{
 public:
  RCRRTBallExtExt(Problem *p);
  virtual ~RCRRTBallExtExt() {};
  virtual bool Plan();
};

#endif







