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

#ifndef MSL_RRT_H
#define MSL_RRT_H

#include "planner.h"
#include "util.h"

#ifdef USE_ANN
  #include <ANN/ANN.h>			// ANN declarations
  #include "nn.h"
  #include "multiann.h"
#endif

/*!  The base class for the planners based on Rapidly-exploring 
Random Trees.  In the base class, a single tree is generated without
any regard to the GoalState.  The best planners to try are 
RRTGoalBias and RRTGoalZoom for single trees, and RRTConCon and 
RRTExtExt for dual trees.  Dual tree approaches are much more efficient
than single tree approaches, assuming dual trees can be applied.  */

//! The base class, which generates a single Rapidly-exploring Random Tree.

class RRT: public IncrementalPlanner {
 protected:
  //! Select the input that gets closest to x2 from x1
  virtual MSLVector SelectInput(const MSLVector &x1, 
				    const MSLVector &x2, 
				    MSLVector &nx_best, 
				    bool &success, 
				    bool forward);
  
  //! Return the nearest neighbor in the graph
  virtual MSLNode* SelectNode(const MSLVector &x, MSLTree *t,
				   bool forward);

  //! Incrementally extend the RRT
  virtual bool Extend(const MSLVector &x, MSLTree *t, MSLNode *&nn,
		      bool forward);

  //! Iterated Extend
  virtual bool Connect(const MSLVector &x, MSLTree *t, MSLNode *&nn,
		       bool forward);
  
  //! Pick a state using some sampling technique
  virtual MSLVector ChooseState();

  public:

  //! If true, then the ANN package is used for nearest neighbors.  It
  //! assumes R^n topology and Euclidean metric.  The default is false.
  bool UseANN;  

  //! The distance of the closest RRT MSLNode to the goal
  double GoalDist;

  //! The closest state to the goal so far (not used in dual-tree planners)
  MSLVector BestState;

  //! The maximum amount of time to move in a Connect step (default = INFINITY)
  double ConnectTimeLimit;

#ifdef USE_ANN
  MultiANN MAG;
  MultiANN MAG2;
#endif

  //! A constructor that initializes data members.
  RRT(Problem *problem);

  //! Empty destructor
  virtual ~RRT() {};
  
  //! Number of times the collision checker has been called
  int SatisfiedCount;

  //! Reset the planner
  virtual void Reset();

  //! Attempt to solve an Initial-Goal query by growing an RRT
  virtual bool Plan();

};


/*! Instead of choosing a state at random, this planner chooses with
    probability GoalProb the GoalState.  It can be considered as a 
    biased coin toss in which heads yields the goal state, and tails
    yields a random sample.
*/
//! With some probability, choose the goal instead of a random sample
class RRTGoalBias: public RRT {
 protected:
  virtual MSLVector ChooseState();
 public:
  double GoalProb;
  RRTGoalBias(Problem *p);
  virtual ~RRTGoalBias() {};
};



/*! The RRT in the base class uses Extend to move a small amount in
    each step toward the random sample.  In RRTCon, Extend is replaced
    by a method called Connect, which iterates the extension until the
    random sample is reached.  Connect only adds the final 
    MSLNode to the tree (not the intermediate increments).  Since RRTCon
    is derived from RRTGoalBias, a biasing probability can be set.  By
    default, GoalProb = 0.0.
 */
//! Replaces Extend with Connect

class RRTCon: public RRTGoalBias {
 public:
  RRTCon(Problem *p);

  //! An empty desctructor.

  virtual ~RRTCon() {};

  //! Here Extend is replaced by Connect.  The tree is extended all the 
  //! to the random sample, if possible.

  virtual bool Plan();
};



/*! This planner grows one tree, G, from the initial state, 
  and one tree, G2, from the goal state.  When the closest
  pair of points between the two trees is within GapError, a
  solution path is returned.  The planners in the derived
  classes also use two trees, and are generally more efficient
  than RRTDual.
*/
//! Planners that grow trees from the initial and goal

class RRTDual: public RRT {
 protected:
  void RecoverSolution(MSLNode *n1, MSLNode *n2);
 public:
  RRTDual(Problem *p);
  virtual ~RRTDual() {};



//! The dual-tree planner used in LaValle, Kuffner, ICRA 1999.  Each 
//! tree is extended toward a randomly-sampled point.  RRTExtExt is
//! generally better.

  virtual bool Plan();
};



/*! This planner balances the computation between growing the trees
    toward random samples and toward each other.  G is the tree from the
    initial state, and G2 is the tree from the goal state.  In each iteration,
    there are four steps:
    <ol>
    <li>Use Extend to grow G toward a random sample</li>
    <li>Use Extend to grow G2 toward the new node in G</li>
    <li>Use Extend to grow G2 toward a random sample</li>
    <li>Use Extend to grow G toward the new node in G2</li>
    </ol>
    In each step, node selection is based on the nearest neighbor.
*/
//! Balance between growing trees toward each other and exploring
class RRTExtExt: public RRTDual {
 public:
  RRTExtExt(Problem *p);
  virtual ~RRTExtExt() {};
  virtual bool Plan();
};


/*! This planner biases the sampling towards a region that contains 
  the goal.  The region shrinks around the goal as the tree comes nearer.  
  This planner is based on a class project at Iowa State by Jun Qu in 1999.
*/
//! Bias the samples toward the goal as the tree gets closer
class RRTGoalZoom: public RRT {
 protected:
  virtual MSLVector ChooseState();
 public:
  double GoalProb,ZoomProb,ZoomFactor;
  RRTGoalZoom(Problem *p);
  virtual ~RRTGoalZoom() {};
};


/*! Instead of random sampling, this planner attempts to gradually bias 
samples toward the goal.
*/
//! Gradually bias the sampling towards the goal
class RRTPolar: public RRT {
 protected:
  virtual MSLVector ChooseState();
  virtual MSLVector SelectInput(const MSLVector &x1, const MSLVector &x2, 
				    MSLVector &nx_best, bool &success);
 public:
  double RadiusExp;
  RRTPolar(Problem *p);
  virtual ~RRTPolar() {};
};

/*! This is a special-purpose class that grows a tree in an "infinitely"
large disc to study asymptotic properties.
*/
//! Grow a Rapidly-exploring Random Tree in a large disc.
class RRTHull: public RRT {
 protected:
  virtual MSLVector ChooseState();
 public:
  double Radius;
  RRTHull(Problem *p);
  virtual ~RRTHull() {};
};


/*! This planner balances the computation between growing the trees
    toward random samples and toward each other.  G is the tree from the
    initial state, and G2 is the tree from the goal state.  In each iteration,
    there are four steps:
    <ol>
    <li>Use Extend to grow G toward a random sample</li>
    <li>Use Connect to grow G2 toward the new node in G</li>
    <li>Use Extend to grow G2 toward a random sample</li>
    <li>Use Connect to grow G toward the new node in G2</li>
    </ol>
    In each step, node selection is based on the nearest neighbor.

    The planner is described in Kuffner, LaValle, ICRA, 2000.  The only
    difference between RRTExtExt and RRTExtCon is the replacement of
    Extend with Connect in two steps.
*/
//! Use Connect instead of Extend to connect the two trees
class RRTExtCon: public RRTDual {
 public:
  RRTExtCon(Problem *p);
  virtual ~RRTExtCon() {};

  //! This planner is greedier in its attempt to connect the trees.
  virtual bool Plan();
};



/*! This planner balances the computation between growing the trees
    toward random samples and toward each other.  G is the tree from the
    initial state, and G2 is the tree from the goal state.  In each iteration,
    there are four steps:
    <ol>
    <li>Use Connect to grow G toward a random sample</li>
    <li>Use Connect to grow G2 toward the new node in G</li>
    <li>Use Connect to grow G2 toward a random sample</li>
    <li>Use Connect to grow G toward the new node in G2</li>
    </ol>
    In each step, node selection is based on the nearest neighbor.

    The planner is described in LaValle, Kuffner, WAFR, 2000.  The only
    difference between RRTConCon and RRTExtExt is the replacement of
    Extend with Connect in all four steps.
*/
//! Use Connect for both exploration and connecting of trees

class RRTConCon: public RRTDual {
 public:
  RRTConCon(Problem *p);
  virtual ~RRTConCon() {};

  //! The greediest of the dual-tree planners.  
  //! Very fast for holonomic planning.
  virtual bool Plan();
};


/*! This planner behaves similar to RRTConCon, except that a
    cardinality criteria is introduced to maintain relative balance
    between the number of nodes in each tree.

    At each iteration, the tree with the fewest nodes is selected as
    the active tree.  The planner attempts to add a new branch to the
    currently active tree using the Connect step.
    
    Keeping the trees balanced has the dual effect of minimizing the
    overall number of nearest-neighbor calculations, and efficiently
    solving problems in which either of the initial or goal
    configurations is highly constrained relative to the other (as is
    often the case with assembly or disassembly planning).

    This planner is able to consistently solve the original well-known
    alpha puzzle motion planning benchmark.

    The planner is described in Kuffner, LaValle, Yang, 2002 */
//! Balanced Bidirectional RRTConCon planner

class RRTBidirBalanced: public RRTDual {
 public:
  RRTBidirBalanced(Problem *p);
  virtual ~RRTBidirBalanced() {};

  //! First planner to solve the alpha 1.0 puzzle
  //! Very fast for holonomic planning.
  virtual bool Plan();
};


/*! Grow a tree incrementally by simply selecting vertex at random and 
    moving in a random direction from the chosen vertex.   It is not 
    really a Rapidly-exploring Random Tree since there is no random
    sampling over the state space to "pull" the tree.  */
//! Naively extend the tree by random node selection (not really an RRT)
class RandomTree: public RRT {
 protected:
  virtual MSLNode* SelectNode(const MSLVector &x, MSLTree *t,
			       bool forward);
  virtual MSLVector SelectInput(const MSLVector &x1, const MSLVector &x2, 
				MSLVector &nx_best, bool &success, 
				bool forward);
 public:
  RandomTree(Problem *p);
  virtual ~RandomTree() {};
};

#endif



