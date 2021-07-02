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

#ifndef MSL_PLANNER_H
#define MSL_PLANNER_H

#include <list>
#include <fstream>
using namespace std;

#include "solver.h"
#include "random.h"
#include "graph.h"
#include "tree.h"
#include "vector.h"
#include "util.h"

//! The base class for all path planners
class Planner: public Solver {
 protected:
  MSLRandomSource R;

  //! Choose a state at random
  MSLVector RandomState();

  //! Pick a state using a Normal distribution
  MSLVector NormalState(MSLVector mean, double sd);

 public:
  //! Total amount of time spent on planning
  double CumulativePlanningTime;

  //! Total amount of time spent on construction
  double CumulativeConstructTime;

  //! The solution path, as a list of states
  list<MSLVector> Path;

  //! The solution policy, as a list of inputs
  list<MSLVector> Policy;

  //! The last state in a path before a jump occurs
  MSLVector GapState;

  //! Set to true to ignore inputs and avoid integration (default false).
  //! This will make "regular" path planning much faster.
  bool Holonomic;

  //! How much gap error is allowed for each element in bidirectional search
  MSLVector GapError; 

  //! A search tree (used by incremental planners, but included in 
  //! Planner base to allow GuiPlanner to handle all planners)
  MSLTree *T;  

  //! A second tree (if needed)
  MSLTree *T2;
  
  //! A graph to represent a roadmap (used by roadmap planners, but 
  //! included in Planner base to allow GuiPlanner to handle all planners)
  MSLGraph *Roadmap;  

  //! The times associated with a solution path
  list<double> TimeList;

  //! The states associated with a solution path
  list<MSLVector> StateList;

  //! The inputs associated with a solution path
  list<MSLVector> InputList;

  //! Number of nodes to generate in a single execution of Plan or Construct
  int NumNodes;
  
  //! Time step to use for incremental planners
  double PlannerDeltaT;

  //! A constructor that initializes data members.
  Planner(Problem *problem);

  ~Planner();

  //! Reset the planner
  void Reset();

  //! Generate a planning graph
  virtual void Construct() = 0;

  //! Attempt to solve an Initial-Goal query
  virtual bool Plan() = 0;

  //! Write roadmap or trees to a file
  virtual void WriteGraphs(ofstream &fout) = 0;

  //! Read roadmap or trees from a file
  virtual void ReadGraphs(ifstream &fin) = 0;

  //! Determine if the gap error is staisfied
  bool GapSatisfied(const MSLVector &x1, const MSLVector &x2);
};


class IncrementalPlanner: public Planner {
 public:
  //! A constructor that initializes data members.
  IncrementalPlanner(Problem *problem);

  ~IncrementalPlanner() {};

  //! Essentially do nothing (no precomputation for incremental planners)
  virtual void Construct();

  //! Convert a path in the graph to Path and Policy
  void RecordSolution(const list<MSLNode*> &glist,
		      const list<MSLNode*> &g2list);

  void RecordSolution(const list<MSLNode*> &glist);

  //! Write trees to a file
  virtual void WriteGraphs(ofstream &fout);

  //! Read trees from a file
  virtual void ReadGraphs(ifstream &fin);
};


class RoadmapPlanner: public Planner {
 public:
  //! A constructor that initializes data members.
  RoadmapPlanner(Problem *problem);

  ~RoadmapPlanner() {};

  //! Write roadmap to a file
  virtual void WriteGraphs(ofstream &fout);

  //! Read roadmap from a file
  virtual void ReadGraphs(ifstream &fin);
};


#endif



