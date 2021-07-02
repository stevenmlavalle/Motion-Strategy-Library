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
#include <stdio.h>

#include "msl/planner.h"
#include "msl/defs.h"


// *********************************************************************
// *********************************************************************
// CLASS:     Planner base class
//
// *********************************************************************
// *********************************************************************

Planner::Planner(Problem *problem):Solver(problem) {
  T = NULL;
  T2 = NULL;
  Roadmap = NULL;
  Reset();
}


Planner::~Planner() {
  Reset();
}

void Planner::Reset() {
  int i;

  NumNodes = 1000;
  std::ifstream fin;

  READ_PARAMETER_OR_DEFAULT(PlannerDeltaT,1.0);

  GapError = MSLVector(P->StateDim);
  for (i = 0; i < P->StateDim; i++)
    GapError[i] = 1.0;
  READ_OPTIONAL_PARAMETER(GapError);

  Path.clear();
  Policy.clear();

  fin.open((FilePath+"Holonomic").c_str());
  Holonomic = fin ? true : false; // Nonholonomic by default
  fin.close();

  CumulativePlanningTime = 0.0;
  CumulativeConstructTime = 0.0;

  if (T)
    delete T;
  if (T2)
    delete T2;
  T = NULL;
  T2 = NULL;

  if (Roadmap)
    delete Roadmap;
  Roadmap = NULL;
}


MSLVector Planner::RandomState() {
  int i;
  double r;
  MSLVector rx;

  rx = P->LowerState;
  for (i = 0; i < P->StateDim; i++) {
      R >> r;
      rx[i] += r * (P->UpperState[i] - P->LowerState[i]);
    }

  return rx;
}



MSLVector Planner::NormalState(MSLVector mean, double sd = 0.5) {
  int i,j;
  double r;
  MSLVector rx;
  bool success = false;

  rx = mean;
  for (i = 0; i < P->StateDim; i++) {
    success = false;
    while (!success) {
      rx[i] = 0.0;
      for (j = 0; j < 12; j++) {  // Increase 12 here and below for more accuracy
	R >> r; rx[i] += r;
      }
      rx[i] = (rx[i] - 12/2)*sd*(P->UpperState[i]-P->LowerState[i])+mean[i];
      if ((rx[i] <= P->UpperState[i])&&(rx[i] >= P->LowerState[i]))
	success = true;
    }
  }

  return rx;
}



bool Planner::GapSatisfied(const MSLVector &x1, const MSLVector &x2) {
  MSLVector x;
  int i;

  x = P->StateDifference(x1,x2);
  for (i = 0; i < P->StateDim; i++) {
    if (fabs(x[i]) > GapError[i])
      return false;
  }

  return true;
}



// *********************************************************************
// *********************************************************************
// CLASS:     IncrementalPlanner base class
//
// *********************************************************************
// *********************************************************************

IncrementalPlanner::IncrementalPlanner(Problem *problem):Planner(problem) {
}

void IncrementalPlanner::Construct() {
  cout << "  Incremental planners do not use Construct.\n";
  cout << "  Try Plan.\n";
}


void IncrementalPlanner::RecordSolution(const list<MSLNode*> &glist,
					const list<MSLNode*> &g2list)
{
  list<MSLNode*>::const_iterator n,nfirst,nlast;
  double ptime;

  Path.clear();
  Policy.clear();

  ptime = 0.0; TimeList.clear();
  nfirst = glist.begin();

  forall(n,glist) {
    Path.push_back((*n)->State());
    if (n != nfirst) {
      Policy.push_back((*n)->Input());
    }
    ptime += (*n)->Time();
    TimeList.push_back(ptime);
  }

  // The GapState is always comes from last node in glist
  GapState = Path.back();

  // Make a dummy input to get from GapState to the next state
  Policy.push_back(P->GetInputs().front());

  if (g2list.size() == 0) {
    // Push the goal state onto the end (jumps the gap)
    Path.push_back(P->GoalState);
  }
  else { // Using two graphs
    ptime += PlannerDeltaT; // Add a time step for the gap
    nlast = g2list.end();
    forall(n,g2list) {
      Path.push_back((*n)->State());
      if (n != nlast) {
	Policy.push_back((*n)->Input());
      }
    TimeList.push_back(ptime);
    ptime += (*n)->Time();
    }
  }

  //cout << "Path: " << Path << "\n";
  //cout << "Policy: " << Policy << "\n";
  //cout << "TimeList: " << TimeList << "\n";
}



void IncrementalPlanner::RecordSolution(const list<MSLNode*> &glist)
{
  list<MSLNode*> emptylist;
  emptylist.clear(); // Make sure it is clear

  RecordSolution(glist,emptylist);
}



void IncrementalPlanner::WriteGraphs(ofstream &fout)
{
  if (T)
    fout << *T << "\n\n\n";
  if (T2)
    fout << *T2;
}



void IncrementalPlanner::ReadGraphs(ifstream &fin)
{
  if (T)
    delete T;
  if (T2)
    delete T2;

  T = new MSLTree();
  T2 = new MSLTree();

  fin >> *T;
  cout << "T \n" << *T << endl;
  fin >> *T2;
}




// *********************************************************************
// *********************************************************************
// CLASS:     RoadmapPlanner base class
//
// *********************************************************************
// *********************************************************************

RoadmapPlanner::RoadmapPlanner(Problem *problem):Planner(problem) {
}




void RoadmapPlanner::WriteGraphs(ofstream &fout)
{
  fout << *Roadmap << "\n\n\n";
}



void RoadmapPlanner::ReadGraphs(ifstream &fin)
{
  if (Roadmap)
    delete Roadmap;

  Roadmap = new MSLGraph();
  fin >> *Roadmap;
}
