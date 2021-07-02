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

#include "msl/fdp.h"


// This is needed because g++ does not support ANSI export keyword!
#include "msl/marray.h"


// *********************************************************************
// *********************************************************************
// CLASS:     FDP base class
//
// *********************************************************************
// *********************************************************************

FDP::FDP(Problem *problem): IncrementalPlanner(problem) {

  GridDefaultResolution = 10; // 50 is too big
  Reset();
}



void FDP::Reset() {
  int i,dim;
  bool done;

  IncrementalPlanner::Reset();

  SatisfiedCount = 0;
  NumNodes = 20000;

  GridDimensions = vector<int>(P->StateDim);
  for (i = 0; i < P->StateDim; i++)
    GridDimensions[i] = GridDefaultResolution;
  if (is_file(P->FilePath + "GridDimensions")) {
    ifstream fin((P->FilePath + "GridDimensions").c_str());
    for (i = 0; i < P->StateDim; i++) {
      fin >> dim;
      GridDimensions[i] = dim;
    }
    fin.close();
  }

  Grid = new MultiArray<int>(GridDimensions, 0);

  Quantization = MSLVector(P->StateDim);

  for (i = 0; i < P->StateDim; i++) {
    Quantization[i] = (P->UpperState[i] - P->LowerState[i])/
      (GridDimensions[i]);
  }

  if (!is_file(P->FilePath+"GapError")) {
    for (i = 0; i < P->StateDim; i++)
      GapError[i] = Quantization[i] / 2.0;
  }

  // Mark the collision region
  vector<int> indices(P->StateDim);
  for (i = 0; i < P->StateDim; i++)
    indices[i] = 0;

  // Loop through all of the indices and check each for collision
  cout << "Performing collision detection to initialize grid.\n";
  done = false;
  while (!done) {
    (*Grid)[indices] =
      (P->Satisfied(IndicesToState(indices))) ? UNVISITED : COLLISION;
    done = Grid->Increment(indices); // This modifies indices
  }
  cout << "Finished.\n";
}



bool FDP::Plan()
{
  int i;
  MSLNode *n,*nn;
  MSLVector nx,x;
  double ptime;
  list<MSLNode*> path;
  double cost;
  vector<int> indices;
  list<MSLVector>::iterator u;
  list<MSLNode*>::iterator ni;
  list<MSLVector> ulist;

  // Make the root node of G
  if (!T) {
    T = new MSLTree(P->InitialState);
    T->Root()->SetCost(0.0);
    Q.push(T->Root());  // Add the root node to the queue
  }

  i = 0;
  while ((i < NumNodes)&&
	 (!Q.empty())) {

    // Remove the element with smallest cost
    n = Q.top();
    cost = n->Cost();
    Q.pop();
    x = n->State();

    // Try all inputs
    ulist = P->GetInputs(x);
    forall(u,ulist) {
      nx = P->Integrate(x,*u,PlannerDeltaT);
      indices = StateToIndices(nx);
      // If we are visiting an UNVISITED place...
      if ((*Grid)[indices] == UNVISITED) {
	(*Grid)[indices] = VISITED;
	// Make a new node and edge
	nn = T->Extend(n,nx,*u,PlannerDeltaT); // Make a new node with state nx
	nn->SetCost(SearchCost(cost,n,nn));
	Q.push(nn); // Put it into the priority queue
	//cout << "New node: " << nn->State() << "  " << nn->Cost() << "\n";

	// Check if goal reached
	if (GapSatisfied(nn->State(),P->GoalState)) {
	  cout << "Successful Path Found\n";
	  path = T->PathToRoot(nn); path.reverse();
	  // Make the correct times
	  ptime = 0.0; TimeList.clear();
	  forall(ni,path) {
	    ptime += (*ni)->Time();
	    TimeList.push_back(ptime);
	  }

	  RecordSolution(path); // Write to Path and Policy
	  return true;
	}
      }
    }

    i++;
  }

  cout << "Failure to find a path\n";
  return false;
}


double FDP::SearchCost(double initcost, MSLNode* &n, MSLNode* &nn) {
  return initcost + PlannerDeltaT;
}




vector<int> FDP::StateToIndices(const MSLVector &x) {
  int i;

  vector<int> indices(P->StateDim);

  for (i = 0; i < P->StateDim; i++) {
    indices[i] = (int) ((x[i] - P->LowerState[i])/Quantization[i]);
    // These are needed because the state is sometimes out of bounds.
    // The other way to fix this is to ensure that every instance of
    // Integrate in Model and subclasses stays within bounds.
    if (indices[i] < 0)
      indices[i] = 0;
    if (indices[i] >= GridDimensions[i])
      indices[i] = GridDimensions[i] - 1;
  }

  return indices;
}


MSLVector FDP::IndicesToState(const vector<int> &indices) {
  int i;

  MSLVector x(P->StateDim);
  for (i = 0; i < P->StateDim; i++) {
    x[i] = Quantization[i]*(indices[i]+0.5) + P->LowerState[i];
  }

  return x;
}


// *********************************************************************
// *********************************************************************
// CLASS:     FDPStar
//
// *********************************************************************
// *********************************************************************

FDPStar::FDPStar(Problem *problem): FDP(problem) {
}


// The cost always represents the cost to come using the Metric
// for each step, plus the cost to go from nn to the goal.
double FDPStar::SearchCost(double initcost, MSLNode* &n, MSLNode* &nn) {
  return initcost -
    P->Metric(n->State(),P->GoalState) +
    P->Metric(n->State(),nn->State()) +
    P->Metric(nn->State(),P->GoalState);
}



// *********************************************************************
// *********************************************************************
// CLASS:     FDPBestFirst
//
// *********************************************************************
// *********************************************************************

FDPBestFirst::FDPBestFirst(Problem *problem): FDP(problem) {
}


double FDPBestFirst::SearchCost(double initcost, MSLNode* &n,
				MSLNode* &nn) {
  return P->Metric(n->State(),P->GoalState);
}





// *********************************************************************
// *********************************************************************
// CLASS:     FDPBi
//
// *********************************************************************
// *********************************************************************

FDPBi::FDPBi(Problem *problem): FDP(problem) {
}



void FDPBi::Reset() {
  FDP::Reset();
}



bool FDPBi::Plan()
{
  int i,k;
  MSLNode *n,*nn,*nn2;
  MSLVector nx,x;
  double ptime;
  list<MSLNode*> path,nlist;
  double cost;
  vector<int> indices,indices2;
  bool match;
  list<MSLVector>::iterator u;
  list<MSLNode*>::iterator ni;
  list<MSLVector> ulist;

  // Make the root node of T
  if (!T) {
    T = new MSLTree(P->InitialState);
    T->Root()->SetCost(0.0);
    Q.push(T->Root());  // Add the root node to the queue
  }

  // Make the root node of T2
  if (!T2) {
    T2 = new MSLTree(P->GoalState);
    T2->Root()->SetCost(0.0);
    Q2.push(T2->Root());  // Add the root node to the queue
  }

  i = 0;
  while ((i < NumNodes)&&
	 (!Q.empty())&&
	 (!Q2.empty())) {

    // ******** Handle the tree from the initial state *************
    // Remove the element with smallest cost
    n = Q.top();
    cost = n->Cost();
    Q.pop();
    x = n->State();

    // Try all inputs
    ulist = P->GetInputs(x);
    forall(u,ulist) {
      nx = P->Integrate(x,*u,PlannerDeltaT);
      indices = StateToIndices(nx);

      // If we are visiting a place visited by T2...
      if ((*Grid)[indices] == VISITED2) {
	// Make a new node and edge
	nn = T->Extend(n,nx,*u,PlannerDeltaT); // Make a new node with state nx
	nn->SetCost(SearchCost(cost,n,nn));

	// Get the node in T2 that was visited
	nlist = T2->Nodes();
	forall(ni,nlist) {
	  indices2 = StateToIndices((*ni)->State());
	  match = true;
	  for (k = 0; k < P->StateDim; k++)
	    if (indices[k] != indices2[k])
	      match = false;
	  if (match)
	    nn2 = (*ni);
	}
	RecoverSolution(nn,nn2);
	cout << "Successful Path Found\n";
	return true;
      }

      // If we are visiting an UNVISITED place...
      if ((*Grid)[indices] == UNVISITED) {
	(*Grid)[indices] = VISITED;
	// Make a new node and edge
	nn = T->Extend(n,nx,*u,PlannerDeltaT); // Make a new node with state nx
	nn->SetCost(SearchCost(cost,n,nn));
	Q.push(nn); // Put it into the priority queue

	// Check if goal reached
	if (GapSatisfied(nn->State(),P->GoalState)) {
	  cout << "Successful Path Found\n";
	  path = T->PathToRoot(nn); path.reverse();
	  // Make the correct times
	  ptime = 0.0; TimeList.clear();
	  forall(ni,path) {
	    ptime += (*ni)->Time();
	    TimeList.push_back(ptime);
	  }

	  RecordSolution(path); // Write to Path and Policy
	  return true;
	}
      }
    }

    // ******** Handle the tree from the goal state *************
    // Remove the element with smallest cost
    n = Q2.top();
    cost = n->Cost();
    Q2.pop();
    x = n->State();

    // Try all inputs
    ulist = P->GetInputs(x);
    forall(u,ulist) {
      nx = P->Integrate(x,*u,-PlannerDeltaT);  // Reverse time integration
      indices = StateToIndices(nx);

      // If we are visiting a place visited by T...
      if ((*Grid)[indices] == VISITED) {
	// Make a new node and edge
	nn = T2->Extend(n,nx,*u,PlannerDeltaT); // Make new node with state nx
	nn->SetCost(SearchCost(cost,n,nn));

	// Get the node in T that was visited
	nlist = T->Nodes();
	forall(ni,nlist) {
	  indices2 = StateToIndices((*ni)->State());
	  match = true;
	  for (k = 0; k < P->StateDim; k++)
	    if (indices[k] != indices2[k])
	      match = false;
	  if (match)
	    nn2 = (*ni);
	}
	RecoverSolution(nn2,nn);
	cout << "Successful Path Found\n";
	return true;
      }

      // If we are visiting an UNVISITED place...
      if ((*Grid)[indices] == UNVISITED) {
	(*Grid)[indices] = VISITED2;
	// Make a new node and edge
	nn = T2->Extend(n,nx,*u,PlannerDeltaT); // Make new node with state nx
	nn->SetCost(SearchCost(cost,n,nn));
	Q2.push(nn); // Put it into the priority queue

	// Check if initial reached
	if (GapSatisfied(nn->State(),P->InitialState)) {
	  cout << "Successful Path Found\n";
	  path = T2->PathToRoot(nn); path.reverse();
	  // Make the correct times
	  ptime = 0.0; TimeList.clear();
	  forall(ni,path) {
	    ptime += (*ni)->Time();
	    TimeList.push_back(ptime);
	  }

	  RecordSolution(path); // Write to Path and Policy
	  return true;
	}
      }
    }

    i++;
  }

  cout << "Failure to find a path\n";
  return false;
}


// This is duplicated from RRTDual
void FDPBi::RecoverSolution(MSLNode* &n1, MSLNode* &n2) {
  list<MSLNode*> path,path2;
  double ptime;
  list<MSLNode*>::iterator n;

  path = T->PathToRoot(n1); path.reverse();
  // Figure out forward timings
  ptime = 0.0; TimeList.clear();
  forall(n,path) {
    ptime += (*n)->Time();
    TimeList.push_back(ptime);
  }

  path2 = T2->PathToRoot(n2);
  // Figure out backwards timings
  ptime += PlannerDeltaT; // Add a time step for the gap
  forall(n,path2)  { // Compute total time in T2 part of path
    TimeList.push_back(ptime);
    ptime += (*n)->Time();
  }

  //cout << "Path: " << path;
  //cout << "Path2: " << path2;
  //cout << "\n TimeList: " << TimeList << " L: " << TimeList.length();

  RecordSolution(path,path2); // Write to Path and Policy
}
