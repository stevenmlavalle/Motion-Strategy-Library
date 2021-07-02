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

#include "msl/rrt.h"
#include "msl/defs.h"


// *********************************************************************
// *********************************************************************
// CLASS:     RRT base class
//
// *********************************************************************
// *********************************************************************

RRT::RRT(Problem *problem): IncrementalPlanner(problem) {

  UseANN = false;

  READ_PARAMETER_OR_DEFAULT(ConnectTimeLimit,INFINITY);

  Reset();
}



void RRT::Reset() {
  IncrementalPlanner::Reset();

  NumNodes = 1000;

  SatisfiedCount = 0;
  GoalDist = P->Metric(P->InitialState,P->GoalState);
  BestState = P->InitialState;

#ifdef USE_ANN
  MAG = MultiANN(&G);
  MAG2 = MultiANN(&G2);
#endif

}


// Return the best new state in nx_best
// success will be false if no action is collision free
MSLVector RRT::SelectInput(const MSLVector &x1, const MSLVector &x2,
			   MSLVector &nx_best, bool &success,
			   bool forward = true)
{
  MSLVector u_best,nx;
  list<MSLVector>::iterator u;
  double d,d_min;
  success = false;
  d_min = (forward) ? P->Metric(x1,x2) : P->Metric(x2,x1);
  list<MSLVector> il = P->GetInputs(x1);

  if (Holonomic) { // Just do interpolation
    u_best = P->InterpolateState(x1,x2,0.1) - x1;
    u_best = u_best.norm(); // Normalize the direction
    nx_best = P->Integrate(x1,u_best,PlannerDeltaT);
    SatisfiedCount++;
    if (P->Satisfied(nx_best))
      success = true;
  }
  else {  // Nonholonomic (the more general case -- look at Inputs)
    forall(u,il) {
      if (forward)
	nx = P->Integrate(x1,*u,PlannerDeltaT);
      else
	nx = P->Integrate(x1,*u,-PlannerDeltaT);

      d  = (forward) ? P->Metric(nx,x2): P->Metric(x2,nx);

      SatisfiedCount++;

      if ((d < d_min)&&(x1 != nx)) {
	if (P->Satisfied(nx)) {
	  d_min = d; u_best = *u; nx_best = nx; success = true;
	}
      }
    }
  }

  //cout << "u_best: " << u_best << "\n";
  //cout << "nx_best: " << nx_best << "\n";
  //cout << "success: " << success << "\n";
  //success = true;

  return u_best;
}




MSLNode* RRT::SelectNode(const MSLVector &x, MSLTree* t,
			 bool forward = true) {
  double d,d_min;
  MSLNode *n_best;
  list<MSLNode*>::iterator n;

  d_min = INFINITY; d = 0.0;

#ifdef USE_ANN
  if (!UseANN) {
#endif
    list<MSLNode*> nl;
    nl = t->Nodes();
    forall(n,nl) {
      d = (forward) ? P->Metric((*n)->State(),x) : P->Metric(x,(*n)->State());
      if (d < d_min) {
	d_min = d; n_best = (*n);
      }
    }
#ifdef USE_ANN
  }
  else {
    if (&g == &G)
      n_best = MAG.SelectNode(x);
    else
      n_best = MAG2.SelectNode(x);
  }
#endif

  //cout << "n_best: " << (*n_best) << "\n";

  return n_best;
}



bool RRT::Extend(const MSLVector &x,
		 MSLTree *t,
		 MSLNode *&nn, bool forward = true) {
  MSLNode *n_best;
  MSLVector nx,u_best;
  bool success;

  n_best = SelectNode(x,t,forward);
  u_best = SelectInput(n_best->State(),x,nx,success,forward);
  // nx gets next state
  if (success) {   // If a collision-free input was found
    // Extend the tree
    nn = t->Extend(n_best, nx, u_best, PlannerDeltaT);

    //cout << "n_best: " << n_best << "\n";
    //cout << "New node: " << nn << "\n";
  }

  return success;
}



// This function essentially iterates Extend until it has to stop
// The same action is used for every iteration
bool RRT::Connect(const MSLVector &x,
		  MSLTree *t,
		  MSLNode *&nn, bool forward = true) {
  MSLNode *nn_prev,*n_best;
  MSLVector nx,nx_prev,u_best;
  bool success;
  double d,d_prev,clock;
  int steps;

  n_best = SelectNode(x,t,forward);
  u_best = SelectInput(n_best->State(),x,nx,success,forward);
  steps = 0;
           // nx gets next state
  if (success) {   // If a collision-free input was found
    d = P->Metric(nx,x); d_prev = d;
    nx_prev = nx; // Initialize
    nn = n_best;
    clock = PlannerDeltaT;
    while ((P->Satisfied(nx))&&
	   (clock <= ConnectTimeLimit)&&
	   (d <= d_prev))
      {
        SatisfiedCount++;
	steps++; // Number of steps made in connecting
	nx_prev = nx;
	d_prev = d; nn_prev = nn;
	// Uncomment line below to select best action each time
	//u_best = SelectInput(g.inf(nn),x,nx,success,forward);
	if (Holonomic) {
	    nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
	}
	else { // Nonholonomic
	  if (forward)
	    nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
	  else
	    nx = P->Integrate(nx_prev,u_best,-PlannerDeltaT);
	}
	d = P->Metric(nx,x);
	clock += PlannerDeltaT;
	// Uncomment the subsequent two lines to
	//   make each intermediate node added
	//nn = g.new_node(nx_prev); // Make a new node
	//g.new_edge(nn_prev,nn,u_best);
      }
    nn = t->Extend(n_best, nx_prev, u_best, steps*PlannerDeltaT);

  }

  return success;
}



bool RRT::Plan()
{
  int i;
  double d;
  MSLNode *n,*nn,*n_goal;
  MSLVector nx,u_best;
  list<MSLNode*> path;

  // Keep track of time
  float t = used_time();

  // Make the root node of G
  if (!T)
    T = new MSLTree(P->InitialState);

  nn = T->Root();

  i = 0;
  n = SelectNode(P->GoalState,T);
  n_goal = n;

  GoalDist = P->Metric(n->State(),P->GoalState);
  while ((i < NumNodes)&&(!GapSatisfied(n_goal->State(),P->GoalState))) {
    if (Extend(ChooseState(),T,nn)) {
      d = P->Metric(nn->State(),P->GoalState);
      if (d < GoalDist) {  // Decrease if goal closer
	GoalDist = d;
	BestState = nn->State();
	n_goal = nn;
	//cout << "GoalDist " << GoalDist << "\n";
      }
    }
    i++;
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  // Get the solution path
  if (GapSatisfied(n_goal->State(),P->GoalState)) {
    cout << "Success\n";
    path = T->PathToRoot(n_goal);
    path.reverse();
    RecordSolution(path); // Write to Path and Policy
    return true;
  }
  else {
    cout << "Failure\n";
    return false;
  }
}



MSLVector RRT::ChooseState() {
  return RandomState();
}




// *********************************************************************
// *********************************************************************
// CLASS:     RRTGoalBias
//
// Occasionally pick the goal state instead of a random sample.
// *********************************************************************
// *********************************************************************


RRTGoalBias::RRTGoalBias(Problem *p):RRT(p) {
  READ_PARAMETER_OR_DEFAULT(GoalProb,0.05);
}



MSLVector RRTGoalBias::ChooseState()
{
  double rv;

  R >> rv;
  if (rv > GoalProb)
    return RandomState();
  else
    return P->GoalState;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RRTCon
//
// Make the regular RRT greedy by attempting to connect all the way
// to the random sample, or go as far as possible before colliding.
// *********************************************************************
// *********************************************************************


// Build an RRT that tries to go as far as it can for each edge
RRTCon::RRTCon(Problem *p):RRTGoalBias(p) {
  READ_PARAMETER_OR_DEFAULT(GoalProb,0.0);
}




bool RRTCon::Plan()
{
  int i;
  double d;
  MSLNode *n,*nn,*n_goal;
  MSLVector nx,u_best;
  list<MSLNode*> path;

  // Keep track of time
  float t = used_time();

  // Make the root node of G
  if (!T)
    T = new MSLTree(P->InitialState);


  i = 0;
  n = SelectNode(P->GoalState,T);
  n_goal = n;

  GoalDist = P->Metric(n->State(),P->GoalState);
  while ((i < NumNodes)&&(!GapSatisfied(n_goal->State(),P->GoalState))) {
    if (Connect(ChooseState(),T,nn)) {
      d = P->Metric(nn->State(),P->GoalState);
      if (d < GoalDist) {  // Decrease if goal closer
	GoalDist = d;
	BestState = nn->State();
	n_goal = nn;
	//cout << "GoalDist " << GoalDist << "\n";
      }
    }
    i++;
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  // Get the solution path
  if (GapSatisfied(n_goal->State(),P->GoalState)) {
    cout << "Successful Path Found\n";
    path = T->PathToRoot(n_goal);
    path.reverse();
    RecordSolution(path); // Write to Path and Policy
    return true;
  }
  else {
    cout << "Failure to connect after " <<
      T->Size() << " nodes\n";
    cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

    return false;
  }
}


// *********************************************************************
// *********************************************************************
// CLASS:     RRTDual
//
// The dual-tree planner used in LaValle, Kuffner, ICRA 1999.  Each
// tree is extended toward a randomly-sampled point.  RRTExtExt is
// generally better.
// *********************************************************************
// *********************************************************************

RRTDual::RRTDual(Problem *p):RRT(p) {
}


void RRTDual::RecoverSolution(MSLNode *n1, MSLNode *n2) {
  list<MSLNode*> path,path2;

  path = T->PathToRoot(n1);
  path.reverse();

  path2 = T2->PathToRoot(n2);

  //cout << "Path: " << path;
  //cout << "Path2: " << path2;

  RecordSolution(path,path2); // Write to Path and Policy
}



bool RRTDual::Plan()
{
  int i;
  MSLVector rx,u_best,nx;
  MSLNode *nn,*nn2;
  bool connected;

  connected = false;

  float t = used_time();

  //int total = 0;
  //for (int j=0; j < 100; j++) {

  if (!T)
    T = new MSLTree(P->InitialState);
  if (!T2)
    T2 = new MSLTree(P->GoalState);

  nn = T->Root();
  nn2 = T2->Root();

  i = 0;
  connected = false;
  while ((i < NumNodes) && (!connected)) {
    rx = ChooseState();
    Extend(rx,T,nn);
    Extend(rx,T2,nn2,false);  // false means reverse-time integrate

    if (GapSatisfied(nn->State(),nn2->State())) {
      cout << "CONNECTED!!  MSLNodes: " <<
	T->Size() + T2->Size() << "\n";
      connected = true;
      RecoverSolution(nn,nn2);
    }

    i++;
  }

  if (!connected)
    cout << "Failure to connect after " <<
      T->Size()+T2->Size()
	 << " nodes\n";

  cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

  //total += i;
  // }
  //cout << "Avg: " << total/100 << "\n";

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  return connected;
}





// *********************************************************************
// *********************************************************************
// CLASS:     RRTExtExt
//
// A dual-tree planner in which computation is balanced between
// exploring and connecting trees to each other.  Greedier verions
// of this are RRTExtCon and RRTConCon.
// *********************************************************************
// *********************************************************************

RRTExtExt::RRTExtExt(Problem *p):RRTDual(p) {
}


bool RRTExtExt::Plan()
{
  int i;
  MSLVector u_best,nx,nx2;
  MSLNode *nn,*nn2;
  bool connected;

  connected = false;

  float t = used_time();

  //int total = 0;
  //for (int j=0; j < 100; j++) {

  if (!T)
    T = new MSLTree(P->InitialState);
  if (!T2)
    T2 = new MSLTree(P->GoalState);

  nn = T->Root();
  nn2 = T2->Root();

  i = 0;
  connected = false;
  while ((i < NumNodes) && (!connected)) {
    if (Extend(ChooseState(),T,nn)) {
      if (Extend(nn->State(),T2,nn2,false)) {
	i++;
	if (GapSatisfied(nn->State(),nn2->State())) {
	  cout << "CONNECTED!!  MSLNodes: " <<
	    T->Size() + T2->Size() << "\n";
	  connected = true;
	  RecoverSolution(nn,nn2);
	}
      }
    }
    i++;

    if ((!connected) && (Extend(ChooseState(),T2,nn,false))) {
      if (Extend(nn->State(),T,nn2)) {
	i++;
	if (GapSatisfied(nn2->State(),nn->State())) {
	  cout << "CONNECTED!!  MSLNodes: " <<
	    T->Size() + T2->Size();
	  connected = true;
	  RecoverSolution(nn2,nn);
	}
      }
    }
  }

  i++;

  if (!connected)
    cout << "Failure to connect after " <<
      T->Size() + T2->Size()
	 << " nodes\n";

  cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

  //total += i;
  // }
  //cout << "Avg: " << total/100 << "\n";

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  return connected;
}




// *********************************************************************
// *********************************************************************
// CLASS:     RRTGoalZoom
//
// Bias sampling towards a region that contains the goal.  The
// region shrinks around the goal as the RRT comes nearer.  This
// planner is based on a class project at Iowa State by Jun Qu in 1999.
// *********************************************************************
// *********************************************************************

RRTGoalZoom::RRTGoalZoom(Problem *p):RRT(p) {
  GoalProb = 0.05;
  ZoomProb = 0.5;
  ZoomFactor = 2.0;
}


// Zoom using a square box
MSLVector RRTGoalZoom::ChooseState()
{
  double rv,r,diff;
  MSLVector zx;
  int i;

  R >> rv;
  diff = 0.0;
  zx = P->LowerState;  // Initialize to anything

  if (rv < GoalProb)
    return P->GoalState;
  else {
    if (rv < (ZoomProb+GoalProb)) {
      for (i = 0; i < P->GoalState.dim(); i++) {
	if (fabs(P->GoalState[i] - BestState[i]) > diff)
	  diff = fabs(P->GoalState[i] - BestState[i]);
      }
      for (i = 0; i < P->GoalState.dim(); i++) {
	R >> r;
	zx[i] += P->GoalState[i] - diff + 2.0*r*ZoomFactor*diff;
      }
      return zx;
    }
    else
      return RandomState();
  }

}


// *********************************************************************
// *********************************************************************
// CLASS:     RRTPolar
//
// Instead of random sampling, attempt to gradually bias samples
// toward the goal.
// *********************************************************************
// *********************************************************************

RRTPolar::RRTPolar(Problem *p):RRT(p) {
  // RadiusExp = 1.0/(P->InitialState.dim() - 1);
  RadiusExp = 1.0;
}






// This implementation ignores C-space topology, VERY BAD!
MSLVector RRTPolar::ChooseState()
{
  double r,w;
  MSLVector zx;
  int i,j;
  bool success;

  w = 0.0;
  zx = P->GoalState;  // Initialize to anything

  success = false;

  while (!success) {
    for (i = 0; i < P->GoalState.dim(); i++) {
      // Generate sample from N(0,1)
      zx[i] = 0.0;
      for (j = 0; j < 12; j++) {
	R >> r; zx[i] += r;
      }
      zx[i] -= 6.0;
      w += zx[i]*zx[i];
    }

    w = sqrt(w);

    //cout << "RadiusExp: " << RadiusExp;

    R >> r;  // Radius
    r = pow(r,RadiusExp);
    for (i = 0; i < P->GoalState.dim(); i++) {
      zx[i] = (P->UpperState[i] - P->LowerState[i])*
	sqrt(P->GoalState.dim())*r*zx[i]/w +
	P->GoalState[i];
    }

    // Check if sample is within bounds
    success = true;
    for (i = 0; i < P->GoalState.dim(); i++) {
      if ((zx[i] >= P->UpperState[i])||(zx[i] <= P->LowerState[i]))
	success = false;
    }
  }

  return zx;
}



MSLVector RRTPolar::SelectInput(const MSLVector &x1, const MSLVector &x2,
				 MSLVector &nx_best, bool &success)
{
  MSLVector u_best,nx;
  list<MSLVector>::iterator u;
  double d,dg,dmax,d_min;
  success = false;
  d_min = INFINITY;
  dg  = P->Metric(x1,x2);
  dmax  = P->Metric(P->LowerState,P->UpperState);
  list<MSLVector> il = P->GetInputs(x1);
  forall(u,il) {
    //nx = P->Integrate(x1,u,PlannerDeltaT*sqrt(dg/dmax));  // Slow down near goal
    nx = P->Integrate(x1,*u,PlannerDeltaT);
    d  = P->Metric(nx,x2);
    if ((d < d_min)&&(P->Satisfied(nx))&&(x1 != nx))
      {
	d_min = d; u_best = *u; nx_best = nx; success = true;
      }
  }

  //cout << "u_best: " << u_best << "\n");

  return u_best;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RRTHull
//
// A simple exploration of what happens to the RRT in a large disc.
// Only works for 2DPoint model!!!!!
// *********************************************************************
// *********************************************************************

RRTHull::RRTHull(Problem *p):RRT(p) {
  Radius = 100000000.0;
}



MSLVector RRTHull::ChooseState() {
  double theta;
  MSLVector v(2);

  R >> theta;  theta *= 2.0*PI;

  v[0] = Radius*cos(theta);
  v[1] = Radius*sin(theta);

  return v;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RRTExtCon
//
// The planner presented in Kuffner, LaValle, 2000.  There are two
// trees, and the attempt to connect them is greedy.
// *********************************************************************
// *********************************************************************

RRTExtCon::RRTExtCon(Problem *p):RRTDual(p) {
}



bool RRTExtCon::Plan()
{
  int i;
  MSLVector nx,nx_prev;
  MSLNode *nn,*nn2;
  bool connected;

  connected = false;

  float t = used_time();

  if (!T)
    T = new MSLTree(P->InitialState);
  if (!T2)
    T2 = new MSLTree(P->GoalState);

  nn = T->Root();
  nn2 = T2->Root();

  i = 0;
  connected = false;
  while ((i < NumNodes) && (!connected)) {
    if (Extend(ChooseState(),T,nn)) {
      // Update the goal RRT
      if (Connect(nn->State(),T2,nn2,false)) {
	if (GapSatisfied(nn->State(),nn2->State())) {
	  cout << "CONNECTED!!  MSLNodes: " <<
	    T->Size() + T2->Size()
	       << "\n";
	  connected = true;
	  RecoverSolution(nn,nn2);
	}
      }
    }
    i++;

    if ((!connected)&&(Extend(ChooseState(),T2,nn,false))) {
      // Update the initial RRT
      if (Connect(nn->State(),T,nn2)) {
	if (GapSatisfied(nn->State(),nn2->State())) {
	  cout << "CONNECTED!!  MSLNodes: " <<
	    T->Size() + T2->Size() << "\n";
	  connected = true;
	  RecoverSolution(nn2,nn);
	}
      }
    }
    i++;
  }

  if (!connected)
    cout << "Failure to connect after " << T->Size() + T2->Size()
	 << " nodes\n";

  cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  return connected;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RRTConCon
//
// One the best dual-tree planners.  Be greedy in growing the trees
// and in trying to connect them to each other.  This is usually the
// fastest planner, but there is often a price for being greedy...
// *********************************************************************
// *********************************************************************

RRTConCon::RRTConCon(Problem *p):RRTDual(p) {
}



bool RRTConCon::Plan()
{
  int i;
  MSLVector nx,nx_prev;
  MSLNode *nn,*nn2;
  bool connected;

  connected = false;

  // Keep track of time
  float t = used_time();

  if (!T)
    T = new MSLTree(P->InitialState);
  if (!T2)
    T2 = new MSLTree(P->GoalState);

  nn = T->Root();
  nn2 = T2->Root();

  i = 0;
  connected = false;

  while ((i < NumNodes) && (!connected)) {
    if (Connect(ChooseState(),T,nn)) {
      // Update the goal RRT
      //cout << "nn: " << nn->State() << "  nn2: " << nn2->State() << "\n";
      if (Connect(nn->State(),T2,nn2,false)) {
	if (GapSatisfied(nn->State(),nn2->State())) {
	  cout << "CONNECTED!!  MSLNodes: " <<
	    T->Size()+T2->Size() << "\n";
	  connected = true;
	  RecoverSolution(nn,nn2); // Defined in RRTDual
	}
      }
    }
    i++;

    if ((!connected)&&(Connect(ChooseState(),T2,nn,false))) {
      // Update the initial RRT
      if (Connect(nn->State(),T,nn2)) {
	if (GapSatisfied(nn->State(),nn2->State())) {
	  cout << "CONNECTED!!  MSLNodes: " <<
	    T->Size()+T2->Size() << "\n";
	  connected = true;
	  RecoverSolution(nn2,nn);
	}
      }
    }
    i++;
  }

  if (!connected)
    cout << "Failure to connect after " <<
      T->Size()+T2->Size() << " nodes\n";

  cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  return connected;
}


// *********************************************************************
// *********************************************************************
// CLASS:     RRTBidirBalanced
//
// This planner behaves similar to RRTConCon, except that a
// cardinality criteria is introduced to maintain relative balance
// between the number of nodes in each tree.  This planner was the
// first to be able to consistently solve the original well-known
// alpha puzzle motion planning benchmark.
// *********************************************************************
// *********************************************************************

RRTBidirBalanced::RRTBidirBalanced(Problem *p):RRTDual(p) {
}

bool RRTBidirBalanced::Plan()
{
  // Code for time trials
  float t = used_time();

  // init trees if needed
  MSLNode *nn,*nn2;
  if (!T)
    T = new MSLTree(P->InitialState);
  if (!T2)
    T2 = new MSLTree(P->GoalState);

  // initialize planner
  bool connected = false;
  nn = T->Root();
  nn2 = T2->Root();
  int i = 0;

  // set current active tree and target node
  bool bInitActive = true;

  MSLTree *pActiveTree = T;
  MSLTree *pOtherTree  = T2;
  MSLVector target = P->GoalState;

  while ((i < NumNodes) && (!connected))
  {
    if (Connect(target, pActiveTree, nn, bInitActive)) {
      if (Connect(nn->State(), pOtherTree, nn2, !bInitActive)) {
	if (GapSatisfied(nn->State(), nn2->State())) {
	  cout << "CONNECTED!!  MSLNodes: " <<
	    pActiveTree->Size() + pOtherTree->Size() << "\n";
	  connected = true;
	  if (bInitActive)
	    RecoverSolution(nn, nn2); // Defined in RRTDual
	  else
	    RecoverSolution(nn2, nn);
	}
      }
    }

    // select the active tree and new target
    if (!connected) {
      bInitActive = (T->Size() <= T2->Size());
      if (bInitActive)
	{ pActiveTree = T;  pOtherTree = T2; }
      else
	{ pActiveTree = T2; pOtherTree = T;  }
      target = ChooseState();
    }
    i++;
  }

  if (!connected)
    cout << "Failure to connect after " <<
      T->Size() + T2->Size() << " nodes\n";

  cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

  // Code for time trials
  //total += i;
  //  }
  //cout << "Avg: " << total/100.0 << "\n";
  //cout << "Avg time: " << ((double)used_time(t)/100.0) << "\n";
  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  return connected;
}



// *********************************************************************
// *********************************************************************
// CLASS:     RandomTree
//
// Technically not an RRT.  Pick a vertex and input at random.
// *********************************************************************
// *********************************************************************

RandomTree::RandomTree(Problem *p):RRT(p) {
}


MSLNode* RandomTree::SelectNode(const MSLVector &x, MSLTree *t,
				bool forward = true) {
  cout << "WARNING: RESULT NOT CORRECT!!!\n";
  cout << "NEED TO FIX RandomTree::SelectNode\n";

  //return g.choose_node();
  return t->Root();
}


MSLVector RandomTree::SelectInput(const MSLVector &x1, const MSLVector &x2,
				  MSLVector &nx_best, bool &success,
				  bool forward = true) {
  double r;
  list<MSLVector> il;
  int k;
  MSLVector u_best;

  il = P->GetInputs(x1);
  R >> r;

  k = (int) (r * il.size());

  cout << "WARNING: RESULT NOT CORRECT!!!\n";
  cout << "NEED TO FIX RandomTree::SelectInput\n";
  //u_best = il.inf(il[k]);
  u_best = il.front();

  nx_best = P->Integrate(x1,u_best,PlannerDeltaT);
  SatisfiedCount++;

  return u_best;

}
