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


#define BACKTREE_ANALYSIS          1
#define FOREWARDTREE_ANALYSIS      2
#define NON_ANALYSIS               0

#include <math.h>
#include <stdio.h>

#include "msl/vector.h"
#include "msl/rrt.h"
#include "msl/nodeinfo.h"
#include "msl/rcrrt.h"
#include "msl/defs.h"


RCRRT::RCRRT(Problem *problem): RRTGoalBias(problem) {
  MSLVector counter;
  MSLVector state;

  //! Assume a solution exists
  issolutionexist = true;

  inputset = P->GetInputs();

  inputnum = inputset.size();

  //! Initial exploration information, all the inputs marked as unexplored
  initexploreinfo = MSLVector(inputnum);
  for(int i=0; i<inputnum; i++) initexploreinfo[i] = 0.0;

  initcoltend = 0.0;

}

//! true : if exists unexplored inputs
//!        biasvalue will be the collision tendency
//! false: all the inputs have been explored
//!
bool RCRRT::IsNodeExpanded(MSLNode* x, double& biasvalue,
			    bool forward)
{
  MSLNodeInfo* nodeinfo;
  MSLVector counter;

  //! Get extra information from node x
  nodeinfo = (MSLNodeInfo*) x->GetInfo();

  //! Get the exploration information
  counter = nodeinfo->GetExplorationInfo();

  biasvalue = nodeinfo->GetCollisionTendency();

  //! Check if existing unexplored inputs
  for(int j = 0; j < inputnum; j++)
    {
      //! if one input has not been tried, it is satisfied
      if(counter[j]<1) return true;
    }

  return false;
}


//! Check if the ith input is explored or not
bool RCRRT::IsInputApplied(const int& inputindex, const MSLVector& exploreinfo)
{
  return (exploreinfo[inputindex-1] < 1);
}



void RCRRT::BackWardBiasSet(MSLNode* n, MSLTree* t)
{
  int backstep;
  MSLNodeInfo *nodeinfo;
  MSLNode *n1, *n2;
  MSLVector counter;
  MSLVector input;
  double biasvalue, coltend;

  n1 = n;
  backstep = 2;

  biasvalue = BiasValue(backstep);

  //!!!!!!!!!! the backward information updata should go back to the root
  //!  while(biasvalue > 0.000001)
  //!  {
  //! Check if the current node is the root
  while(n1 != t->Root())
    {
      //! get the parent node
      n2 = n1->Parent();

      //! get the counter for the parent node
      nodeinfo = (MSLNodeInfo*) n2->GetInfo();
      coltend = nodeinfo->GetCollisionTendency();


      //! increase the corresponding counter by 1
      coltend = coltend + biasvalue;
      if(coltend > 1.0) coltend = 1.0;

      //! save the counter information back
      nodeinfo->SetCollisionTendency(coltend);
      n2->SetInfo(nodeinfo);

      n1 = n2;

      backstep ++;
      biasvalue = BiasValue(backstep);
    }
}


double RCRRT::BiasValue(int backstep)
{
  return pow(1.0/inputnum, backstep);
}



//! Select the nearestneighbor node according to given metric
//! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//! This function is metric dependent
//! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//! Choose the nestest neighbor in the search structure
//! If all nodes are explored, return (NULL) node
//! If there exist unexpanded nodes
//! (1) return the closest node with probability equal collisiontendency
//! (2) or, return the closed node when (2) return nothing
MSLNode* RCRRT::SelectNode(const MSLVector &x,
			   MSLTree* t,
			   bool forward = true)
{
  MSLNode *n_best, *n_best1;
  MSLVector counter;
  list<MSLNode*>::iterator niter;

  double d, d_min, d_min1;
  double biasvalue;
  double r;

  n_best = NULL;
  n_best1 = NULL;

  d_min1 = INFINITY;
  d_min = INFINITY;
  d = 0.0;

  list<MSLNode*> nl;
  nl = t->Nodes();
  forall(niter,nl) {
    if(IsNodeExpanded((*niter), biasvalue, forward))  {
      d = (forward) ? P->Metric((*niter)->State(),x) : P->Metric(x,(*niter)->State());
      if(d < d_min1)  {
	d_min1 = d;  n_best1 = (*niter);
      }

      R >> r;

      if(r>biasvalue) {
	if(d < d_min) {
	  d_min = d;  n_best = (*niter);
	}
      }
    }
  }

  if (d_min != INFINITY)
    return n_best;
  else
    return n_best1;
}


//! Extend the nearest node to the random state return true: a new node
//! is generated return false: 1. all the successors of the chosen node
//! are in collision 2. no nearest node is chosen because all of them
//! are explored
bool RCRRT::Extend(const MSLVector &x, MSLTree* t,
		    MSLNode*& nn, bool forward = true) {
  MSLNodeInfo* nodeinfo;
  MSLNode* n_best;
  MSLVector nx,u_best;
  bool success;

  //! if all the state space is explored, return false and set 'nn' not NULL
  if((n_best = SelectNode(x, t, forward)) == NULL) {
    nn = new MSLNode();
    return false;
  }


  nodeinfo = (MSLNodeInfo*) n_best->GetInfo();

  u_best = SelectInput(n_best, x, nx, success, forward);

  nodeinfo = (MSLNodeInfo*) n_best->GetInfo();

  if (success) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    nn = t->Extend(n_best, nx, u_best, PlannerDeltaT, nodeinfo);
  }
  else nn = NULL;

  return success;
}


//! Choose the best input and return the new state and uncontrolled state
MSLVector RCRRT::SelectInput(MSLNode* n1,
			     const MSLVector &x2,
			     MSLVector& nx_best,
			     bool& success,
			     bool forward = true)
{
  MSLNodeInfo* nodeinfo;
  MSLVector u_best,nx;
  MSLVector state;
  MSLVector counter;
  list<MSLVector>::iterator uiter;

  double coltend;
  double d,d_min;
  int u_best_index;
  int inputindex;

  u_best_index = 0;

  //! read the state for this node
  state = n1->State();
  success = false;

  //! set d_min to inifinity in order to make for each try to extend the tree be success
  d_min = INFINITY;

  //! read the counter information of the current node n1
  nodeinfo = (MSLNodeInfo*) n1->GetInfo();
  counter = nodeinfo->GetExplorationInfo();
  coltend = nodeinfo->GetCollisionTendency();

  inputindex = 1;

  forall(uiter,inputset) {
    //! if ith input is not failed, try it
    if(IsInputApplied(inputindex, counter)) {

      //!!!!!!!!!!!!!!!! need to modify the intergrate function to
      //!include the uncontrolled state
      if (forward)
	nx = P->Integrate(state, *uiter, PlannerDeltaT);
      else
	nx = P->Integrate(state, *uiter, -PlannerDeltaT);

      d = (forward) ? P->Metric(nx, x2) : P->Metric(x2, nx);

      //! Check if this input leads to collision

      //! If it does not lead to collision, record its index
      if (P->Satisfied(nx)) {
	if(d<d_min) {
	  //! if the new state is satisfied and closer, keep this information
	  d_min = d; u_best = *uiter; nx_best = nx; success = true;
	  u_best_index = inputindex;
	}
      }
      //! If it leads to collision, set it as expanded
      //!
      else {
	if(forward) BackWardBiasSet(n1, T);
	else BackWardBiasSet(n1, T2);
	coltend = coltend + 1.0 / inputnum;
	counter[inputindex-1] = 1;
      }
    }

    inputindex++;
  }

  if (success) {
    counter[u_best_index-1] = 1;
  }

  //! save the counter information
  nodeinfo->SetCollisionTendency(coltend);
  nodeinfo->SetExplorationInfo(counter);
  n1->SetInfo(nodeinfo);

  return u_best;
}


//! method to increase the extending time because the more time the
//! RCRRT extend in each step, more quickly the overall information of
//! state space is gotten
bool RCRRT::Connect(const MSLVector &x, MSLTree* t,
		     MSLNode*& nn, bool forward = true)
{
  MSLNodeInfo *nodeinfo;
  MSLNode *nn_prev, *n_best;
  MSLVector nx,nx_prev,u_best;
  bool success;
  double d, d_prev, clock;
  int steps;

  //! if all the states are explored, return false, same as extend method
  if((n_best = SelectNode(x, t, forward)) == NULL) {
    nn = new MSLNode();
    return false;
  }

  u_best = SelectInput(n_best, x, nx, success, forward);

  steps = 0;

  //! nx gets next state
  if (success) {
    d = P->Metric(nx,x); d_prev = d;
    nx_prev = nx; //! Initialize
    nn = n_best;
    clock = PlannerDeltaT;
    while ((P->Satisfied(nx))&&
	   (clock <= ConnectTimeLimit)&&
	   (d <= d_prev))
      {
        SatisfiedCount++;
	steps++; //! Number of steps made in connecting
	nx_prev = nx;
	d_prev = d; nn_prev = nn;

	if (forward)
	  nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
	else
	  nx = P->Integrate(nx_prev,u_best,-PlannerDeltaT);

	d = P->Metric(nx,x);
	clock += PlannerDeltaT;
      }

    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    nn = t->Extend(n_best, nx_prev, u_best, steps*PlannerDeltaT, nodeinfo);
  }
  return success;
}



bool RCRRT::Plan()
{
  MSLNodeInfo* nodeinfo;
  MSLNode *n,*nn,*n_goal;
  MSLVector nx,u_best;
  MSLVector counter;
  list<MSLNode*> path;

  int i;
  double d;

  float t = used_time();

  //! Make the root node of G
  if (!T) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T = new MSLTree(P->InitialState, nodeinfo);
  }

  i = 0;
  n = SelectNode(P->GoalState, T);
  n_goal = n;

  GoalDist = P->Metric(n->State(),P->GoalState);

  if(!issolutionexist)
    {
      cout << "Solution does not exists to this resolution and control period" << endl;
      return false;
    }

  while (issolutionexist &&
	 (!GapSatisfied(n_goal->State(),P->GoalState))) {
    if (Extend(ChooseState(), T, nn, true)) {
      d = P->Metric(nn->State(),P->GoalState);
      if (d < GoalDist) {
	GoalDist = d;
	BestState = nn->State();
	n_goal = nn;
      }
    }
    else
      if(nn != NULL){
	cout << "No solution exists for the current resolution and tolerance" << endl;
	issolutionexist = false;
      }
    i++;
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  //! Get the solution path
  if (GapSatisfied(n_goal->State(),P->GoalState)) {
    cout << "Successful Path Found at " << i <<  " iterations" << endl;
    path = T->PathToRoot(n_goal);
    path.reverse();
    RecordSolution(path);
    return true;
  }
  else {
    cout << "Failure to connect\n";
    return false;
  }
}





RCRRTDual::RCRRTDual(Problem *p):RCRRT(p) {

}


void RCRRTDual::RecoverSolution(MSLNode* n1, MSLNode* n2) {
  list<MSLNode*> path,path2;

  path = T->PathToRoot(n1);
  path.reverse();

  path2 = T2->PathToRoot(n2);

  //!cout << "Path: " << path;
  //!cout << "Path2: " << path2;

  RecordSolution(path,path2); //! Write to Path and Policy

}


bool RCRRTDual::Plan()
{
  MSLNodeInfo* nodeinfo;
  MSLVector rx,u_best,nx;
  MSLNode *nn,*nn2;

  int i;
  bool connected;

  float t = used_time();

  //! Make the root node of G
  if (!T) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T = new MSLTree(P->InitialState, nodeinfo);
  }

  if (!T2) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T2 = new MSLTree(P->GoalState, nodeinfo);
  }

  nn = T->Root();
  nn2 = T2->Root();

  i = 0;
  connected = false;

  if(!issolutionexist)
    {
      cout << "Solution does not exists to this resolution and control period" << endl;
      return false;
    }

  while ((i < NumNodes) && (!connected)) {
    rx = ChooseState();

    if (Extend(rx,T,nn,true)) {
      if (Extend(rx,T2,nn2,false)) {
	i++;
	connected = GetConnected(nn, nn2);
      }
      else if(nn2 != NULL){
	cout << "No solution exists for the current resolution and tolerance" << endl;
	issolutionexist = false;
      }
    }
    else if(nn != NULL){
      cout << "No solution exists for the current resolution and tolerance" << endl;
      issolutionexist = false;
    }

    i++;
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  if (!connected)
    cout << "Failure to connect after " << T->Size()+T2->Size()
  	 << " nodes\n";

  cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

  return connected;
}


bool RCRRTDual::GetConnected(MSLNode* n1, MSLNode* n2)
{
  list<MSLNode*> nl;
  list<MSLNode*>::iterator niter;

  nl = T2->Nodes();
  forall(niter, nl)
    if (GapSatisfied(n1->State(),(*niter)->State())) {
      cout << "CONNECTED!!  Nodes: " <<
	T->Size()+T2->Size() << "\n";
      RecoverSolution(n1,(*niter));
      return true;
    }

  nl = T->Nodes();
  forall(niter, nl)
    if (GapSatisfied((*niter)->State(), n2->State())) {
      cout << "CONNECTED!!  Nodes: " <<
	T->Size()+T2->Size() << "\n";
      RecoverSolution((*niter),n2);
      return true;
    }

  return false;
}




RCRRTExtExt::RCRRTExtExt(Problem *p):RCRRTDual(p) {
}


bool RCRRTExtExt::Plan()
{
  MSLNodeInfo* nodeinfo;
  MSLVector rx,u_best,nx;
  MSLNode *nn,*nn2;

  bool connected;
  int i;

  float t = used_time();

  //! Make the root node of G
  if (!T) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T = new MSLTree(P->InitialState, nodeinfo);
  }

  if (!T2) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T2 = new MSLTree(P->GoalState, nodeinfo);
  }

  nn = T->Root();
  nn2 = T2->Root();

  i = 0;
  connected = false;

  if(!issolutionexist) {
    cout << "Solution does not exists to this resolution and control period" << endl;
    return false;
  }

  while (issolutionexist && (!connected)) {
    if (Extend(ChooseState(),T,nn,true)) {
      if (Extend(nn->State(),T2,nn2,false)) {
	//!    if (Connect(ChooseState(),G,nn)) {
	//!      if (Connect(G.inf(nn),G2,nn2,false)) {
	i++;
	connected = GetConnected(nn, nn2);
      }
    }
    else if(nn != NULL) {
      cout << "No solution exists for the current resolution and tolerance" << endl;
      issolutionexist = false;
    }

    i++;

    if ((!connected) && (Extend(ChooseState(),T2,nn,false))) {
      if (Extend(nn->State(),T,nn2)) {
	//!    if ((!connected) && (Connect(ChooseState(),G2,nn,false))) {
	//!      if (Connect(G.inf(nn),G,nn2)) {
	i++;
	connected = GetConnected(nn2, nn);
      }
      else if(nn2 != NULL) {
	cout << "No solution exists for the current resolution and tolerance" << endl;
	issolutionexist = false;
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



RCRRTBall::RCRRTBall(Problem *p):RCRRT(p) {

  BallRadius = 0.02;
  FailNumTh = 10;
  FailNum = 0;

}


MSLNode* RCRRTBall::SelectNode(const MSLVector &x, MSLTree* t,
			       bool forward = true) {
  MSLNode *n,*n_best,*n_best1;
  MSLVector counter;
  list<MSLNode*>::iterator niter;

  double d,d_min,d_min1;
  double biasvalue;
  bool inball;
  double r;

  n_best = NULL;
  n_best1 = NULL;

  d_min1 = INFINITY;
  d_min = INFINITY;
  d = 0.0;

  inball = false;

  list<MSLNode*> nl;
  nl = t->Nodes();
  forall(niter,nl) {

    d = (forward) ? P->Metric((*niter)->State(),x) : P->Metric(x,(*niter)->State());

    //! Check if the random point is in some balls
    if(d<BallRadius) inball = true;

    if(IsNodeExpanded(n, biasvalue, forward)) {
      if(d < d_min1) {
	d_min1 = d;  n_best1 = n;
      }

      R >> r;

      if(r>biasvalue) {
	if(d < d_min) {
	  d_min = d;  n_best = n;
	}
      }
    }
  }

  //! check if the random point is in some ball
  if(inball) FailNum ++;
  else FailNum = 0;

  if (d_min != INFINITY)
    return n_best;
  else
    return n_best1;
}



//! Extend the nearest node to the random state
bool RCRRTBall::Extend(const MSLVector &x, MSLTree* t,
		        MSLNode*& nn, bool forward = true) {
  MSLNodeInfo* nodeinfo;
  MSLNode *n_best;
  MSLVector nx,u_best;
  list<MSLNode*>::iterator niter;
  list<MSLNode*> nl;

  bool success;
  bool inball;
  double d;

  if((n_best = SelectNode(x, t, forward)) == NULL) {
    nn = new MSLNode();
    return false;
  }

  u_best = SelectInput(n_best, x, nx, success, forward);

  //! Check if the new state is in some balls
  //! YES: do nothing
  //! NO:  add the new state as a new node
  if (success) {
    inball = false;
    nl = t->Nodes();
    forall(niter,nl) {
      d = (forward) ? P->Metric((*niter)->State(),nx) : P->Metric(nx,(*niter)->State());
      if(d<BallRadius) inball = true;
    }

    if(!inball) {
      nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
      nn = t->Extend(n_best, nx, u_best, PlannerDeltaT, nodeinfo);
    }
    else return false;
  }

  return success;
}



bool RCRRTBall::Connect(const MSLVector &x, MSLTree* t,
			 MSLNode*& nn, bool forward = true)
{
  MSLNodeInfo *nodeinfo;
  MSLNode *nn_prev, *n_best;
  MSLVector nx,nx_prev,u_best;
  list<MSLNode*>::iterator niter;
  list<MSLNode*> nl;

  bool success;
  double clock, d, d_prev;
  int steps;
  bool inball;

  inball = false;

  if((n_best = SelectNode(x, t, forward)) == NULL) {
    nn = new MSLNode();
    return false;
  }

  u_best = SelectInput(n_best, x, nx, success, forward);

  steps = 0;

  //! nx gets next state
  if (success) {
    d = P->Metric(nx,x); d_prev = d;
    nx_prev = nx; //! Initialize
    nn = n_best;
    clock = PlannerDeltaT;
    while ((P->Satisfied(nx))&&
	   (clock <= ConnectTimeLimit)&&
	   (d <= d_prev))
      {
        SatisfiedCount++;
	steps++; //! Number of steps made in connecting
	nx_prev = nx;
	d_prev = d; nn_prev = nn;

	if (forward)
	  nx = P->Integrate(nx_prev,u_best,PlannerDeltaT);
	else
	  nx = P->Integrate(nx_prev,u_best,-PlannerDeltaT);

	d = P->Metric(nx,x);
	clock += PlannerDeltaT;
      }

    nl = t->Nodes();
    forall(niter,nl) {
      d = (forward) ? P->Metric((*niter)->State(),nx) : P->Metric(nx,(*niter)->State());
      if(d<BallRadius) inball = true;
    }

    if(!inball) {
      nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
      nn = t->Extend(n_best, nx, u_best, PlannerDeltaT, nodeinfo);
    }
    else return false;
  }

  return success;
}



bool RCRRTBall::Plan()
{
  MSLNodeInfo* nodeinfo;
  MSLNode *n,*nn,*n_goal;
  MSLVector nx,u_best;
  MSLVector counter;
  list<MSLNode*> path;

  int i;
  double d;
  bool isfail;

  isfail = false;

  float t = used_time();

  //! Make the root node of G
  if (!T) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T = new MSLTree(P->InitialState, nodeinfo);
  }

  i = 0;
  n = SelectNode(P->GoalState,T);
  n_goal = n;

  GoalDist = P->Metric(n->State(),P->GoalState);

  if(!issolutionexist)
    {
      cout << "Solution does not exists to this resolution and control period" << endl;
      return false;
    }

  while ((i < NumNodes)&&(!GapSatisfied(n_goal->State(),P->GoalState))
	 && ! isfail) {
    //!    if (Connect(ChooseState(), G, nn, true)) {
    if (Extend(ChooseState(), T, nn, true)) {
      d = P->Metric(nn->State(),P->GoalState);
      if (d < GoalDist) {
	GoalDist = d;
	BestState = nn->State();
	n_goal = nn;
      }
    }
    else if(nn != NULL){
      cout << "No solution exists for the current resolution and tolerance" << endl;
      issolutionexist = false;
    }

    i++;
    if(FailNum>FailNumTh) isfail = true;
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  //! Get the solution path
  if (GapSatisfied(n_goal->State(),P->GoalState)) {
    cout << "Successful Path Found at " << i <<  " iterations" << endl;
    path = T->PathToRoot(n_goal);
    path.reverse();
    RecordSolution(path);
    return true;
  }
  else {
    cout << "Failure to connect\n";
    return false;
  }
}




RCRRTBallDual::RCRRTBallDual(Problem *p):RCRRTBall(p) {

}


void RCRRTBallDual::RecoverSolution(MSLNode* n1, MSLNode* n2) {
  list<MSLNode*> path,path2;

  path = T->PathToRoot(n1);
  path.reverse();

  path2 = T2->PathToRoot(n2);

  RecordSolution(path,path2); //! Write to Path and Policy

}


bool RCRRTBallDual::Plan()
{
  MSLNodeInfo* nodeinfo;
  MSLVector rx,u_best,nx;
  MSLNode *nn,*nn2;

  int i;
  bool connected;

  float t = used_time();

  //! Make the root node of G
  if (!T) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T = new MSLTree(P->InitialState, nodeinfo);
  }

  if (!T2) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T2 = new MSLTree(P->GoalState, nodeinfo);
  }

  nn = T->Root();
  nn2 = T2->Root();

  i = 0;
  connected = false;

  if(!issolutionexist)
    {
      cout << "Solution does not exists to this resolution and control period" << endl;
      return false;
    }

  while ((i < NumNodes) && (!connected)) {
    rx = ChooseState();

    if(!(Extend(rx, T, nn) && Extend(rx, T2, nn2, false))) {
      cout << "No solution exists for the current resolution and tolerance" << endl;
      issolutionexist = false;
    }

    connected = GetConnected(nn, nn2);

    i++;
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  if (!connected)
    cout << "Failure to connect after " << T->Size()+T2->Size()
  	 << " nodes\n";

  cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

  return connected;
}


bool RCRRTBallDual::GetConnected(MSLNode* n1, MSLNode* n2) {
  list<MSLNode*> nl;
  list<MSLNode*>::iterator niter;

  nl = T2->Nodes();
  forall(niter, nl)
    if (GapSatisfied(n1->State(),(*niter)->State())) {
      cout << "CONNECTED!!  Nodes: " <<
	T->Size()+T2->Size() << "\n";
      RecoverSolution(n1,(*niter));
      return true;
    }

  nl = T->Nodes();
  forall(niter, nl)
    if (GapSatisfied((*niter)->State(), n2->State())) {
      cout << "CONNECTED!!  Nodes: " <<
	T->Size()+T2->Size() << "\n";
      RecoverSolution((*niter),n2);
      return true;
    }

  return false;
}



RCRRTBallExtExt::RCRRTBallExtExt(Problem *p):RCRRTBallDual(p) {
}


bool RCRRTBallExtExt::Plan()
{
  MSLNodeInfo* nodeinfo;
  MSLVector u_best,nx,nx2;
  MSLNode *nn,*nn2;

  int i;
  bool connected;

  float t = used_time();

  //! Make the root node of G
  if (!T) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T = new MSLTree(P->InitialState, nodeinfo);
  }

  if (!T2) {
    nodeinfo = new MSLNodeInfo(initexploreinfo, initcoltend);
    T2 = new MSLTree(P->GoalState, nodeinfo);
  }

  nn = T->Root();
  nn2 = T2->Root();

  i = 0;
  connected = false;

  if(!issolutionexist) {
    cout << "Solution does not exists to this resolution and control period" << endl;
    return false;
  }

  while ((i < NumNodes) && (!connected)) {
    if (Extend(ChooseState(),T,nn,true)) {
      if (Extend(nn->State(),T2,nn2,false)) {
	//!    if (Connect(ChooseState(),G,nn)) {
	//!      if (Connect(G.inf(nn),G2,nn2,false)) {
	i++;
	connected = GetConnected(nn, nn2);
      }
      else {
	cout << "No solution exists for the current resolution and tolerance" << endl;
	issolutionexist = false;
      }
    }
    else {
      cout << "No solution exists for the current resolution and tolerance" << endl;
      issolutionexist = false;
    }

    i++;

    if ((!connected) && (Extend(ChooseState(),T2,nn,false))) {
      if (Extend(nn->State(),T,nn2)) {
	i++;
	connected = GetConnected(nn2, nn);
      }
      else {
	cout << "No solution exists for the current resolution and tolerance" << endl;
	issolutionexist = false;
      }
    }
    else {
      cout << "No solution exists for the current resolution and tolerance" << endl;
      issolutionexist = false;
    }

    i++;
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  if (!connected)
    cout << "Failure to connect after " << T->Size()+T2->Size()
	 << " nodes\n";

  cout << "Collision Detection Calls: " << SatisfiedCount << "\n";

  return connected;
}
