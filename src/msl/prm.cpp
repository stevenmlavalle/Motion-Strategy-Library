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

#include "msl/prm.h"
#include "msl/defs.h"


// These are used for quasi-random sampling
static int Primes[] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,
		       61,67,71,73,79,83,89,97,101,103,107,109,113,127,
		       131,137,139,149,151,157,163,167,173};


// *********************************************************************
// *********************************************************************
// CLASS:     PRM base class
//
// *********************************************************************
// *********************************************************************

PRM::PRM(Problem *problem): RoadmapPlanner(problem) {
  MaxEdgesPerVertex = 20;
  MaxNeighbors = 20;

  READ_PARAMETER_OR_DEFAULT(Radius,20.0);

  SatisfiedCount = 0;
  QuasiRandom = is_file(P->FilePath + "QuasiRandom");

  //! Choose Hammersley (which is better) or Halton sequence for quasi-random points
  QuasiRandomHammersley = true;

  if (!Holonomic)
    cout << "WARNING: Differential constraints will be ignored.\n";
}




list<MSLVertex*> PRM::NeighboringVertices(const MSLVector &x) {
  double d;
  MSLVertex n;
  list<MSLVertex*> best_list,all_vertices;
  list<MSLVertex*>::iterator vi;
  int k;

  all_vertices = Roadmap->Vertices();

  best_list.clear();
  k = 0;
  forall(vi,all_vertices) {
    d = P->Metric((*vi)->State(),x);
    if ((d < Radius)&&(k < MaxNeighbors)) {
      best_list.push_back(*vi);
      k++;  // Count the number of attempted connections
    }
  }

  //cout << "Number of Neighbors: " << best_list.size() << "\n";

  return best_list;
}




// This function essentially iterates Extend until it has to stop

bool PRM::Connect(const MSLVector &x1, const MSLVector &x2, MSLVector &u_best) {
  double a;
  int k;
  MSLVector x;

  k = (int) (P->Metric(x1,x2) / StepSize);

  //cout << "metric: " << P->Metric(x1,x2) <<
  //  "    stepsize: " << StepSize << "\n";

  if (k == 0)
    return true;

  for (a = 1.0/(k+1); a < 1.0; a += 1.0/(k+1) ) {
    x = P->InterpolateState(x1,x2,a);
    SatisfiedCount++;
    if (!P->Satisfied(x))
      return false;
  }

  return true;
}



void PRM::Construct()
{
  int i,k;
  MSLVector u_best,nx;
  MSLVertex *nn;
  list<MSLVertex*> nhbrs;
  list<MSLVertex*>::iterator ni;

  float t = used_time();

  if (!Roadmap)
    Roadmap = new MSLGraph();

  // Set the step size
  StepSize = P->Metric(P->InitialState,P->Integrate(P->InitialState,
 	     P->GetInputs(P->InitialState).front(),PlannerDeltaT));

  i = 0;
  while (i < NumNodes) {
    nx = ChooseState(i,NumNodes,P->InitialState.dim());
    SatisfiedCount++;
    i++;
    while (!P->Satisfied(nx)) {
      nx = ChooseState(i,NumNodes,P->InitialState.dim());
      SatisfiedCount++;
      i++;
      // Keep trying until a good sample is found
    }
    nhbrs = NeighboringVertices(nx);
    nn = Roadmap->AddVertex(nx);

    k = 0;
    forall(ni,nhbrs) {
      if (Connect((*ni)->State(),nx,u_best)) {
	Roadmap->AddEdge(nn,*ni,u_best,1.0);
	Roadmap->AddEdge(*ni,nn,-1.0*u_best,1.0);
	k++;
      }
      if (k > MaxEdgesPerVertex)
	break;
    }

    if (Roadmap->NumVertices() % 1000 == 0)
      cout << Roadmap->NumVertices() << " vertices in the PRM.\n";
  }

  //MSLVertex_array<int> labels(G);
  //c = COMPONENTS(G,labels); // Compute connected components

  /* Show component sizes
  int count;
  for (i = 0; i < c; i++) {
    count = 0;
    forall_MSLVertexs(n,G) {
      if (labels[n] == i)
	count++;
    }
    cout << "Component " << i << " has " << count << " MSLVertexs.\n";
  }
  */

  cout << "PRM Vertices: " << Roadmap->NumVertices() <<
    "  Edges: " << Roadmap->NumEdges() <<
    "  ColDet: " << SatisfiedCount;
  //cout <<  "  Components: " << c;
  cout << "\n";

  CumulativeConstructTime += ((double)used_time(t));
  cout << "Construct Time: " << CumulativeConstructTime << "s\n";

}



bool PRM::Plan()
{
  list<MSLVertex*> nlist;
  list<MSLEdge*> elist;
  MSLVertex *n,*ni,*ng,*nn,*n_best;
  MSLVector u_best;
  bool success;
  list<MSLVertex*> vpath;
  list<MSLVertex*>::iterator vi;
  list<MSLEdge*>::iterator ei;
  priority_queue<MSLVertex*,vector<MSLVertex*>,MSLVertexGreater> Q;
  double cost,mincost,time;

  float t = used_time();

  if (!Roadmap) {
    cout << "Empty roadmap.  Run Construct before Plan.\n";
    return false;
  }

  // Set the step size
  StepSize = P->Metric(P->InitialState,P->Integrate(P->InitialState,
 	     P->GetInputs(P->InitialState).front(),PlannerDeltaT));

  // Connect to the initial state
  nlist = NeighboringVertices(P->InitialState);

  if (nlist.size() == 0) {
    cout << "No neighboring vertices to the Initial State\n";
    cout << "Planning Time: " << ((double)used_time(t)) << "s\n";
    return false;
  }

  vi = nlist.begin();
  while ((!success) && (vi != nlist.end())) {
    success = Connect(P->InitialState,(*vi)->State(),u_best);
    vi++;
  }

  if (!success) {
    cout << "Failure to connect to Initial State\n";
    cout << "Planning Time: " << ((double)used_time(t)) << "s\n";
    return false;
  }

  // Make ni a new vertex in the PRM
  n = *vi;
  ni = Roadmap->AddVertex(P->InitialState);
  Roadmap->AddEdge(n,ni,u_best,1.0);
  Roadmap->AddEdge(ni,n,-1.0*u_best,1.0);

  // Connect to the goal state
  nlist = NeighboringVertices(P->GoalState);

  if (nlist.size() == 0) {
    cout << "No neighboring vertices to the Goal State\n";
    cout << "Planning Time: " << ((double)used_time(t)) << "s\n";
    return false;
  }

  vi = nlist.begin();
  while ((!success) && (vi != nlist.end())) {
    success = Connect((*vi)->State(),P->GoalState,u_best);
    vi++;
  }

  if (!success) {
    cout << "Failure to connect to Goal State\n";
    cout << "Planning Time: " << ((double)used_time(t)) << "s\n";
    return false;
  }

  // Make ng a new MSLVertex in the PRM
  n = *vi;
  ng = Roadmap->AddVertex(P->GoalState);
  Roadmap->AddEdge(n,ng,u_best,1.0);
  Roadmap->AddEdge(ng,n,-1.0*u_best,1.0);

  // Initialize for DP search (the original PRM used A^*)
  ni->SetCost(0.0);
  Q.push(ni);
  nlist = Roadmap->Vertices();
  forall(vi,nlist)
    (*vi)->Unmark();
  ni->Mark();

  // Loop until Q is empty or goal is found
  success = false;
  while ((!success)&&(!Q.empty())) {
    // Remove smallest element
    n = Q.top();
    cost = n->Cost();
    Q.pop();

    // Expand its unexplored neighbors
    elist = n->Edges();
    forall(ei,elist) {
      nn = (*ei)->Target();
      if (!nn->IsMarked()) { // If not yet visited
	nn->Mark();
	nn->SetCost(cost+(*ei)->Cost());
	Q.push(nn);
      }
    }
  }

  CumulativePlanningTime += ((double)used_time(t));
  cout << "Planning Time: " << CumulativePlanningTime << "s\n";

  if (ng->IsMarked()) {
    // Get the path
    n = ng;
    while (n != ni) {
      mincost = INFINITY;
      vpath.push_front(n);
      // Pick neighboring vertex with lowest cost
      elist = n->Edges();
      forall(ei,elist) {
	nn = (*ei)->Target();
	if (nn->Cost() < mincost) {
	  n_best = nn;
	  mincost = nn->Cost();
	}
      }
      n = n_best;
    }
    vpath.push_front(ni);
  }
  else {
    cout << "  Failure to find a path in the graph.\n";
    return false;
  }

  // Make the solution
  Path.clear();
  TimeList.clear();
  time = 0.0;
  forall(vi,vpath) {
    Path.push_back((*vi)->State());
    TimeList.push_back(time);
    time += 1.0;
  }

  cout << "  Success\n";

  return true;
}



MSLVector PRM::ChooseState(int i, int maxnum, int dim) {
  if (QuasiRandom) {
    if (QuasiRandomHammersley)
      return QuasiRandomStateHammersley(i,maxnum,dim);
    else
      return QuasiRandomStateHalton(i,dim);
  }
  else
    return RandomState();
}




MSLVector PRM::QuasiRandomStateHammersley(int i, int maxnum, int dim) {
  int j,k,r,ppow;
  MSLVector qrx;

  if (dim > 30) {
    cout << "ERROR: Dimension too high for quasi-random samples\n";
    exit(-1);
  }

  qrx = MSLVector(dim);

  k = i;

  qrx[0] = ((double) k / maxnum) * (P->UpperState[0] - P->LowerState[0]);
  qrx[0] += P->LowerState[0];
  for (j = 1; j < dim; j++) {
    qrx[j] = 0.0;
    ppow = Primes[j-1];
    k = i;
    while (k != 0) {
      r = k % Primes[j-1];
      k = (int) (k / Primes[j-1]);
      qrx[j] += (double) r / ppow;
      ppow *= Primes[j-1];
    }
    qrx[j] *= (P->UpperState[j] - P->LowerState[j]);
    qrx[j] += P->LowerState[j];
  }

  return qrx;
}


MSLVector PRM::QuasiRandomStateHalton(int i, int dim) {
  int j,k,r,ppow;
  MSLVector qrx;

  if (dim > 30) {
    cout << "ERROR: Dimension too high for quasi-random samples\n";
    exit(-1);
  }

  qrx = MSLVector(dim);
  k = i;

  for (j = 0; j < dim; j++) {
    qrx[j] = 0.0;
    ppow = Primes[j];
    k = i;
    while (k != 0) {
      r = k % Primes[j];
      k = (int) (k / Primes[j]);
      qrx[j] += (double) r / ppow;
      ppow *= Primes[j];
    }
    qrx[j] *= (P->UpperState[j] - P->LowerState[j]);
    qrx[j] += P->LowerState[j];
  }

  return qrx;
}
