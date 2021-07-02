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


#include "msl/graph.h"


// *********************************************************************
// *********************************************************************
// CLASS:     MSLVertex class
//
// *********************************************************************
// *********************************************************************

MSLVertex::MSLVertex() {
}


MSLVertex::MSLVertex(const MSLVector &x) {
  state = x;
  cost = 0.0;
}


MSLVertex::~MSLVertex() {
  edges.clear();
}

/*ostream& operator<<(ostream& out, const list<MSLVertex*>& L)
{
  list<MSLVertex*>::iterator x;
  list<MSLVertex*> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++)
    out << " " << **x;
  return out;
}
*/

ostream& operator<<(ostream& out, const MSLVertex& n)
{
  out << n.ID() << " " << n.State();
  out << "\n";

  return out;
}


/*istream& operator>>(istream& in, list<MSLVertex*>& L)
{
  L.clear();
  //  MSLVertex *x;
  for(;;)
    {
      char c;
      while (in.get(c) && isspace(c));
      if (!in) break;
      in.putback(c);
      //  x = new MSLVertex(); in >> x; L.push_back(x);
    }
  return in;
}
*/


// *********************************************************************
// *********************************************************************
// CLASS:     MSLEdge class
//
// *********************************************************************
// *********************************************************************

MSLEdge::MSLEdge() {
}


MSLEdge::MSLEdge(MSLVertex* v1, MSLVertex* v2) {
  // Make sure the node edge lists point correctly
  v1->edges.push_back(this);
  v2->edges.push_back(this);

  source = v1; target = v2;
  time = 1.0; cost = 1.0;
}


MSLEdge::MSLEdge(MSLVertex* v1, MSLVertex* v2, const MSLVector &u,
		 double t) {
  v1->edges.push_back(this);
  v2->edges.push_back(this);

  source = v1; target = v2;
  input = u; time = t; cost = 1.0;
}



MSLEdge::MSLEdge(MSLVertex* v1, MSLVertex* v2, double t) {
  v1->edges.push_back(this);
  v2->edges.push_back(this);

  source = v1; target = v2;
  time = t; cost = 1.0;
}


/*ostream& operator<<(ostream& out, const list<MSLEdge*>& L)
{
  list<MSLEdge*>::iterator x;
  list<MSLEdge*> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++)
    out << " " << **x;
  return out;
}
*/

ostream& operator<<(ostream& out, const MSLEdge& e)
{
  out << e.Source()->ID() << " " << e.Target()->ID();
  out << "\n";

  return out;
}


// *********************************************************************
// *********************************************************************
// CLASS:     MSLGraph class
//
// *********************************************************************
// *********************************************************************

MSLGraph::MSLGraph() {
  numvertices = 0;
  numedges = 0;
}



MSLGraph::~MSLGraph() {
  Clear();
}


MSLVertex* MSLGraph::AddVertex(const MSLVector &x) {
  MSLVertex *nv;

  nv = new MSLVertex(x);
  nv->id = numvertices;
  vertices.push_back(nv);
  numvertices++;

  return nv;
}


MSLEdge* MSLGraph::AddEdge(MSLVertex* v1, MSLVertex* v2) {
  MSLVector u;
  return AddEdge(v1,v2,u,1.0);
}


MSLEdge* MSLGraph::AddEdge(MSLVertex* v1, MSLVertex* v2, double time) {
  MSLVector u;
  return AddEdge(v1,v2,u,time);
}


MSLEdge* MSLGraph::AddEdge(MSLVertex* v1, MSLVertex* v2,
			   const MSLVector &u, double time) {

  MSLEdge* e;
  e = new MSLEdge(v1,v2,u,time);

  edges.push_back(e);
  numedges++;

  return e;
}


bool MSLGraph::IsEdge(MSLVertex* v1, MSLVertex* v2) {

  list<MSLEdge*> elist;
  elist = v1->Edges();

  list<MSLEdge*>::iterator e;
  for (e = elist.begin(); e != elist.end(); e++) {
    if ((*e)->Target() == v2)
      return true;
  }

  return false;
}



MSLVertex* MSLGraph::FindVertex(int nid) {
  list<MSLVertex*>::iterator vi;

  for(vi =  vertices.begin(); vi != vertices.end(); vi++) {
    if ((*vi)->id == nid)
      return *vi;
  }

  return NULL; // Indicates failure
}



void MSLGraph::Clear() {
  list<MSLVertex*>::iterator v;
  for (v = vertices.begin(); v != vertices.end(); v++)
    delete *v;
  vertices.clear();

  list<MSLEdge*>::iterator e;
  for (e = edges.begin(); e != edges.end(); e++)
    delete *e;
  edges.clear();
}


ostream& operator<< (ostream& os, const MSLGraph& G) {
  os << G.NumVertices() << " " << G.NumEdges() << "\n";
  list<MSLVertex*>::iterator x;
  list<MSLVertex*> vl;
  vl = G.Vertices();
  for (x = vl.begin(); x != vl.end(); x++)
    os << " " << **x;

  os << "\n";
  list<MSLEdge*>::iterator y;
  list<MSLEdge*> el;
  el = G.Edges();
  for (y = el.begin(); y != el.end(); y++)
    os << " " << **y;
  os << "\n\n";

  return os;
}


istream& operator>> (istream& is, MSLGraph & G) {
  int i,nid1,nid2,nume,numv;
  MSLVector x,u;
  MSLVertex *v1,*v2;

  G.Clear();

  is >> numv >> nume;
  cout << "Loading a graph that has " << numv
       << " vertices and " << nume << " edges\n";

  // Handle vertices
  for (i = 0; i < numv; i++) {
    is >> nid1 >> x;
    v1 = G.AddVertex(x);
    v1->SetID(nid1);
  }

  // Handle edges
  for (i = 0; i < nume; i++) {
    is >> nid1 >> nid2;
    v1 = G.FindVertex(nid1);
    v2 = G.FindVertex(nid2);
    if (v1 && v2)
      G.AddEdge(v1,v2);
    else
      cout << "ERROR:  Read edge does not connect to vertices\n";
  }

  return is;
}
