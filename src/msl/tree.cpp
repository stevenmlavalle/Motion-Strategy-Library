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


#include "msl/tree.h"

// *********************************************************************
// *********************************************************************
// CLASS:     MSLNode class
//
// *********************************************************************
// *********************************************************************

ostream& operator<<(ostream& out, const MSLNode& n)
{
  out << n.id;
  if (n.parent)
    out << " " << n.parent->id;
  else
    out << " -1";
  out << " " << n.state << " " << n.input;
  out << "\n";

  return out;
}


ostream& operator<<(ostream& out, const list<MSLNode*>& L)
{
  list<MSLNode*>::iterator x;
  list<MSLNode*> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++)
    out << " " << **x;
  return out;
}




MSLNode::MSLNode() {
}


MSLNode::MSLNode(void* pninfo) {
  info = pninfo;
}


MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u) {
  state = x;
  input = u;
  parent = pn;
  time = 1.0; // Make up a default
  cost = 0.0;
}


MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u, double t) {
  state = x;
  input = u;
  parent = pn;
  time = t;
  cost = 0.0;
}

MSLNode::MSLNode(MSLNode *pn, const MSLVector &x, const MSLVector &u, double t, void* pninfo) {
  state = x;
  input = u;
  parent = pn;
  time = t;
  cost = 0.0;

  info = pninfo;
}


// *********************************************************************
// *********************************************************************
// CLASS:     MSLTree class
//
// *********************************************************************
// *********************************************************************


ostream& operator<< (ostream& os, const MSLTree& T) {
  list<MSLNode*>::iterator x;
  list<MSLNode*> vl;
  vl = T.nodes;
  os << T.size << "\n";
  for (x = vl.begin(); x != vl.end(); x++)
    os << **x;
  return os;
}



istream& operator>> (istream& is, MSLTree & T) {
  int i,nid,pid,tsize;
  MSLVector x,u;
  MSLNode *pnode,*n; // Parent node

  T.Clear();

  is >> tsize;
  cout << "Loading a tree that has " << tsize << " nodes\n";
  for (i = 0; i < tsize; i++) {
    is >> nid >> pid >> x >> u;
    if (pid == -1) {
      T.MakeRoot(x);
      T.Root()->SetID(0);
    }
    else {
      n = T.Extend(T.Root(),x,u);
      n->SetCost((double)pid);
      n->SetID(nid);
    }

    list<MSLNode*> vl;
    vl = T.nodes;
    list<MSLNode*>::iterator nx;
    for (nx = vl.begin(); nx != vl.end(); nx++) {
      n = *nx;
      if (n->ID() != 0) {
	pid = (int)(n->Cost());  // Use the cost as a parent id (horrible)
	pnode = T.FindNode(pid);
	n->SetParent(pnode);
      }
    }
  }

  return is;
}


MSLTree::MSLTree() {
  root = NULL;
  size = 0;
}


MSLTree::MSLTree(const MSLVector &x) {
  MSLVector u;

  root = new MSLNode(NULL,x,u,0.0);
  root->id = 0;
  nodes.push_back(root);
  size = 1;
}


MSLTree::MSLTree(const MSLVector &x, void* nodeinfo) {
  MSLVector u;

  root = new MSLNode(NULL,x,u,0.0,nodeinfo);
  root->id = 0;
  nodes.push_back(root);
  size = 1;
}


MSLTree::~MSLTree() {
  Clear();
}


void MSLTree::MakeRoot(const MSLVector &x) {
  MSLVector u;

  if (!root) {
    root = new MSLNode(NULL,x,u,0.0);
    root->id = 0;
    nodes.push_back(root);
  }
  else
    cout << "Root already made.  MakeRoot has no effect.\n";
  size = 1;
}


MSLNode* MSLTree::Extend(MSLNode *parent, const MSLVector &x, const MSLVector &u) {
  MSLNode *nn;

  nn = new MSLNode(parent, x, u);
  nn->id = size;
  nodes.push_back(nn);
  size++;

  return nn;
}



MSLNode* MSLTree::Extend(MSLNode *parent, const MSLVector &x,
			 const MSLVector &u,
			 double time) {
  MSLNode *nn;

  nn = new MSLNode(parent, x, u, time);
  nn->id = size;
  nodes.push_back(nn);
  size++;

  return nn;
}


MSLNode* MSLTree::Extend(MSLNode *parent, const MSLVector &x,
			 const MSLVector &u,
			 double time, void* pninfo) {
  MSLNode *nn;

  nn = new MSLNode(parent, x, u, time, pninfo);
  nn->id = size;
  nodes.push_back(nn);
  size++;

  return nn;
}



MSLNode* MSLTree::FindNode(int nid) {
  list<MSLNode*>::iterator ni;

  for(ni = nodes.begin(); ni != nodes.end(); ni++) {
    if ((*ni)->id == nid)
      return *ni;
  }

  return NULL; // Indicates failure
}



list<MSLNode*> MSLTree::PathToRoot(MSLNode *n) {
  list<MSLNode*> nl;
  MSLNode *ni;

  ni = n;
  while (ni != root) {
    nl.push_back(ni);
    ni = ni->Parent();
  }

  nl.push_back(root);

  return nl;
}


void MSLTree::Clear() {
  list<MSLNode*>::iterator n;
  for (n = nodes.begin(); n != nodes.end(); n++)
    delete *n;
  nodes.clear();
  root = NULL;
}
