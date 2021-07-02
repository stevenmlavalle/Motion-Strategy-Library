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
#
#ifndef MSL_TREE_H
#define MSL_TREE_H

#include <list>
#include <string>
using namespace std;



#include "vector.h"
#include "mslio.h"

class MSLTree;

class MSLNode {
 private:
  MSLVector state;
  MSLVector input;
  MSLNode* parent;
  list<MSLNode*> children;
  double time;
  double cost;
  int id;

  void* info;

 public:
  //! The state to which this node corresponds
  MSLVector State() const {return state; };

  //! The input vector that leads to this state from the parent
  inline MSLVector Input() const {return input; };

  inline MSLNode* Parent() {return parent; };
  inline void SetParent(MSLNode* p) {parent = p; };
  inline list<MSLNode*> const Children() {return children; };
  
  //! The time required to reach this node from the parent
  inline double Time() const {return time; };

  //! A cost value, useful in some algorithms
  inline double Cost() const {return cost; };

  //! A cost value, useful in some algorithms
  inline void SetCost(const double &x) {cost = x; };

  //! Change the node ID
  inline void SetID(const int &i) {id = i; };

  //! Get the node ID
  inline int ID() const {return id; };

  //! Get the information 
  void* GetInfo() {return info; };
  
  //! Set the information
  void SetInfo(void* in) {info = in; };


  MSLNode();
  MSLNode(void* pninfo);
  MSLNode(MSLNode* pn, const MSLVector &x, const MSLVector &u);
  MSLNode(MSLNode* pn, const MSLVector &x, const MSLVector &u, double t);
  MSLNode(MSLNode* pn, const MSLVector &x, const MSLVector &u, double t, void* pninfo);
  ~MSLNode() { children.clear(); };

  inline void AddChild(MSLNode *cn) { children.push_back(cn); }

  //friend istream& operator>> (istream& is, MSLNode& n);
  friend ostream& operator<< (ostream& os, const MSLNode& n);
  //friend istream& operator>> (istream& is, list<MSLNode*> & nl);
  friend ostream& operator<< (ostream& os, const list<MSLNode*> & nl);

  friend class MSLTree;
};


//! This is a comparison object to be used for STL-based sorting
class MSLNodeLess {
 public:
  bool operator() (MSLNode* p, MSLNode* q) const {
    return p->Cost() < q->Cost();
  }
};


//! This is a comparison object to be used for STL-based sorting
class MSLNodeGreater {
 public:
  bool operator() (MSLNode* p, MSLNode* q) const {
    return p->Cost() > q->Cost();
  }
};


class MSLTree {
 private:
  //  list<MSLNode*> nodes;
  //MSLNode* root;
  int size;
 public:
  list<MSLNode*> nodes;
  MSLNode* root;

  MSLTree();
  MSLTree(const MSLVector &x); // Argument is state of root node
  MSLTree(const MSLVector &x, void* nodeinfo);
  ~MSLTree();

  void MakeRoot(const MSLVector &x);
  MSLNode* Extend(MSLNode *parent, const MSLVector &x, const MSLVector &u);
  MSLNode* Extend(MSLNode *parent, const MSLVector &x, const MSLVector &u, 
		  double time);
  MSLNode* Extend(MSLNode *parent, const MSLVector &x, const MSLVector &u, 
		  double time, void* pninfo);

  list<MSLNode*> PathToRoot(MSLNode *n);
  MSLNode* FindNode(int nid);
  inline list<MSLNode*> Nodes() const { return nodes; };
  inline MSLNode* Root() {return root; };
  inline int Size() {return size;}

  void Clear();

  friend istream& operator>> (istream& is, MSLTree& n);
  friend ostream& operator<< (ostream& os, const MSLTree& n);
};

#endif

