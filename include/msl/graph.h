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


#ifndef MSL_GRAPH_H
#define MSL_GRAPH_H

//#include <list.h>
//#include <string>

#include "vector.h"
#include "mslio.h"

class MSLEdge;

class MSLVertex {
 private:
  MSLVector state;
  double cost;
  list<MSLEdge*> edges;
  int id;
  bool mark;

 public:
  //! The state to which this node corresponds
  MSLVector State() const {return state; };

  //! Return all of the incident edges
  inline list<MSLEdge*> Edges() const {return edges; };

  //! A cost value, useful in some algorithms
  inline double Cost() const {return cost; };

  //! A cost value, useful in some algorithms
  inline void SetCost(const double &x) {cost = x; };

  //! Change the vertex ID
  inline void SetID(const int &i) {id = i; };

  //! Get the vertex ID
  inline int ID() const {return id; };

  //! A cost value, useful in some algorithms
  inline void Mark() {mark = true; };

  //! A cost value, useful in some algorithms
  inline void Unmark() {mark = false; };

  //! A cost value, useful in some algorithms
  inline bool IsMarked() const {return mark; };

  MSLVertex();
  MSLVertex(const MSLVector &x);
  ~MSLVertex();

  //MSLVertex& operator=(const MSLVertex& n);
  friend istream& operator>> (istream& is, MSLVertex& n) { return is; };
  friend ostream& operator<< (ostream& os, const MSLVertex& n);
  //friend istream& operator>> (istream& is, list<MSLVertex*> & nl);
  //friend ostream& operator<< (ostream& os, const list<MSLVertex*> & nl);

  friend class MSLEdge;
  friend class MSLGraph;
};

//istream& operator>> (istream& is, MSLVertex& n) { return is; };
//ostream& operator<< (ostream& os, const MSLVertex& n);


class MSLEdge {
 private:
  MSLVector input;
  MSLVertex* source;
  MSLVertex* target;
  double time;
  double cost;
 public:
  MSLEdge();
  MSLEdge(MSLVertex* v1, MSLVertex* v2, 
	  const MSLVector &u, double t);
  MSLEdge(MSLVertex* v1, MSLVertex* v2, double t);
  MSLEdge(MSLVertex* v1, MSLVertex* v2);
  ~MSLEdge() {};

  //! The time required to reach this node from the parent
  inline double Time() const {return time; };

  //! A cost value, useful in some algorithms
  inline double Cost() const {return cost; };

  //! A cost value, useful in some algorithms
  inline void SetCost(const double &x) {cost = x; };

  //! An input vector that leads from the first state to the second
  inline MSLVector Input() const {return input; };

  //! The time required to reach this node from the parent
  inline MSLVertex* Source() const {return source; };

  //! The time required to reach this node from the parent
  inline MSLVertex* Target() const {return target; };

  //MSLEdge& operator=(const MSLEdge& n);
  friend istream& operator>> (istream& is, MSLEdge& e) { return is; };
  friend ostream& operator<< (ostream& os, const MSLEdge& e);
};

ostream& operator<< (ostream& os, const MSLEdge& e);


//! This is a comparison object to be used for STL-based sorting
class MSLVertexLess {
 public:
  bool operator() (MSLVertex* p, MSLVertex* q) const {
    return p->Cost() < q->Cost();
  }
};


//! This is a comparison object to be used for STL-based sorting
class MSLVertexGreater {
 public:
  bool operator() (MSLVertex* p, MSLVertex* q) const {
    return p->Cost() > q->Cost();
  }
};


class MSLGraph {
 private:
  list<MSLVertex*> vertices;
  list<MSLEdge*> edges;
  int numvertices;
  int numedges;
 public:

  MSLGraph();
  virtual ~MSLGraph();

  MSLVertex* AddVertex(const MSLVector &x);
  MSLEdge* AddEdge(MSLVertex* v1, MSLVertex* v2);
  MSLEdge* AddEdge(MSLVertex* v1, MSLVertex* v2, double time);
  MSLEdge* AddEdge(MSLVertex* v1, MSLVertex* v2, 
		   const MSLVector &u, double time);
  bool IsEdge(MSLVertex* v1, MSLVertex* v2);
  MSLVertex* FindVertex(int nid);
  inline list<MSLVertex*> Vertices() const { return vertices; };
  inline list<MSLEdge*> Edges() const { return edges; };
  inline int Size() {return numvertices+numedges;}
  inline int NumVertices() const {return numvertices;}
  inline int NumEdges() const {return numedges;}

  void Clear();

  //MSLGraph& operator=(const MSLGraph& n);
  friend istream& operator>> (istream& is, MSLGraph& n);
  friend ostream& operator<< (ostream& os, const MSLGraph& n);
};

istream& operator>> (istream& is, MSLGraph& n);
ostream& operator<< (ostream& os, const MSLGraph& n);

#endif
