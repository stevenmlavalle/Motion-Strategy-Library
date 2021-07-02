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

#ifndef MSL_TRIANGLE_H
#define MSL_TRIANGLE_H

#include <ctype.h>

#include <list>
#include <iostream>
#include <fstream>
using namespace std;

#include "point3d.h" 
#include "point.h"
#include "mslio.h"
#include "polygon.h"

//! A 3D triangle, made of 3 3D points

class MSLTriangle {
 public:
  MSLPoint3d p1,p2,p3;

  MSLTriangle(MSLPoint3d pt1, MSLPoint3d pt2, MSLPoint3d pt3);
  MSLTriangle();
  ~MSLTriangle(){}
  MSLTriangle(const MSLTriangle& tr);
  friend istream& operator>> (istream& is, MSLTriangle& tr);
  friend ostream& operator<< (ostream& os, const MSLTriangle& tr);
  MSLTriangle& operator= (const MSLTriangle& tr);

  //friend istream& operator>> (istream& is, list<MSLTriangle> & tl);
  //friend ostream& operator<< (ostream& os, const list<MSLTriangle> & tl);

};

istream& operator>> (istream& is, MSLTriangle& tr);
ostream& operator<< (ostream& os, const MSLTriangle& tr);


//! A handy utility for converting 2D geometries into 3D
//list<MSLTriangle> PolygonsToTriangles(const list<list<MSLPoint> > &pl,
//				      double thickness);
list<MSLTriangle> PolygonsToTriangles(const list<MSLPolygon> &pl,
				      double thickness);


#endif

