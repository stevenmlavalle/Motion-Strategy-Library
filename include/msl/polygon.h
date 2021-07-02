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

#ifndef MSL_POLYGON_H
#define MSL_POLYGON_H

#include <iostream>
#include <list>
using namespace std;

#include "mslio.h"
#include "point.h"

class MSLPolygon
{
public:
  list<MSLPoint> LPoints;

  MSLPolygon() {}
  ~MSLPolygon() {LPoints.clear();}

  friend istream& operator>>(istream& in, MSLPolygon& P);
  friend ostream& operator<<(ostream& out, const MSLPolygon& P);
};

istream& operator>>(istream& in, MSLPolygon& P);
ostream& operator<<(ostream& out, const MSLPolygon& P);

#endif





