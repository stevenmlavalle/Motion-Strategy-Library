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
#include <ctype.h>

#include "msl/polygon.h"

istream& operator>>(istream& in, MSLPolygon& P)
{
  P.LPoints.clear();
  MSLPoint x;
  char c;
  for(;;)
  {
	while (in.get(c) && c==' ');
	if ((!in) || isspace(c)) break;
	in.putback(c);
	x = MSLPoint(); in >> x; P.LPoints.push_back(x);
  }
  return in;
}

ostream& operator<<(ostream& out, const MSLPolygon& P)
{
  out << P.LPoints << endl;
  return out;
}
