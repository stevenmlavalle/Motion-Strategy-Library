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

//#include "msllist.h"

#ifdef WIN32
	#include <fstream>
	#include <iostream>
	#include <list>
	#include <vector>
	using namespace std;
#else
	#include <stream>
	#include <list>
	#include <vector>
#endif


template<class T, class A = allocator<T> >
ostream& operator<<(ostream& out, const list<T>& L)
{
  list<MSLVector>::iterator x;
  list<MSLVector> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++)
    out << " " << *x;
  return out;
}

template<class T, class A = allocator<T> >
istream& operator>>(istream& in, list<T>& L)
{
  L.clear();
  MSLVector x;
  for(;;)
    {
      char c;
      while (in.get(c) && isspace(c));
      if (!in) break;
      in.putback(c);
      x = T(); in >> x; L.push_back(x);
   }
  return in;
}
