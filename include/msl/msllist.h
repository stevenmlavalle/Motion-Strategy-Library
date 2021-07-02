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

#ifndef MSL_LIST_H
#define MSL_LIST_H

#ifdef WIN32
	#include <fstream>
	#include <iostream>
	#include <list>
	using namespace std;
#else
	#include <stream.h>
	#include <list.h>
#endif

template<class T, class A = allocator<T> >
class MSLList: public list<T, A>{

  friend ostream& operator<<(ostream& out, const MSLList<T>& L);
  friend istream& operator>>(istream& in, MSLList<T>& L);
};

#endif