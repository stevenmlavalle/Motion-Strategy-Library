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

#ifndef MSL_IO_H
#define MSL_IO_H

#include <list>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

template<class T> ostream& operator<<(ostream& out, const list<T>& L);
template<class T> istream& operator>>(istream& in, list<T>& L);
template<class T> ostream& operator<<(ostream& out, const vector<T>& L);
template<class T> istream& operator>>(istream& in, vector<T>& L);

template<class T> ostream& operator<<(ostream& out, const list<T>& L)
{
  typename list<T>::iterator x; 
  list<T> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++) 
    out << " " << *x;
  return out;
}

template<class T> istream& operator>>(istream& in, list<T>& L)
{ 
  L.clear();
  T x;
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


template<class T> ofstream& operator<<(ofstream& out, const vector<T>& L)
{
  typename vector<T>::iterator x; 
  vector<T> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++) 
    out << " " << *x;
  return out;
}


template<class T> ifstream& operator>>(ifstream& in, vector<T>& L)
{ 
  L.clear();
  T x;
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


#endif
