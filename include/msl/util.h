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

#ifndef MSL_UTIL_H
#define MSL_UTIL_H

#include <list>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>
#include <vector>
using namespace std;

float used_time();

float used_time(float&);

//ostream& operator<<(ostream& out, const list<string>& L);

//istream& operator>>(istream& in, list<string>& L);

//ostream& operator<<(ostream& out, const list<int>& L);

//istream& operator>>(istream& in, list<int>& L);

bool is_file(string fname);

bool is_directory(string fname);

#endif
