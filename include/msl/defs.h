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
#ifndef MSL_DEFS_H
#define MSL_DEFS_H

#ifndef PI
#define PI 3.1415926535897932385
#endif 
#ifndef INFINITY
#define INFINITY 1.0e40
#endif
#ifndef sqr
#define sqr(x) ((x)*(x))
#endif
//#ifndef min
//#define min(x,y) ((x<y) ? x : y)
//#endif
//#ifndef max
//#define max(x,y) ((x>y) ? x : y)
//#endif

#ifndef MSL_GLOBAL_VARS
#define MSL_GLOBAL_VARS
	#include <fstream>
	#include <iostream>
        #include <cstdlib>
	using namespace std;
	static std::ifstream _msl_fin;
#endif

#define READ_OPTIONAL_PARAMETER(F)\
_msl_fin.clear();\
_msl_fin.open((FilePath+""#F"").c_str());\
if (_msl_fin) {_msl_fin >> F;}\
_msl_fin.close();\

#define READ_PARAMETER_OR_DEFAULT(F,D)\
_msl_fin.clear();\
_msl_fin.open((FilePath+""#F"").c_str());\
if (_msl_fin) {_msl_fin >> F;}\
else F = D;\
_msl_fin.close();\

#define READ_PARAMETER_OR_ERROR(F)\
_msl_fin.clear();\
_msl_fin.open((FilePath+""#F"").c_str());\
if (_msl_fin) {_msl_fin >> F; _msl_fin.close();}\
else {cerr << "Error reading "#F"\n"; exit(-1);}\
_msl_fin.close();\
  
// Convenient list iterator
#define forall(x,S)\
for (x = S.begin(); x != S.end(); x++)\

#endif



