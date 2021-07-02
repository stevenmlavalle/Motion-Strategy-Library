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

#ifndef MSL_POINT3D_H
#define MSL_POINT3D_H

#include <iostream>
#include <list>
using namespace std;

#include "mslio.h"

class MSLPoint3d
{
  double xrep;
  double yrep;
  double zrep;

public:
  
  MSLPoint3d();
  MSLPoint3d(double x, double y, double z);
  ~MSLPoint3d() {}
  double  xcoord()  const   { return xrep; }
  double  ycoord()  const   { return yrep; }
  double  zcoord()  const   { return zrep; }
  double  W()   const   { return 1; }
  double  WD()  const   { return 1; }
  int     dim() const { return 3; }
  double  sqr_dist(const MSLPoint3d& q) const;
  double xdist(const MSLPoint3d& q) const;
  double ydist(const MSLPoint3d& q) const;
  double zdist(const MSLPoint3d& q) const;
  double  distance(const MSLPoint3d& q) const;
  MSLPoint3d translate(double dx, double dy, double dz) const;
  MSLPoint3d reflect(const MSLPoint3d& q, const MSLPoint3d& r, 
		     const MSLPoint3d& s) const;
  MSLPoint3d reflect(const MSLPoint3d& q) const;
  bool operator==(const MSLPoint3d& q) const;
  bool operator!=(const MSLPoint3d& q)  const { return !operator==(q);}
  friend ostream& operator<<(ostream& O, const MSLPoint3d& p) ;
  friend istream& operator>>(istream& I, MSLPoint3d& p) ;
  //friend istream& operator>>(istream& is, list<MSLPoint3d> & vl);
  //friend ostream& operator<<(ostream& os, const list<MSLPoint3d> & vl);
};

ostream& operator<<(ostream& O, const MSLPoint3d& p) ;
istream& operator>>(istream& I, MSLPoint3d& p) ;

#endif





