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

#include <math.h>
#include <ctype.h>
#include <cstdlib>
#include "msl/point3d.h"

/*ostream& operator<<(ostream& out, const list<MSLPoint3d>& L)
{
  list<MSLPoint3d>::iterator x;
  list<MSLPoint3d> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++)
    out << " " << *x;
  return out;
}


istream& operator>>(istream& in, list<MSLPoint3d>& L)
{
  L.clear();
  MSLPoint3d x;
  for(;;)
    {
      char c;
      while (in.get(c) && isspace(c));
      if (!in) break;
      in.putback(c);
      x = MSLPoint3d(); in >> x; L.push_back(x);
    }
  return in;
}
*/

static void error_handler(int i, const char* s) {
  cerr << s << "\n";
  exit(i);
}


MSLPoint3d::MSLPoint3d() { xrep = 0; yrep = 0; zrep=0; }

MSLPoint3d::MSLPoint3d(double x, double y, double z)
{ xrep = x; yrep = y; zrep=z; }


// Translations

MSLPoint3d MSLPoint3d::translate(double dx, double dy, double dz) const
{ return MSLPoint3d(xcoord()+dx,ycoord()+dy,zcoord()+dz); }



// Distances

double MSLPoint3d::sqr_dist(const MSLPoint3d& p)  const
{ double dx = p.xcoord() - xrep;
  double dy = p.ycoord() - yrep;
  double dz = p.zcoord() - zrep;
  return dx*dx + dy*dy + dz*dz;
 }

double MSLPoint3d::xdist(const MSLPoint3d& q) const
{ return fabs(xrep - q.xcoord()); }

double MSLPoint3d::ydist(const MSLPoint3d& q) const
{ return fabs(yrep - q.ycoord()); }

double MSLPoint3d::zdist(const MSLPoint3d& q) const
{ return fabs(zrep - q.zcoord()); }


MSLPoint3d MSLPoint3d::reflect(const MSLPoint3d& q) const
{ // reflect point across point q
  return MSLPoint3d(2*q.xcoord()-xcoord(), 2*q.ycoord()-ycoord(),
                                         2*q.zcoord()-zcoord());
 }


MSLPoint3d  MSLPoint3d::reflect(const MSLPoint3d& a, const MSLPoint3d& b,
                                               const MSLPoint3d& c) const
{
  // reflect point across plane through a, b, and c

  double x1 = b.xcoord() - a.xcoord();
  double y1 = b.ycoord() - a.ycoord();
  double z1 = b.zcoord() - a.zcoord();

  double x2 = c.xcoord() - a.xcoord();
  double y2 = c.ycoord() - a.ycoord();
  double z2 = c.zcoord() - a.zcoord();

  double x3 = xcoord() - a.xcoord();
  double y3 = ycoord() - a.ycoord();
  double z3 = zcoord() - a.zcoord();

  double x = (z1*y2-y1*z2);
  double y = (x1*z2-z1*x2);
  double z = (y1*x2-x1*y2);

  if (x == 0 && y == 0 && z == 0)
      error_handler(1,"MSLPoint3d::reflect(a,b,c): a,b,c are coplanar");


  double f = -2*(x*x3+y*y3+z*z3)/(x*x+y*y+z*z);

  return translate(f*x,f*y,f*z);
}




double  MSLPoint3d::distance(const MSLPoint3d& q) const
{ return sqrt(sqr_dist(q)); }


bool MSLPoint3d::operator==(const MSLPoint3d& p) const
{ return xrep == p.xrep &&
         yrep == p.yrep &&
         zrep == p.zrep;
}



ostream& operator<<(ostream& out, const MSLPoint3d& p)
{ out << "(" << p.xcoord() << "," << p.ycoord() << "," << p.zcoord() << ")";
  return out;
 }

istream& operator>>(istream& in, MSLPoint3d& p)
{ // syntax: {(} x {,} y {,} z{)}

  double x,y,z;
  char c;

  do in.get(c); while (in && isspace(c));

  if (!in) return in;

  if (c != '(') in.putback(c);

  in >> x;

  do in.get(c); while (isspace(c));
  if (c != ',') in.putback(c);

  in >> y;

  do in.get(c); while (isspace(c));
  if (c != ',') in.putback(c);

  in >> z;

  do in.get(c); while (c == ' ');
  if (c != ')') in.putback(c);

  p = MSLPoint3d(x,y,z);
  return in;

 }
