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

#ifndef PI
#define PI 3.1415926535897932385
#endif

#include <math.h>
#include <ctype.h>
#include <cstdlib>

#include "msl/point.h"


MSLPoint::MSLPoint() {
  xrep = 0; yrep = 0;
}


MSLPoint::MSLPoint(double x, double y) {
  xrep = x; yrep = y;
}


double MSLPoint::angle(const MSLPoint& q, const MSLPoint& r) const
{
  double dx1 = q.xcoord() - xrep;
  double dy1 = q.ycoord() - yrep;
  double dx2 = r.xcoord() - xrep;
  double dy2 = r.ycoord() - yrep;

  if ((dx1 == 0 && dy1 == 0) || (dx2 == 0 && dy2 == 0)) {
    cerr << "MSLPoint::angle:  zero vector input.\n";
    exit(-1);
  }

  double norm  = (dx1*dx1+dy1*dy1)*(dx2*dx2+dy2*dy2);

  double cosfi = (dx1*dx2+dy1*dy2) / sqrt(norm);

  if (cosfi >=  1.0 ) return 0;
  if (cosfi <= -1.0 ) return PI;

  double fi = acos(cosfi);

  if (dx1*dy2 < dy1*dx2) fi = -fi;

  if (fi < 0) fi += 2*PI;

  return fi;
}



// Rotations

MSLPoint MSLPoint::rotate90(const MSLPoint& p) const
{ double px = p.xcoord();
  double py = p.ycoord();
  double dx = xrep - px;
  double dy = yrep - py;
  return MSLPoint(px-dy,py+dx);
}

MSLPoint MSLPoint::rotate90() const
{ return MSLPoint(-yrep,xrep); }


MSLPoint MSLPoint::rotate(const MSLPoint& origin, double fi) const
{ double cx = origin.xcoord();
  double cy = origin.ycoord();
  double sinfi = sin(fi);
  double cosfi = cos(fi);
  double dx = xrep - cx;
  double dy = yrep - cy;
  return MSLPoint(cx+dx*cosfi-dy*sinfi,cy+dx*sinfi+dy*cosfi);
}


MSLPoint MSLPoint::rotate(double fi) const
{ double sinfi = sin(fi);
  double cosfi = cos(fi);
  double x = xrep;
  double y = yrep;
  return MSLPoint(x*cosfi-y*sinfi,x*sinfi+y*cosfi);
}


MSLPoint MSLPoint::reflect(const MSLPoint& p, const MSLPoint& q) const
{ // reflect MSLPoint across line through p and q

  double px = p.xrep;
  double py = p.yrep;

  double x1 = xrep   - px;
  double y1 = yrep   - py;
  double x2 = q.xcoord() - px;
  double y2 = q.ycoord() - py;

  double L = (x1*x1 + y1*y1) * (x2*x2 + y2*y2);

  double cosfi = (x1*x2 + y1*y2);
  double sinfi = (x1*y2 - x2*y1);
  double cos2 = (cosfi*cosfi - sinfi*sinfi)/L;
  double sin2 = 2*cosfi*sinfi/L;

  return MSLPoint(px + x1*cos2-y1*sin2, py + x1*sin2+y1*cos2);
}


MSLPoint MSLPoint::reflect(const MSLPoint& q) const
{ // reflect MSLPoint across MSLPoint q
  return MSLPoint(2*q.xcoord()-xrep, 2*q.ycoord()-yrep);
 }



// Translations

MSLPoint MSLPoint::translate(double dx, double dy) const
{ return MSLPoint(xrep+dx,yrep+dy); }


MSLPoint MSLPoint::translate_by_angle(double phi, double d) const
{ double dx = cos(phi) * d;
  double dy = sin(phi) * d;
  if (fabs(dx) < 1e-12) dx = 0;
  if (fabs(dy) < 1e-12) dy = 0;
  return MSLPoint(xrep+dx,yrep+dy);
 }


// Distances

double MSLPoint::sqr_dist(const MSLPoint& p)  const
{ double dx = p.xcoord() - xrep;
  double dy = p.ycoord() - yrep;
  return dx*dx + dy*dy;
 }



double MSLPoint::xdist(const MSLPoint& q) const
{ return fabs(xrep - q.xcoord()); }

double MSLPoint::ydist(const MSLPoint& q) const
{ return fabs(yrep - q.ycoord()); }

double  MSLPoint::distance(const MSLPoint& q) const
{ return sqrt(sqr_dist(q)); }


bool MSLPoint::operator==(const MSLPoint& p) const
{ return (xrep == p.xcoord()) && (yrep == p.ycoord()); }



int side_of_circle(const MSLPoint& a, const MSLPoint& b, const MSLPoint& c,
                                                   const MSLPoint& d)
{  // comments indicate bit lengths of values if coordinates have
   // at most L bits.
   double ax = a.xcoord();    // L bits
   double ay = a.ycoord();

   double bx = b.xcoord() - ax;  // L + 1 bits
   double by = b.ycoord() - ay;
   double bw = bx*bx + by*by;    // 2L + 3 bits

   double cx = c.xcoord() - ax;  // L + 1 bits
   double cy = c.ycoord() - ay;
   double cw = cx*cx + cy*cy;    // 2L + 3 bits

   double D1 = cy*bw - by*cw;  // 2L + 3 + L + 1 + 1 = 3L + 5 bits
   double D2 = bx*cw - cx*bw;  // 3L + 5 bits
   double D3 = by*cx - bx*cy;  // 2L + 3

   double dx = d.xcoord() - ax;  // L + 1 bits
   double dy = d.ycoord() - ay;

   double D  = D1*dx  + D2*dy + D3*(dx*dx + dy*dy);
                               // 3L + 5 + L + 1 + 2 = 4L + 8 bits

   if (D != 0)
      return (D > 0) ? 1 : -1;
   else
      return 0;
}

/*
ostream& operator<<(ostream& out, const list<list<MSLPoint> >& L)
{
  list<list<MSLPoint> >::iterator x;
  list<list<MSLPoint> > vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++)
    out << *x << "\n";
  return out;
}


istream& operator>>(istream& in, list<list<MSLPoint> >& L)
{
  L.clear();
  list<MSLPoint> x;
  for(;;)
    {
      char c;
      while (in.get(c) && isspace(c));
      if (!in) break;
      in.putback(c);
      x = MSLPolygon(); in >> x; L.push_back(x);
    }
  return in;
}


ostream& operator<<(ostream& os, const list<MSLPoint>& vl)
{
  list<MSLPoint>::iterator x;
  list<MSLPoint> L;
  L = vl;
  for (x = L.begin(); x != L.end(); x++)
	os << " " << *x;
  return os;
}


istream& operator>>(istream& in, list<MSLPoint>& L)
{
  L.clear();
  MSLPoint x;
  char c;
  for(;;)
    {
      while (in.get(c) && c==' ');
      if ((!in) || isspace(c)) break;
      in.putback(c);
      x = MSLPoint(); in >> x; L.push_back(x);
    }
  return in;
}
*/

ostream& operator<<(ostream& out, const MSLPoint& p)
{ out << "(" << p.xcoord() << "," << p.ycoord() << ")";
  return out;
}

istream& operator>>(istream& in, MSLPoint& p)
{ // syntax: {(} x {,} y {)}

  double x,y;
  char c;

  do in.get(c); while (in && isspace(c));

  if (!in) return in;

  if (c != '(') in.putback(c);

  in >> x;

  do in.get(c); while (isspace(c));
  if (c != ',') in.putback(c);

  in >> y;

  do in.get(c); while (c == ' ');
  if (c != ')') in.putback(c);

  p = MSLPoint(x,y);
  return in;

}
