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

#ifndef MSL_MATRIX_H
#define MSL_MATRIX_H

#include "vector.h"
#include "mslio.h"

class MSLMatrix {

  MSLVector** v;
  int  d1;
  int  d2;

  void     flip_rows(int,int);
  void     check_dimensions(const MSLMatrix&) const; 
  double&  elem(int i, int j) const { return v[i]->v[j]; }
  double** triang(const MSLMatrix&, int&) const;
  
public:

  MSLMatrix(int n=0, int m=0);
  MSLMatrix(int n, int m, double* D);
  MSLMatrix(const MSLMatrix&);
  MSLMatrix(const MSLVector&);
  MSLMatrix& operator=(const MSLMatrix&);
  ~MSLMatrix();

  int     dim1()  const  {  return d1; }
  int     dim2()  const  {  return d2; }
  MSLVector& row(int i) const;
  MSLVector  col(int i) const;
  MSLMatrix  trans() const;
  MSLMatrix  inv()   const;
  double  det()   const;
  MSLMatrix solve(const MSLMatrix&) const;
  MSLVector solve(const MSLVector& b) const 
    { return MSLVector(solve(MSLMatrix(b))); }
  operator MSLVector() const;
  MSLVector& operator[](int i)    const { return row(i); }
  double& operator()(int i, int j);
  double  operator()(int,int) const;
  int     operator==(const MSLMatrix&)    const;
  int     operator!=(const MSLMatrix& x)  const { return !(*this == x); }
  MSLMatrix operator+(const MSLMatrix& M1) const;
  MSLMatrix operator-(const MSLMatrix& M1) const;
  MSLMatrix operator-() const;
  MSLMatrix& operator-=(const MSLMatrix&);
  MSLMatrix& operator+=(const MSLMatrix&);
  MSLMatrix operator*(const MSLMatrix& M1) const;
  MSLVector operator*(const MSLVector& vec) const 
    { return MSLVector(*this * MSLMatrix(vec)); }
  MSLMatrix operator*(double x) const;
  void read(istream& I);
  void read() { read(cin); }

  friend ostream& operator<<(ostream& O, const MSLMatrix& M);
  friend istream& operator>>(istream& I, MSLMatrix& M);
};

ostream& operator<<(ostream& O, const MSLMatrix& M);
istream& operator>>(istream& I, MSLMatrix& M);


#endif
