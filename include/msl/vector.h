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

#ifndef MSL_VECTOR_H
#define MSL_VECTOR_H

#include <iostream>

#include "msl/mslio.h"

void error_handler(int i, const char* s);

class MSLVector
{
public:
    MSLVector();

    MSLVector(int d);

    MSLVector(double a, double b);

    MSLVector(double a, double b, double c);

    MSLVector(const MSLVector& w, int prec);

    MSLVector(const MSLVector&);

    ~MSLVector();

    MSLVector& operator=(const MSLVector&);

    int    dim()    const { return d; }

    double& operator[](int i);

    double  operator[](int) const;

    double  hcoord(int i) const { return (i<d) ? (*this)[i] : 1; }

    double  coord(int i)  const { return (*this)[i]; }

    double sqr_length() const;

    double length() const;

    MSLVector norm() const { return *this/length(); }

    double angle(const MSLVector& w) const;

    MSLVector rotate90() const;

    MSLVector rotate(double a) const;

    MSLVector& operator+=(const MSLVector&);

    MSLVector& operator-=(const MSLVector&);

    MSLVector  operator+(const MSLVector& v1) const;

    MSLVector  operator-(const MSLVector& v1) const;

    double  operator*(const MSLVector& v1) const;

    MSLVector  operator*(double r)        const;

    MSLVector  operator-() const;

    MSLVector  operator/(double)        const;

    bool     operator==(const MSLVector& w) const;

    bool     operator!=(const MSLVector& w)  const { return !(*this == w); }

    friend MSLVector operator*(double f, const MSLVector& v) { return v *f; }

    void print(std::ostream& O);

    void print() { print(cout); }

    void read(std::istream& I);

    void read() { read(cin); }

    friend std::ostream& operator<<(std::ostream& O, const MSLVector& v);

    friend std::istream& operator>>(std::istream& I, MSLVector& v);

private:
    friend class MSLMatrix;

    double* v;
    int d;

    void check_dimensions(const MSLVector&) const;
};

std::ostream& operator<<(std::ostream& O, const MSLVector& v);
std::istream& operator>>(std::istream& I, MSLVector& v);

#endif
