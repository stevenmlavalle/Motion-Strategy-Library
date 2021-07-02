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

#include "msl/matrix.h"

#define EPSILON 1e-12


void MSLMatrix::flip_rows(int i,int j)
{ MSLVector* p = v[i];
  v[i] = v[j];
  v[j] = p;
 }

MSLMatrix::MSLMatrix(int dim1, int dim2)
{
  if (dim1<0 || dim2<0)
  error_handler(1,"MSLMatrix: negative dimension.");

  d1=dim1;
  d2=dim2;

  if (d1 > 0)
  { v = new MSLVector*[d1];
    for (int i=0;i<d1;i++) v[i] = new MSLVector(d2);
   }
  else v = NULL;
}


MSLMatrix::MSLMatrix(const MSLMatrix& p)
{
  d1 = p.d1;
  d2 = p.d2;

  if (d1 > 0)
  { v = new MSLVector*[d1];
    for (int i=0;i<d1;i++) v[i] = new MSLVector(*p.v[i]);
   }
  else v = NULL;
}

MSLMatrix::MSLMatrix(int dim1, int dim2, double* D)
{ d1=dim1; d2=dim2;
  v = new MSLVector*[dim1];
  double* p = D;
  for(int i=0;i<dim1;i++)
  { v[i] = new MSLVector(dim2);
    for(int j=0;j<dim2;j++) elem(i,j) = *p++;
   }

 }

MSLMatrix::~MSLMatrix()
{ if (v)
  { while(d1--) delete v[d1];
    delete[] v;
   }
}


void MSLMatrix::check_dimensions(const MSLMatrix& mat) const
{ if (d1 != mat.d1 || d2 != mat.d2)
   error_handler(1,"incompatible MSLMatrix types.");
 }

MSLMatrix::MSLMatrix(const MSLVector& vec)
{ d1 = vec.d;
  d2 = 1;
  v = new MSLVector*[d1];
  for(int i=0; i<d1; i++)
  { v[i] = new MSLVector(1);
    elem(i,0) = vec[i];
   }

}

MSLMatrix& MSLMatrix::operator=(const MSLMatrix& mat)
{ register int i,j;

  if (d1 != mat.d1 || d2 != mat.d2)
  { for(i=0;i<d1;i++) delete v[i];
    delete[] v;
    d1 = mat.d1;
    d2 = mat.d2;
    v = new MSLVector*[d1];
    for(i=0;i<d1;i++) v[i] = new MSLVector(d2);
   }

  for(i=0;i<d1;i++)
    for(j=0;j<d2;j++) elem(i,j) = mat.elem(i,j);

  return *this;
}

int MSLMatrix::operator==(const MSLMatrix& x) const
{ register int i,j;
  if (d1 != x.d1 || d2 != x.d2) return false;

  for(i=0;i<d1;i++)
    for(j=0;j<d2;j++)
      if (elem(i,j) != x.elem(i,j)) return false;

  return true;
 }


MSLVector& MSLMatrix::row(int i) const
{ if ( i<0 || i>=d1 )  error_handler(1,"MSLMatrix: row index out of range");
   return *v[i];
}


double& MSLMatrix::operator()(int i, int j)
{ if ( i<0 || i>=d1 )  error_handler(1,"MSLMatrix: row index out of range");
  if ( j<0 || j>=d2 )  error_handler(1,"MSLMatrix: col index out of range");
  return elem(i,j);
}

double MSLMatrix::operator()(int i, int j) const
{ if ( i<0 || i>=d1 )  error_handler(1,"MSLMatrix: row index out of range");
  if ( j<0 || j>=d2 )  error_handler(1,"MSLMatrix: col index out of range");
  return elem(i,j);
}

MSLVector MSLMatrix::col(int i)  const
{ if ( i<0 || i>=d2 )  error_handler(1,"MSLMatrix: col index out of range");
  MSLVector result(d1);
  int j = d1;
  while (j--) result.v[j] = elem(j,i);
  return result;
}

MSLMatrix::operator MSLVector() const
{ if (d2!=1)
   error_handler(1,"error: cannot make MSLVector from MSLMatrix\n");
  return col(0);
}

MSLMatrix MSLMatrix::operator+(const MSLMatrix& mat) const
{ register int i,j;
  check_dimensions(mat);
  MSLMatrix result(d1,d2);
  for(i=0;i<d1;i++)
   for(j=0;j<d2;j++)
      result.elem(i,j) = elem(i,j) + mat.elem(i,j);
  return result;
}

MSLMatrix& MSLMatrix::operator+=(const MSLMatrix& mat)
{ register int i,j;
  check_dimensions(mat);
  for(i=0;i<d1;i++)
   for(j=0;j<d2;j++)
      elem(i,j) += mat.elem(i,j);
  return *this;
}

MSLMatrix& MSLMatrix::operator-=(const MSLMatrix& mat)
{ register int i,j;
  check_dimensions(mat);
  for(i=0;i<d1;i++)
   for(j=0;j<d2;j++)
      elem(i,j) -= mat.elem(i,j);
  return *this;
}


MSLMatrix MSLMatrix::operator-(const MSLMatrix& mat) const
{ register int i,j;
  check_dimensions(mat);
  MSLMatrix result(d1,d2);
  for(i=0;i<d1;i++)
   for(j=0;j<d2;j++)
      result.elem(i,j) = elem(i,j) - mat.elem(i,j);
  return result;
}


MSLMatrix MSLMatrix::operator-()  const
{ register int i,j;
  MSLMatrix result(d1,d2);
  for(i=0;i<d1;i++)
   for(j=0;j<d2;j++)
      result.elem(i,j) = -elem(i,j);
  return result;
}


MSLMatrix MSLMatrix::operator*(double f) const
{ register int i,j;
  MSLMatrix result(d1,d2);
  for(i=0;i<d1;i++)
   for(j=0;j<d2;j++)
      result.elem(i,j) = elem(i,j) *f;
  return result;
}

MSLMatrix MSLMatrix::operator*(const MSLMatrix& mat) const
{ if (d2!=mat.d1)
     error_handler(1,"MSLMatrix multiplication: incompatible MSLMatrix types\n");

  MSLMatrix result(d1, mat.d2);
  register int i,j;

  for (i=0;i<mat.d2;i++)
  for (j=0;j<d1;j++) result.elem(j,i) = *v[j] * mat.col(i);

 return result;

}

double MSLMatrix::det() const
{
 if (d1!=d2)
   error_handler(1,"MSLMatrix::det: MSLMatrix not quadratic.\n");

 int n = d1;

 MSLMatrix M(n,1);

 int flips;

 double** A = triang(M,flips);

 if (A == NULL)  return 0;

 double Det = 1;

 int i;
 for(i=0;i<n;i++) Det *= A[i][i];
 for(i=0;i<n;i++) delete[] A[i];
 delete[] A;

 return (flips % 2) ? -Det : Det;

}


double** MSLMatrix::triang(const MSLMatrix& M, int& flips)  const
{
 register double **p, **q;
 register double *l, *r, *s;

 register double pivot_el,tmp;

 register int i,j, col, row;

 int n = d1;
 int d = M.d2;
 int m = n+d;

 double** A = new double*[n];

 p = A;

 for(i=0;i<n;i++)
 { *p = new double[m];
   l = *p++;
   for(j=0;j<n;j++) *l++ = elem(i,j);
   for(j=0;j<d;j++) *l++ = M.elem(i,j);
  }

 flips = 0;

 for (col=0, row=0; row<n; row++, col++)
 {
   // search for row j with maximal absolute entry in current col
   j = row;
   for (i=row+1; i<n; i++)
     if (fabs(A[j][col]) < fabs(A[i][col])) j = i;

   if (row < j)
   { double* p = A[j];
     A[j] = A[row];
     A[row] = p;
     flips++;
    }

   tmp = A[row][col];
   q  = &A[row];

   if (fabs(tmp) < EPSILON) // MSLMatrix has not full rank
   { p = A;
     for(i=0;i<n;i++) delete A[i];
     delete[] A;
     return NULL;
    }

   for (p = &A[n-1]; p != q; p--)
   {
     l = *p+col;
     s = *p+m;
     r = *q+col;

     if (*l != 0.0)
     { pivot_el = *l/tmp;
        while(l < s) *l++ -= *r++ * pivot_el;
      }

    }

  }

 return A;
}

MSLMatrix MSLMatrix::inv() const
{
 if (d1!=d2)
     error_handler(1,"MSLMatrix::inv: MSLMatrix not quadratic.\n");
 int n = d1;
 MSLMatrix I(n,n);
 for(int i=0; i<n; i++) I(i,i) = 1;
 return solve(I);
}



MSLMatrix MSLMatrix::solve(const MSLMatrix& M) const
{

if (d1 != d2 || d1 != M.d1)
     error_handler(1, "Solve: wrong dimensions\n");

 register double **p, ** q;
 register double *l, *r, *s;

 int      n = d1;
 int      d = M.d2;
 int      m = n+d;
 int      row, col,i;


 double** A = triang(M,i);

 if (A == NULL)
   error_handler(1,"MSLMatrix::solve: MSLMatrix has not full rank.");

 for (col = n-1, p = &A[n-1]; col>=0; p--, col--)
 {
   s = *p+m;

   double tmp = (*p)[col];

   for(l=*p+n; l < s; l++) *l /=tmp;

   for(q = A; q != p; q++ )
   { tmp = (*q)[col];
     l = *q+n;
     r = *p+n;
     while(r < s)  *l++ -= *r++ * tmp;
    }

  }

  MSLMatrix result(n,d);

  for(row=0; row<n; row++)
  { l = A[row]+n;
    for(col=0; col<d; col++) result.elem(row,col) = *l++;
    delete[] A[row];
   }

  delete[] A;

  return result;
}




MSLMatrix MSLMatrix::trans() const
{ MSLMatrix result(d2,d1);
  for(int i = 0; i < d2; i++)
    for(int j = 0; j < d1; j++)
      result.elem(i,j) = elem(j,i);
  return result;
}


void MSLMatrix::read(istream& is)
{ for(int i = 0; i < d1; i++)
    for(int j = 0; j < d2; j++)
        is >> elem(i,j);
 }


ostream& operator<<(ostream& os, const MSLMatrix& M)
{ os << M.dim1() << " " << M.dim2() << endl;
  for(int i = 0; i < M.dim1(); i++)
  { for(int j = 0; j < M.dim2(); j++) os << " " << M(i,j);
    os << endl;
   }
  return os;
}

istream& operator>>(istream& is, MSLMatrix& M)
{ int d1,d2;
  is >> d1 >> d2;
  MSLMatrix MM(d1,d2);
  for(int i = 0; i < d1; i++)
    for(int j = 0; j < d2; j++)
       is >> MM(i,j);
  M = MM;
  return is;
}
