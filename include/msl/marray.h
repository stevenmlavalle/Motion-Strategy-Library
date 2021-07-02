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

#ifndef MSL_MARRAY_H
#define MSL_MARRAY_H

#include "vector.h"
#include "mslio.h"

//! A multidimensional array made from a 1D vector

template<class E> class MultiArray {
  //! The one-dimensional STL vector which actually holds the data
  vector<E> A;

  //! Used for computing fast offsets
  vector<int> Offsets;

  //! Gives the limit for each index: 0..Dimensions[i]-1
  vector<int> Dimensions;

  //! The dimension of the multi-dimensional array
  int Dimension;

  //! Number of elements in the array
  int Size;

 public:
  //! Maximum allowable array size (default = 10 million)
  int MaxSize;

  //! Constructor with default assignment of x to each element
  MultiArray(const vector<int> &dims, const E &x);

  //! Constructor with no default assignment
  MultiArray(const vector<int> &dims);

  //! Constructor with no initialization
  MultiArray() {};
  ~MultiArray() {};

  //! This can be used for access or assignment (e.g., ma[indices] = 1).
  inline E& operator[](const vector<int> &indices);

  //! Get the next element (used as an iterator).  Return true if at end.
  inline bool Increment(vector<int> &indices);

  //! This will not work correctly unless dimensions are preset correctly
  friend istream& operator>> (istream &is, MultiArray &ma)
    { is >> ma.A; return is; }

  //! Just print out the vector
  friend ostream& operator<< (ostream &os, const MultiArray &ma)
    { os << ma.A; return os; }
};

template<class E> MultiArray<E>::MultiArray(const vector<int> &dims,
					    const E &x) {
  int i,offset;

  MaxSize = 10000000; // Maximum allowable size
  Dimensions = dims;
  Dimension = dims.size();

  Offsets = vector<int>(Dimension);

  offset = 1;
  for (i = 0; i < Dimension; i++) {
    Offsets[i] = offset;
    offset *= Dimensions[i];
  }

  Size = offset;

  if (Size <= MaxSize) {
    // Make the array to hold all of the data
    A = vector<E>(Size);
    for (i = 0; i < Size; i++)
      A[i] = x; // Write the value x to all elements
  }
  else {
    cout << "Size " << Size << " exceeds MaxSize limit " << MaxSize << "\n";
    exit(-1);
  }

}


template<class E> MultiArray<E>::MultiArray(const vector<int> &dims) {
  E x;

  MultiArray(dims,x);
}


template<class E> inline E& MultiArray<E>::operator[](const vector<int>
						      &indices) {
  int i,index;

  index = indices[0];

  for (i = 1; i < Dimension; i++) {
    index += indices[i]*Offsets[i];
  }

  return A[index];
}


template<class E> inline bool MultiArray<E>::Increment(vector<int> &indices) {
  int i;
  bool carry,done;

  carry = false;
  done = false;

  if (indices[0] < Dimensions[0] - 1)
    indices[0]++;
  else { // Carry
    indices[0] = 0;
    carry = true;
    i = 1;
    while ((carry)&&(i < Dimension)) {
      if (indices[i] < Dimensions[i] - 1) {
	indices[i]++;
	carry = false;
      }
      else {
	indices[i] = 0;
	if (i == Dimension - 1)
	  done = true;
      }
      i++;
    }
  }

  // This will report true if the end of array was reached
  return done;
}

#endif
