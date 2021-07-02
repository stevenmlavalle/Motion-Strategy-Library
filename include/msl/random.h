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

#ifndef MSL_RANDOM_H
#define MSL_RANDOM_H


class MSLRandomSource {

  long  table[32];
  long* ptr0;
  long* ptr1;
  long* ptr2;
  long* ptr_end;
  unsigned long pat;
  int  prec;
  int  low;
  int  diff;
  bool bit_mode;
  unsigned long buffer;
  static int count;
  void          init_table();

public:

  unsigned long get_rand31();
  unsigned long get_rand32();
  MSLRandomSource();
  MSLRandomSource(int p);
  MSLRandomSource(int low, int high);
  unsigned long get() { unsigned long r = get_rand32();
#if defined(WORD_LENGTH_64)
 r = (r << 32) | get_rand32();
#endif
 return r; }
  void set_seed(int s);
  void set_range(int low, int high);
  int set_precision(int p);
  int get_precision() { return prec; }
#if defined(__HAS_BUILTIN_BOOL__)
  MSLRandomSource& operator>>(char& x);
#endif
  MSLRandomSource& operator>>(unsigned char& x);
  MSLRandomSource& operator>>(int& x);
  MSLRandomSource& operator>>(long& x);
  MSLRandomSource& operator>>(unsigned int& x);
  MSLRandomSource& operator>>(unsigned long& x);
  MSLRandomSource& operator>>(double& x);
  MSLRandomSource& operator>>(float& x);
  MSLRandomSource& operator>>(bool& b);
  int operator()();    
  int operator()(int prec);
  int operator()(int low, int high);
};


#endif
