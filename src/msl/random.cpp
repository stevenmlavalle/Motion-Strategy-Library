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

#include <time.h>

#include "msl/random.h"

void MSLRandomSource::init_table()
{
  table[ 0] = -1708027847;
  table[ 1] =   853131300;
  table[ 2] = -1687801470;
  table[ 3] =  1570894658;
  table[ 4] =  -566525472;
  table[ 5] =  -552964171;
  table[ 6] =  -251413502;
  table[ 7] =  1223901435;
  table[ 8] =  1950999915;
  table[ 9] = -1095640144;
  table[10] = -1420011240;
  table[11] = -1805298435;
  table[12] = -1943115761;
  table[13] =  -348292705;
  table[14] = -1323376457;
  table[15] =   759393158;
  table[16] =  -630772182;
  table[17] =   361286280;
  table[18] =  -479628451;
  table[19] = -1873857033;
  table[20] =  -686452778;
  table[21] =  1873211473;
  table[22] =  1634626454;
  table[23] = -1399525412;
  table[24] =   910245779;
  table[25] =  -970800488;
  table[26] =  -173790536;
  table[27] = -1970743429;
  table[28] =  -173171442;
  table[29] = -1986452981;
  table[30] =   670779321;

  ptr0    = table;
  ptr1    = table + 3;
  ptr_end = table + 31;
}


void MSLRandomSource::set_seed(int x)
{ table[0] = (unsigned)x;
  int i;
  for(i=1; i<31; i++) table[i] = 1103515245 * table[i-1] + 12345;
  ptr0 = table;
  ptr1 = table + 3;
  for(i=0; i<310; i++) get_rand31();
}


#define RANDMAX 0x7FFFFFFF

unsigned long MSLRandomSource::get_rand31()
{ *ptr1 += *ptr0;
  long i = (*ptr1 >> 1) & 0x7fffffff;
  if (++ptr0 >= ptr_end) ptr0 = table;
  if (++ptr1 >= ptr_end) ptr1 = table;
  return (unsigned long)i;
}

unsigned long MSLRandomSource::get_rand32()
{ if (buffer == 0x00000001) buffer = 0x80000000 | get_rand31();
  unsigned long result = get_rand31();
  if (buffer&1) result |= 0x80000000;
  buffer >>= 1;
  return result;
}



int MSLRandomSource::count = 0;

MSLRandomSource::MSLRandomSource()
{ init_table();
  time_t seed;
  time(&seed);
  count++;
  set_seed(int(seed)*count);
  pat = 0xFFFFFFFF;
  prec = 31;
  bit_mode = true;
  low = diff = 0;
  buffer = 0x00000001;
 }

MSLRandomSource::MSLRandomSource(int bits)
{ init_table();
  time_t seed;
  time(&seed);
  count++;
  set_seed(int(seed)*count);
  if (bits <= 0 || bits >= 32)
     pat = 0xFFFFFFFF;
  else
     pat = (1 << bits) - 1;
  prec = bits;
  bit_mode = true;
  low = diff = 0;
  buffer = 0x00000001;
 }

void MSLRandomSource::set_range(int l, int h)
{ low  = l;
  diff = (h < l) ? 0 : h-l;
  bit_mode = false;
  pat = 1;
  while (pat <= unsigned(diff)) pat <<= 1;
  pat--;
 }


MSLRandomSource::MSLRandomSource(int l, int h)
{ init_table();
  time_t seed;
  time(&seed);
  count++;
  set_seed(int(seed)*count);
  set_range(l,h);
  buffer = 0x00000001;
 }


int MSLRandomSource::set_precision(int bits)
{ int oldprec = prec;
  prec = bits;
  if (0 >= bits || bits >= 32)
     pat = 0xFFFFFFFF;
  else
     pat = (1 << bits) - 1;
  bit_mode = true;
  return oldprec;
 }


int MSLRandomSource::operator()()
{ unsigned long x =  get_rand31() & pat;
  if (bit_mode)
     return int(x);
  else
    { while (x > unsigned(diff)) x = get_rand31() & pat;
      return (int)(low + x);
     }
 }

int  MSLRandomSource::operator()(int bits)
{ unsigned long mask = (1<<bits) - 1;
  int shift = 31 - bits;
  if (shift < 0) shift = 0;
  return int((get_rand31() >> shift) & mask);
}


int  MSLRandomSource::operator()(int l, int h)
{ if (h < l) return 0;

  int  old_low = low;
  int  old_diff = diff;
  unsigned long  old_pat = pat;
  bool old_bit_mode = bit_mode;

  set_range(l,h);
  int x  = operator()();

  low = old_low;
  diff = old_diff;
  pat = old_pat;
  bit_mode = old_bit_mode;

  return x;
}



MSLRandomSource& MSLRandomSource::operator>>(bool& x)
{ x = (get_rand31() & (1<<16)) != 0;
  return *this;
 }

#if defined(__HAS_BUILTIN_BOOL__)
MSLRandomSource& MSLRandomSource::operator>>(char& x)
{ x = char(operator()());
  return *this;
 }
#endif

MSLRandomSource& MSLRandomSource::operator>>(int& x)
{ x = int(operator()());
  return *this;
}

MSLRandomSource& MSLRandomSource::operator>>(long& x)
{ x = long(operator()());
  return *this;
}



MSLRandomSource& MSLRandomSource::operator>>(unsigned char& x)
{ x = (unsigned char)(operator()());
  return *this;
}


MSLRandomSource& MSLRandomSource::operator>>(unsigned int& x)
{ x = (unsigned int)(operator()());
  return *this;
}

MSLRandomSource& MSLRandomSource::operator>>(unsigned long& x)
{ x = (unsigned long)(operator()());
  return *this;
}



MSLRandomSource& MSLRandomSource::operator>>(float&  x)
{ x = float(get_rand31())/RANDMAX;
  return *this;
}



MSLRandomSource& MSLRandomSource::operator>>(double& x)
{ x = double(get_rand31())/RANDMAX;
  return *this;
}
