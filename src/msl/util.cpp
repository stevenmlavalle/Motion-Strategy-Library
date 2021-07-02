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

#if defined(__unix__)
#include <unistd.h>
#include <pwd.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#endif

#ifdef WIN32
#include <time.h>
#endif

#include "msl/util.h"

float used_time()
{
#if defined(__unix__)

#if defined(CLK_TCK)
  long clk_tck = CLK_TCK;
#else
  //  long clk_tck = HZ;
long clk_tck = sysconf(_SC_CLK_TCK);
#endif

  tms x;
  times(&x);
  return  float(x.tms_utime)/clk_tck;

#else

  return  float(clock())/CLOCKS_PER_SEC;

#endif
}


float used_time(float& T)
{ float t = T;
  T = used_time();
  return  T-t;
}

/*
ostream& operator<<(ostream& out, const list<string>& L)
{
  list<string>::iterator x;
  list<string> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++)
    out << *x << "\n";
  return out;
}


istream& operator>>(istream& in, list<string>& L)
{
  L.clear();
  string x;
  for(;;)
    {
      char c;
      while (in.get(c) && isspace(c));
      if (!in) break;
      in.putback(c);
      x = string(); in >> x; L.push_back(x);
    }
  return in;
}

ostream& operator<<(ostream& out, const list<int>& L)
{
  list<int>::iterator x;
  list<int> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++)
    out << *x << "\n";
  return out;
}


istream& operator>>(istream& in, list<int>& L)
{
  L.clear();
  int x;
  for(;;)
    {
      char c;
      while (in.get(c) && isspace(c));
      if (!in) break;
      in.putback(c);
      in >> x; L.push_back(x);
    }
  return in;
}
*/


bool is_file(string fname)
{ struct stat stat_buf;
  if (stat(fname.c_str(),&stat_buf) != 0) return false;
  return (stat_buf.st_mode & S_IFMT) == S_IFREG;
}


bool is_directory(string fname)
{ struct stat stat_buf;
  if (stat(fname.c_str(),&stat_buf) != 0) return false;
  return (stat_buf.st_mode & S_IFMT) == S_IFDIR;
}
