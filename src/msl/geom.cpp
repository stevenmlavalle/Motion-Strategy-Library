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

//#include <fstream.h>
#include <math.h>

#include "msl/geom.h"
#include "msl/defs.h"


Geom::Geom(string path = "") {

  if ((path.length() > 0)&&(path[path.length()-1] != '/'))
    path += "/";
  FilePath = path;

  READ_PARAMETER_OR_DEFAULT(GeomDim,3);

}



MSLVector Geom::ConfigurationDifference(const MSLVector &q1,
					const MSLVector &q2)
{
  return (q2 - q1);  // By default, assume no topological problems
}



// *********************************************************************
// *********************************************************************
// CLASS:     GeomNone
//
// *********************************************************************
// *********************************************************************

GeomNone::GeomNone(string path = ""):Geom(path) {
  NumBodies = 1; // Default
}
