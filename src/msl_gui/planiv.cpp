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
#include <stdlib.h>

#include "msl/rrt.h"
#include "msl/defs.h"
#include "msl/util.h"

#include "msl_gui/renderiv.h"
#include "msl_gui/guiplanner.h"
#include "msl_gui/setup.h"

int main(int argc, char *argv[])
{
  string path;
  GuiPlanner *gui;
  Model *m = NULL;
  Geom *g = NULL;
  Problem *prob;

  if (argc < 2) {
    cout << "Usage:    " << argv[0] << " <problem path>\n";
    exit(-1);
  }

  path = string(argv[1])+"/";

  if (!is_directory(path)) {
    cout << "Error:   Directory does not exist\n";
    exit(-1);
  }

  SetupProblem(m,g,path);

  prob = new Problem(g,m,path);

  gui = new GuiPlanner(new RenderIv(new Scene(prob, path), path),
		       new RRTConCon(prob));

  gui->Start();

  return 0;
}
