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
#include <stdio.h>
//#include <fstream.h>

#include "msl/defs.h"

#include "msl_gui/gui.h"

// *********************************************************************
// *********************************************************************
// CLASS:     Gui base class
//
// *********************************************************************
// *********************************************************************

Gui::Gui(Render *render) {
  R = render;

  FilePath = R->FilePath;
}


void Gui::Init()
{
  // Perform Render initialization
  R->Init();
}



void Gui::Start()
{
  // Perform initialization
  Init();

  // Enter the event processing loop
  MainLoop();
}




void Gui::MainLoop()
{
  int i;
  if (R->ControlFreak) // Does the renderer NEED to be in control?
    R->MainLoop(this); // Give a pointer to this Gui
    // It is the responsibility of Render to handle Gui events
  else {
    Finished = false;
    while (!Finished) {
      R->HandleEvents(); // Handle events in the renderer
      for (i = 0; i < 10; i++) // Need to make this better!!!!!
	HandleEvents();
    }
  }

  R->Terminate();  // If we ever get here (some control freaks don't allow it)
}
