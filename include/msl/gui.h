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

#ifndef MSL_GUI_H
#define MSL_GUI_H

// FOX GUI library
#include <fx.h>

#include "problem.h"
#include "scene.h"
#include "render.h"
#include "util.h"

class Render;  // This needs to be here because of mutual reference

//! A generic class for designing graphical user interfaces (GUIs)
/*!
The graphical user interface (GUI) is designed as a hierarchy of
classes to enable specific user interfaces to be designed for a
variety of different motion strategy problems and planning algorithms.
Currently, there is one derived class which serves as the GUI for all
of the RRT-based planners.  Each instance of Gui includes an instance
of an RRT Planner class and an instance of a Render class.  Using this
design, the same basic GUI design can be used, regardless of the
particular rendering methods.  
*/

class Gui {
 protected:
  string FilePath;

  //! Make the menu window
  virtual void CreateWindow() {};

  //! Initialize Gui and Render
  virtual void Init();

  //! The main event processing loop
  virtual void MainLoop();

  //! The window

 public:
  Gui(Render *render);
  virtual ~Gui() {};

  Render *R;

  //! Start running the Gui
  virtual void Start();

  //! Set to true if you want to main loop to terminate
  bool Finished;

  //! Process any IO events (may be used by Render)
  virtual void HandleEvents() = 0;

  //! Figure out what actions to take based on menu choices
  virtual void ButtonHandle(int b) {};
};


// ID numbers for GUIs
// (a simple, not-so-elegant way to avoid name ID conflicts)

enum {
  // Special IDs used by Render
  GID_RENDER_FIRST = FXMainWindow::ID_LAST,
  GID_TOGGLE_SHOWPATH,
  GID_TOGGLE_BOUNDINGBOX,
  GID_TOGGLE_MULTIPLEVIEWS,
  GID_TOGGLE_ATTACHEDCAMERA,
  GID_VCR_STOP,
  GID_VCR_LAST,
  GID_VCR_PAUSE,
  GID_VCR_NEXT,
  GID_VCR_SLOWER,
  GID_VCR_PLAY,
  GID_VCR_FASTER,
  GID_VCR_RESET,
  GID_RENDER_LAST,
  
  // General Gui IDs
  GID_CONSTRUCT,
  GID_PLAN,
  GID_CLEAR_GRAPHS,
  GID_2D_GRAPH,
  GID_SAVE_GRAPHS,
  GID_LOAD_GRAPHS,
  GID_SAVE_FRAMES,
  GID_LOAD_FRAMES,
  GID_SAVE_PATH,
  GID_LOAD_PATH,
  GID_DONE,
  
  GID_RRT,
  GID_RRTGOALBIAS,
  GID_RRTCON,
  GID_RRTDUAL,
  GID_RRTEXTEXT,
  GID_RRTEXTCON,
  GID_RRTCONCON,
  GID_RCRRT,
  GID_RCRRTEXTEXT,
  GID_RRTBIDIRBALANCED,
  GID_PRM,
  GID_FDP,
  GID_FDPSTAR,
  GID_FDPBESTFIRST,
  GID_FDPBI,

  GID_LAST
};



#endif
