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

#ifndef MSL_RENDER_H
#define MSL_RENDER_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "scene.h"
#include "gui.h"  // Seems circular, but needed because of mutual reference
#include "mslio.h"
#include "util.h"

class Gui; // This needs to be here because of mutual reference

#define RENDERCOLORS 10  // Number of colors

//! A rendering class that accepts commands from a Gui, and determines
//! using specific graphics libraries how to draw the results on a 
//! screen.

/*!  This hierarchy of classes contains different implementations of
graphical rendering requests.  For example, when a graphical user
interface (GUI) requests that the a solution path is animated, a
method in a Render class displays the bodies in motion using
configurations obtained from the Scene class.  Each derived class in
Render corresponds to a different graphics system.  Presently, there
are renderers for Open Inventor and Open GL.  The
flexibility provided by these classes enables easy extensions to be
made for other graphics libraries and platforms, such as Open
Inventor.  

The rendering is expressed in terms of a Scene, in which the 
scene configuration gives the configuration of a collection of
Bodies in a static Environment.
*/

class Render 
{
 protected:
  //! Allow information from a Scene to be accessed
  Scene *S; 

  //! This uses the FrameList to return the SceneConfiguration that is supposed
  //! to be shown at the present time.
  virtual void SetCurrentAnimationFrame();

  //! Display the current animation frame in a rendering window.
  virtual void ShowCurrentAnimationFrame() {};

  //! A sequence of state to use for animation (should be generated by Gui)
  list<MSLVector> StateList;

  //! The time stamps for the state sequence
  list<double> TimeList;

  list<string> EnvList; // File names of all stationary environment objects
  list<string> BodyList; // File names of all movable bodies

  //! RGB color values
  float RGBRed[RENDERCOLORS]; 
  float RGBGreen[RENDERCOLORS];
  float RGBBlue[RENDERCOLORS]; 

  public:
  //! The path name for accessing files
  string FilePath;

  //! The animation frames; each element is a SceneConfiguration
  list<MSLVector> FrameList;

  //! The amount of time for which a frame is shown
  double FrameTime;

  //! The time stamp of the last frame change
  float LastFrameTime;

  //! The amount of time since the last frame change 
  float FrameStuckTime;

  //! The number of frames in the animation
  int NumFrames;

  //! Number of seconds to wait at the start of an animation
  double AnimationStartPause;

  //! Number of seconds to wait at the end of an animation
  double AnimationEndPause;

  //! The speedup factor for the animation (1.0 = normal speed)
  double AnimationTimeScale;
  
  //! The index in FrameList of the frame that should be currently shown
  int AnimationFrameIndex;

  //! The frame that should be shown currently
  MSLVector CurrentAnimationFrame;

  //! Set to true to start the animation
  bool AnimationActive;

  //! Set to true if the renderer needs main loop control; otherwise,
  //! iterated polling of HandleEvents will be performed automatically.
  bool ControlFreak;

  //! The rendering control window
  //  window *RenderCtlWindow;

  //! Is teh rendering control window currently displayed?
  bool RenderCtlWindowOn;

  //! Set to true for the viewpoint to be attached to a body
  bool AttachedCameraOn;

  //! Set to true to draw a bounding box using S->LowerWorld and S->UpperWorld
  bool BoundingBoxOn;

  //! Set to true to show multiple views simultaneously
  bool MultipleViewsOn;

  //! Set to true to show the entire path
  bool ShowPathOn;

  //! A parameter that controls the amount of light
  double AmbientLight;

  Render();
  Render(string filepath);
  Render(Scene *s, string filepath);
  virtual ~Render() {}

  //! Initialized the renderer
  virtual void Init();

  //! Process IO events
  virtual void HandleEvents() {};
  
  //! If ControlFreak = true, then MainLoop is entered here
  virtual void MainLoop(Gui *g);

  //! Reset the renderer
  virtual void Reset();

  //! Implement functions upon termination of the renderer
  virtual void Terminate() {};

  //! Make the set of frames from StateList and TimeList
  virtual void SetFrameList();

  //! Generate FrameList and set AnimationActive to true.
  virtual void MakeAnimationFrames(const list<MSLVector> &xlist, double deltat);
  virtual void MakeAnimationFrames(const list<MSLVector> &xlist, 
		       const list<double> &timelist);

  //! Display an entire path (the specific renderer determines how)
  virtual void DrawPath() {};

  //! Execute actions for render control window choices
  virtual void ButtonHandle(int b);

  //! Change the associated scene
  void SetScene(Scene *s);

  //! Open or close a window that controls the renderer
  //void ToggleRenderCtlWindow();
};

#endif
