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

// RenderPerformer was written by Libo Yang (lyang@cs.iastate.edu)
// Modifications made by Steve LaValle (lavalle@cs.iatate.edu)

#ifndef MSL_RENDERPF_H
#define MSL_RENDERPF_H

#include "triangle.h"
#include "render.h"

#include <Performer/pf/pfChannel.h>
#include <Performer/pf/pfEarthSky.h>
#include <Performer/pf/pfLightSource.h>
#include <Performer/pf/pfNode.h>
#include <Performer/pf/pfGeode.h>
#include <Performer/pf/pfText.h>
#include <Performer/pf/pfScene.h>
#include <Performer/pf/pfGroup.h>
#include <Performer/pf/pfSwitch.h>
#include <Performer/pf/pfDCS.h>
#include <Performer/pf/pfSCS.h>
#include <Performer/pr/pfGeoSet.h>
#include <Performer/pr/pfGeoState.h>
#include <Performer/pr/pfString.h>
#include <Performer/pr/pfFont.h>
#include <Performer/pr/pfMaterial.h>
#include <Performer/pfui.h>
#include <Performer/pfutil.h>
#include <Performer/pfdu.h>
#include <Performer/pr.h>

typedef struct{
  pfPipeWindow *PW;
  pfChannel *Chan;
  pfChannel *ChanEye;
  pfSwitch *ShowCase;
  pfGroup *ControlPanel;
  pfDCS *ControlPad;
  pfDCS *BodiesDCS;
  pfGroup *Env;
  pfGeode *BoundingBox;
  pfSwitch *WorkEnv;
  pfDCS *WorldDCS;
  pfuMouse *Mouse;
  pfuEventStream *InputEvents;
  //static pfLightSource *Sun;

  // Display Control
  double TranX, TranY,TranZ; // Global tranlation offset
  double RotX, RotY, RotZ;  //Global rotation offset

  // Mouse Control
  bool HoldRightMouse, HoldLeftMouse, HoldMiddleMouse;
  //bool ReleaseRightMouse, ReleaseLeftMouse, ReleaseMiddleMouse;
  double MouseXOld, MouseYOld; // Mouse's previous position

  //Control of Eye
  pfMatrix EyeMat;

  //Control of Control panel
  bool ControlPanelOn;
  bool FocusOnControlPad;
  double PadX; 
} SharedData;




//! Perform 3D rendering using the SGI IRIS Performer library
class RenderPerformer: public Render
{
 protected:
  static SharedData *Shared;

  virtual void ShowCurrentAnimationFrame();

  // Loaders
  void LoadEnvironment(pfGroup *env);
  void LoadBodies(pfGroup *bodies);

  //! Load a model that is a list of 2D polygons or 3D triangles
  pfNode* LoadNativeModel(string file, int colorindex);

  void MakeBoundingBox(pfGeode *bound);
  void MakeControlPanel(pfGroup *con, pfDCS *pad);

  void GetCurrentMousePos(double &x, double &y);
  void HandleKeyInput();
  void HandleMouseEvents();
  void GetControlPadSize(double &padwidth_l, double &padwidth_r, 
			 double &padheight_b, double &padheight_top);
  void NormCrossProduct(double v1[3], double v2[3], double out[3]);
  static void DrawChannel(pfChannel *chan, void *data);

 public:
  RenderPerformer();
  RenderPerformer(string filepath);
  RenderPerformer(Scene *s, string filepath);
  virtual ~RenderPerformer() {};

  virtual void Init();
  virtual void HandleEvents();
  virtual void Terminate();
};

#endif
