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

#ifndef MSL_RENDERIV_H
#define MSL_RENDERIV_H


// MSL includes
#include "triangle.h"
#include "render.h"
#include "point.h"

// Inventor classes
class SoSeparator;
class SoXtExaminerViewer;
class SoSensor;
class SoTransform;
class SoSwitch;
class SoVertexProperty;

//-----------------------------------------------------------------
//                       Class RenderIv
//
//! Perform 3D rendering using the OpenInventor library
//
//-----------------------------------------------------------------
class RenderIv: public Render
{
public: 
  
  // Constructors & Destructor
  RenderIv();
  RenderIv(string filepath);
  RenderIv(Scene *s, string filepath);
  virtual ~RenderIv();
  
  // method to reset the scene
  virtual void Reset();
 
  // initialization
  virtual void Init();
  virtual void MainLoop(Gui *g);
  
  
protected:

 // event callbacks
  static void  _TimerCB(void* userData, SoSensor*);
  inline void  _IdleFunction();

 // initialization
  SoSeparator* _ReadIvFile(const char *filename);
  SoSeparator* _InitObject(const string &fname);
  bool         _InitBoundsDisplay();
  bool         _InitPathDisplay();
  SoSeparator* _InitTriangleGeom(list<MSLTriangle> &triangles);
  bool         _InitData();

 // helper methods
  inline void  _SetSwitch(SoSwitch *pSwitch, bool bFlag);
  inline void  _UpdatePathDisplay();
  inline void  _SetTransform(SoTransform* pTrans,
			     double tx, double ty, double tz,
			     double rx, double ry, double rz);
  inline void  _UpdateBodies(const MSLVector &qConfig);


 // Data

  SoXtExaminerViewer*  _viewer;         // the viewer window
  Gui*                 _pGui;           // gui pointer

  SoSeparator*         _ivRoot;         // scene graph root
  SoSeparator*         _ivData;         // root of all scene data

  SoSwitch*            _ivBoundsSwitch; // workspace boundary display
  bool                 _bDisplayBounds;

  SoSwitch*            _ivPathSwitch;   // path display
  SoVertexProperty*    _pPathVertexProp;
  int                  _pathFrames;
  bool                 _bDisplayPath;

  list<SoTransform*>   _bodyTrans;      // list of body transforms
};



#endif
