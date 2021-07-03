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

// Standard includes
#include <math.h>
#include <stdlib.h>
#include <assert.h>

// OpenInventor includes
#include <Xm/Xm.h>
#include <Inventor/Xt/SoXt.h>
#include <Inventor/SoDB.h>
#include <Inventor/Xt/viewers/SoXtExaminerViewer.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/fields/SoMFInt32.h>

// MSL includes
#include "msl/defs.h"

#include "msl_gui/renderiv.h"

// default viewer dimensions
const short DEF_VIEWER_WIDTH  = 500;
const short DEF_VIEWER_HEIGHT = 400;

//---------------------------------------------------------------------
//                       RenderIv
//
// Method:  Constructors
//
//---------------------------------------------------------------------
RenderIv::RenderIv(): Render()
{
  ControlFreak = true;
  _ivRoot = NULL;
  _ivData = NULL;
}


RenderIv::RenderIv(string filepath=""): Render(filepath)
{
  ControlFreak = true;
  _ivRoot = NULL;
  _ivData = NULL;
}


RenderIv::RenderIv(Scene *s, string filepath): Render(s,filepath)
{
  ControlFreak = true;
  _ivRoot = NULL;
  _ivData = NULL;
}


//---------------------------------------------------------------------
//                       RenderIv
//
// Method:  Destructor
//
//---------------------------------------------------------------------
RenderIv::~RenderIv()
{
  if (_ivRoot)
    _ivRoot->unref();
  // inventor garbage collection should take care of deleting
  // this and all of the other scene graph nodes
}


//---------------------------------------------------------------------
//                       RenderIv
//
// Method:  Reset
//
//---------------------------------------------------------------------
void RenderIv::Reset()
{
  // Use reset from base class
  Render::Reset();

  // clear all data
  if (_ivData)
    _ivData->removeAllChildren();
}



//---------------------------------------------------------------------
//                       RenderIv
//
// Method:  Init
//
//---------------------------------------------------------------------
void RenderIv::Init()
{
  Render::Init();

  // initialize Xt and Inventor
  const char* MAIN_WINDOW_TITLE = "MSL Library   University of Illinois";
  Widget mainWindow = SoXt::init(MAIN_WINDOW_TITLE);
  if (!mainWindow) {
    exit(1);
  }

  //create the root
  _ivRoot = new SoSeparator;
  _ivRoot->ref();

  // create the main perspective viewer
  _viewer = new SoXtExaminerViewer(mainWindow);
  _viewer->setTitle(MAIN_WINDOW_TITLE);
  _viewer->setSceneGraph(_ivRoot);
  _viewer->setSize(SbVec2s(DEF_VIEWER_WIDTH, DEF_VIEWER_HEIGHT));
  _viewer->show();

  // Sest default view for the camera
  float defaultCam[3] = { 0.0, 3.0, 5.0 };
  if (S->GeomDim == 2)
    defaultCam[1] = -defaultCam[1];
  SoCamera *camera = _viewer->getCamera();
  camera->position.setValue(defaultCam[0], defaultCam[1], defaultCam[2]);
  camera->pointAt(SbVec3f(0.0, 0.0, 0.0));
  camera->focalDistance.setValue(camera->position.getValue().length());
  _viewer->saveHomePosition();

  // set the background color
  SbColor ivColor(0,0,0);
  _viewer->setBackgroundColor(ivColor);

  // set up the timer callback
  SoTimerSensor *callbackSensor = new SoTimerSensor(_TimerCB, this);
  callbackSensor->setInterval(1/2000.0);  // as fast as possible
  callbackSensor->schedule();

  // initialize the data
  _ivData = new SoSeparator;
  _ivRoot->addChild(_ivData);
  if (!_InitData()) cerr << "renderiv: Error initializing data" << endl;
  _viewer->viewAll();

  // show the window
  SoXt::show(mainWindow);
}


//---------------------------------------------------------------------
//                           RenderIv
// Method:  MainLoop
//
//---------------------------------------------------------------------
void RenderIv::MainLoop(Gui *g)
{
  g->Finished = false;
  _pGui = g;

  // enter the inventor loop
  SoXt::mainLoop();
}


////////////////
//
// PROTECTED METHODS
//


//---------------------------------------------------------------------
//                           RenderIv
// Method:  _TimerCB
//
// Purpose: Callback for the timer sensor
//
//---------------------------------------------------------------------
void RenderIv::_TimerCB(void *userData, SoSensor *)
{
  RenderIv *riv = (RenderIv *)userData;
  // execute idle function
  riv->_IdleFunction();
}



//---------------------------------------------------------------------
//                           RenderIv
// Method:  _TimerCB
//
// Purpose: Callback for the timer sensor
//
//---------------------------------------------------------------------
inline void RenderIv::_IdleFunction()
{
  // Handle the window events for the Gui
  _pGui->HandleEvents();

  // Allow exiting by pressing Exit button in Gui
  if (_pGui->Finished) exit(-1);

  // update display toggles
  if (_bDisplayBounds != BoundingBoxOn) {
    _bDisplayBounds = BoundingBoxOn;
    _SetSwitch(_ivBoundsSwitch, _bDisplayBounds);
  }
  if (_bDisplayPath != ShowPathOn) {
    _bDisplayPath = ShowPathOn;
    _SetSwitch(_ivPathSwitch, _bDisplayPath);
  }
  if (_bDisplayPath && NumFrames != _pathFrames)
    _UpdatePathDisplay();

  // update body state if animation is on
  if (AnimationActive) {
    SetCurrentAnimationFrame();
    _UpdateBodies(CurrentAnimationFrame);
  }
}


//---------------------------------------------------------------------
//                            RenderIv
// Method:  _ReadIvFile
//
// Purpose: Read Inventor scene from a file
//
//---------------------------------------------------------------------
SoSeparator* RenderIv::_ReadIvFile(const char *filename)
{
  // Open the input file
  SoInput sceneInput;
  if (!sceneInput.openFile(filename)) {
    cerr << "Cannot open file: " << filename << endl;
    return NULL;
  }

  // Read the whole file into the database
  SoSeparator *graph = SoDB::readAll(&sceneInput);
  if (graph == NULL) {
    cerr << "Problem reading file: " << filename << endl;
    return NULL;
  }

  sceneInput.closeFile();
  return graph;
}


//---------------------------------------------------------------------
//                           RenderIv
// Method:  _InitObject
//
// Purpose: Load a single object
//
//---------------------------------------------------------------------
SoSeparator* RenderIv::_InitObject(const string &fname)
{
  SoSeparator* pObject = NULL;
  cout << "  loading file: " << fname << endl;

  // check for native inventor file
  if (fname.substr(fname.length()-3,3) == ".iv")
    pObject = _ReadIvFile(fname.c_str());
  else {
    // otherwise load raw geometry
    list<MSLTriangle> trlist;
    std::ifstream fin(fname.c_str());

    if (S->GeomDim == 2) {
      list<MSLPolygon> plist;
      fin >> plist;
      trlist = PolygonsToTriangles(plist, 2.0); // Defined in 3Dtriangle.C
    }
    else
      fin >> trlist;
    pObject = _InitTriangleGeom(trlist);
  }

  return pObject;
}


//---------------------------------------------------------------------
//                           RenderIv
// Method:  _InitBoundsDisplay
//
// Purpose: initialize workspace boundary visualization
//
//---------------------------------------------------------------------
bool RenderIv::_InitBoundsDisplay()
{
  // build configuration bounds visualization nodes
  SoVertexProperty* pVertexProp = new SoVertexProperty;
  SoLineSet *lines = new SoLineSet;
  lines->vertexProperty = pVertexProp;
  _ivBoundsSwitch = new SoSwitch(1);
  _ivBoundsSwitch->addChild(lines);
  _ivRoot->addChild(_ivBoundsSwitch);
  _bDisplayBounds = false;
  // display the bounds if flag is set
  _SetSwitch(_ivBoundsSwitch, _bDisplayBounds);

  // set the color
  const float color[3] = { 0.5, 0.5, 0.5};
  uint32_t red   = uint32_t(color[0] * 255) << 24;
  uint32_t green = uint32_t(color[1] * 255) << 16;
  uint32_t blue  = uint32_t(color[2] * 255) << 8;
  uint32_t alpha = 0x000000FF;
  uint32_t packedColor = red | green | blue | alpha;
  pVertexProp->orderedRGBA.setValue(packedColor);

  // default bounding box
  float L[3] = { -10, -10, -10 };
  float U[3] = { 10, 10, 10 };

  // get the workspace bounds
  if (S->LowerWorld[0] != S->UpperWorld[0]) {
    L[0]= S->LowerWorld[0];  L[1]= S->LowerWorld[1];  L[2]= S->LowerWorld[2];
    U[0]= S->UpperWorld[0];  U[1]= S->UpperWorld[1];  U[2]= S->UpperWorld[2];
  }
  cerr << "Workspace boundary: ( " << L[0] << ", " << L[1] << ", " << L[2]
       << " ) - ( "  << U[0] << ", " << U[1] << ", " << U[2] << " )" << endl;

  // set the bounding box lines
  const int numPoints = 16;
  float points[numPoints][3] = {
    {L[0],L[1],L[2]}, {L[0],L[1],U[2]}, {U[0],L[1],U[2]}, {U[0],L[1],L[2]},
    {L[0],L[1],L[2]}, {L[0],U[1],L[2]}, {L[0],U[1],U[2]}, {U[0],U[1],U[2]},
    {U[0],U[1],L[2]}, {L[0],U[1],L[2]}, {U[0],U[1],L[2]}, {U[0],L[1],L[2]},
    {U[0],L[1],U[2]}, {U[0],U[1],U[2]}, {L[0],U[1],U[2]}, {L[0],L[1],U[2]}
  };
  pVertexProp->vertex.setNum(numPoints);
  pVertexProp->vertex.setValues(0, numPoints, points);

  return true;
}


//---------------------------------------------------------------------
//                           RenderIv
// Method:  _InitPathDisplay
//
// Purpose: initialize path visualization
//
//---------------------------------------------------------------------
bool RenderIv::_InitPathDisplay()
{
  // build configuration bounds visualization nodes
  _pPathVertexProp = new SoVertexProperty;
  SoLineSet *lines = new SoLineSet;
  lines->vertexProperty = _pPathVertexProp;
  _ivPathSwitch = new SoSwitch(1);
  _ivPathSwitch->addChild(lines);
  _ivRoot->addChild(_ivPathSwitch);
  _bDisplayPath = false;
  _pathFrames = 0;
  // display the bounds if flag is set
  _SetSwitch(_ivPathSwitch, _bDisplayPath);

  // set the color
  const float color[3] = { 1.0, 1.0, 0.2};
  uint32_t red   = uint32_t(color[0] * 255) << 24;
  uint32_t green = uint32_t(color[1] * 255) << 16;
  uint32_t blue  = uint32_t(color[2] * 255) << 8;
  uint32_t alpha = 0x000000FF;
  uint32_t packedColor = red | green | blue | alpha;
  _pPathVertexProp->orderedRGBA.setValue(packedColor);

  // default bounding box
  float L[3] = { -3, -3, -1 };
  float U[3] = { 2, 3, 1 };

  // reset to empty path
  const int numPoints = 16;
  float points[numPoints][3] = {
    {L[0],L[1],L[2]}, {L[0],L[1],U[2]}, {U[0],L[1],U[2]}, {U[0],L[1],L[2]},
    {L[0],L[1],L[2]}, {L[0],U[1],L[2]}, {L[0],U[1],U[2]}, {U[0],U[1],U[2]},
    {U[0],U[1],L[2]}, {L[0],U[1],L[2]}, {U[0],U[1],L[2]}, {U[0],L[1],L[2]},
    {U[0],L[1],U[2]}, {U[0],U[1],U[2]}, {L[0],U[1],U[2]}, {L[0],L[1],U[2]}
  };
  _pPathVertexProp->vertex.setNum(numPoints);
  _pPathVertexProp->vertex.setValues(0, numPoints, points);

  return true;
}


//---------------------------------------------------------------------
//                           RenderIv
//
// Method:  _InitTriangleGeom
//
// Purpose: initialize object geometry from triangles
//
//---------------------------------------------------------------------
SoSeparator* RenderIv::_InitTriangleGeom(list<MSLTriangle> &triangles)
{
  int numTriangles = triangles.size();
  if (numTriangles <= 0) {
    cerr << "renderiv: WARNING - empty triangle list." << endl;
    return NULL;
  }

  // build iv hierarchy
  SoSeparator* pSep = new SoSeparator;
  SoMaterial* pMat = new SoMaterial;
  SoVertexProperty* pVertexProp = new SoVertexProperty;
  SoIndexedFaceSet* pFaceSet = new SoIndexedFaceSet;
  pSep->addChild(pMat);
  pSep->addChild(pFaceSet);

  // Change the colors for different bodies
  static int objnum = 0;
  objnum++;
  SbColor c(RGBRed[(objnum) % RENDERCOLORS],
	    RGBGreen[(objnum) % RENDERCOLORS],
	    RGBBlue[(objnum) % RENDERCOLORS]);
  pMat->diffuseColor.setValue(c[0], c[1], c[2]);

  // set the vertex data
  pFaceSet->vertexProperty = pVertexProp;
  const int numVerts = numTriangles * 3;
  typedef float float3[3];
  float3* points = new float3[numVerts];
  int v = 0;
  list<MSLTriangle>::iterator t;
  forall(t, triangles) {
    points[v][0] = t->p1.xcoord();
    points[v][1] = t->p1.ycoord();
    points[v][2] = t->p1.zcoord();
    v++;
    points[v][0] = t->p2.xcoord();
    points[v][1] = t->p2.ycoord();
    points[v][2] = t->p2.zcoord();
    v++;
    points[v][0] = t->p3.xcoord();
    points[v][1] = t->p3.ycoord();
    points[v][2] = t->p3.zcoord();
    v++;
  }
  assert(v == numVerts);
  pVertexProp->vertex.setNum(numVerts);
  pVertexProp->vertex.setValues(0, numVerts, points);

  // set the triangle vertex indices
  const int numIndices = numTriangles * 4;
  int32_t *vIndex = new int32_t[numIndices];
  int i = 0;
  v = 0;
  for (int t = 0; t < numTriangles; t++) {
    vIndex[i++] = v++;  vIndex[i++] = v++;  vIndex[i++] = v++;
    vIndex[i++] = -1;
  }
  assert (v == numVerts);
  assert (i == numIndices);

  pFaceSet->coordIndex.setValues(0, numIndices, vIndex);
  pFaceSet->coordIndex.setNum(numIndices);
  delete [] vIndex;

  return pSep;
}


//---------------------------------------------------------------------
//                           RenderIv
// Method:  _InitData
//
// Purpose: Load all obstacles and body geometry
//
//---------------------------------------------------------------------
bool RenderIv::_InitData()
{
  // set control variables
  AnimationActive = false;

  // EnvList was initialized by Init in Render base class
  list<string>::iterator fname;
  forall(fname, EnvList) {
    SoSeparator* pObject = _InitObject(FilePath + *fname);
    if (!pObject)
      cerr << "renderiv: ERROR initializing obstacle: " << *fname << endl;
    else
      _ivRoot->addChild(pObject);
  }

  // Bodies
  forall(fname, BodyList) {
    SoSeparator* pObject = _InitObject(FilePath + *fname);
    if (!pObject)
      cerr << "renderiv: ERROR initializing body: " << *fname << endl;
    else {
      // create a movable transformation
      SoSeparator* objRoot  = new SoSeparator;
      SoTransform* objTrans = new SoTransform;
      objRoot->addChild(objTrans);
      objRoot->addChild(pObject);
      _ivRoot->addChild(objRoot);
      _bodyTrans.push_back(objTrans);
    }
  }

  // init the display items
  if (!_InitBoundsDisplay())
    cerr << "renderiv: ERROR initializing workspace boundary" << endl;
  if (!_InitPathDisplay())
    cerr << "renderiv: ERROR initializing path display" << endl;


  // update the current positions
  _UpdateBodies(CurrentAnimationFrame);

  return true;
}


//---------------------------------------------------------------------
//                           RenderIv
// Method:  _SetSwitch
//
// Purpose: set the value of a rendering switch
//
//---------------------------------------------------------------------
inline void RenderIv::_SetSwitch(SoSwitch *pSwitch, bool bFlag)
{
  pSwitch->whichChild.setValue(bFlag ? SO_SWITCH_ALL : SO_SWITCH_NONE);
}


//---------------------------------------------------------------------
//                           RenderIv
//
// Method:  _UpdatePathDisplay
//
// Purpose: update the path visualization
//
//---------------------------------------------------------------------
void RenderIv::_UpdatePathDisplay()
{
  _pathFrames = NumFrames;

  // check for empty path
  if (NumFrames < 2) {
    _pPathVertexProp->vertex.setNum(0);
    return;
  }

  MSLVector next, prev = FrameList.front();
  int BodyNum = prev.dim() / 6;

  // point allocation
  const int numPoints = BodyNum * ((NumFrames - 1) * 2);
  typedef float float3[3];
  float3* points = new float3[numPoints];

  // get all frame data
  int bInd;
  int p = 0;

  list<MSLVector>::iterator frp;
  frp = FrameList.begin();
  for (int i = 1; i < NumFrames; i++) {
    for (int j = 0; j < BodyNum; j++) {
      bInd = 6 * j;
      frp++; next = *frp;
      // set the endpoints
      points[p][0] = prev[bInd];
      points[p][1] = prev[bInd+1];
      points[p][2] = prev[bInd+2];
      p++;
      points[p][0] = next[bInd];
      points[p][1] = next[bInd+1];
      points[p][2] = next[bInd+2];
      p++;
      prev = next;
    }
  }
  assert(p == numPoints);

  // set the points
  _pPathVertexProp->vertex.setNum(numPoints);
  _pPathVertexProp->vertex.setValues(0, numPoints, points);
  delete [] points;
}


//---------------------------------------------------------------------
//                           RenderIv
//
// Method:  _SetBodyTransform
//
// Purpose: set a 3D transformation
//
//---------------------------------------------------------------------
inline void RenderIv::_SetTransform(SoTransform* pTrans,
				    double tx, double ty, double tz,
				    double rx, double ry, double rz)
{
  // set the rotation matrix from RPY euler angles
  float ca = cos(rx);   float sa = sin(rx);
  float cb = cos(ry);   float sb = sin(ry);
  float cc = cos(rz);   float sc = sin(rz);

  float R11 = cb * cc;
  float R12 = sa * sb * cc  -  ca * sc;
  float R13 = ca * sb * cc  +  sa * sc;
  float R21 = cb * sc;
  float R22 = sa * sb * sc  +  ca * cc;
  float R23 = ca * sb * sc  -  sa * cc;
  float R31 = -sb;
  float R32 = sa * cb;
  float R33 = ca * cb;

  // build row-major matrix
  SbMatrix matrix(R11,  R21,  R31,  0.0,
		  R12,  R22,  R32,  0.0,
		  R13,  R23,  R33,  0.0,
		  tx,   ty,   tz,   1.0);
  pTrans->setMatrix(matrix);
}


//---------------------------------------------------------------------
//                           RenderIv
//
// Method:  _UpdateBodies
//
// Purpose: update the body configuration
//
//---------------------------------------------------------------------
void RenderIv::_UpdateBodies(const MSLVector &qConfig)
{
  int index = 0;
  list<SoTransform*>::iterator pTrans;
  forall (pTrans, _bodyTrans) {
    _SetTransform(*pTrans, qConfig[index], qConfig[index+1], qConfig[index+2],
		  qConfig[index+3], qConfig[index+4], qConfig[index+5]);
    index += 6;
  }
  assert(index == qConfig.dim());
}
