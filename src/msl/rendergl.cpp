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

#include "msl/rendergl.h"

// Because the callback methods are static members that are not allowed by
// glut to take an arguments, we have to recover the class data.  There
// must be a better way...
RenderGL *RGL;


////////////////////////////////////////////////////////
//
// RenderGL Class
//
///////////////////////////////////////////////////////

RenderGL::RenderGL(): Render()
{
  WindowX = 600.0;
  WindowY = 600.0;
  WindowZ = 600.0;

  NumberOfObject = 0;
  NumberOfBody = 0;
  NumberOfEnvObj = 0;

  SceneBodyLib = new (mslGLObject *);
  SceneEnvObjLib = new (mslGLObject *);

  RGL = this;
  ControlFreak = true;
}


RenderGL::RenderGL(string filepath=""): Render(filepath)
{
  WindowX = 600.0;
  WindowY = 600.0;
  WindowZ = 600.0;

  NumberOfObject = 0;
  NumberOfBody = 0;
  NumberOfEnvObj = 0;

  SceneBodyLib = new (mslGLObject *);
  SceneEnvObjLib = new (mslGLObject *);

  RGL = this;
  ControlFreak = true;
}


RenderGL::RenderGL(Scene *s, string filepath): Render(s,filepath)
{
  WindowX = 600.0;
  WindowY = 600.0;
  WindowZ = 600.0;

  NumberOfObject = 0;
  NumberOfBody = 0;
  NumberOfEnvObj = 0;

  SceneBodyLib = new (mslGLObject *);
  SceneEnvObjLib = new (mslGLObject *);

  RGL = this;
  ControlFreak = true;
}


RenderGL::~RenderGL()
{
  // Delete a bunch of GL stuff
}


void RenderGL::AddBodyObject(mslGLObject * obj)
{

  if(SceneBodyLib == NULL)
    {
      if((SceneBodyLib = (mslGLObject **) malloc(sizeof(mslGLObject *))) == 0)
	{
	  printf("Error reallocating mem\n");
	  exit(-1);
	}
    }
  else
    if((SceneBodyLib = (mslGLObject **)
	realloc(SceneBodyLib, sizeof(mslGLObject *)*(NumberOfBody+1))) == 0)
      {
	printf("Error reallocating mem\n");
	exit(-1);
      }

  SceneBodyLib[NumberOfBody] = obj;
  SceneBodyLib[NumberOfBody]->ID = NumberOfObject;

  NumberOfBody++;
  NumberOfObject++;
}

void RenderGL::AddEnvObject(mslGLObject * obj)
{

  if(SceneEnvObjLib == NULL)
    {
      if((SceneEnvObjLib = (mslGLObject **) malloc(sizeof(mslGLObject *))) == 0)
	{
	  printf("Error reallocating mem\n");
	  exit(-1);
	}
    }
  else
    if((SceneEnvObjLib = (mslGLObject **)
	realloc(SceneEnvObjLib, sizeof(mslGLObject *) * (NumberOfEnvObj+1))) == 0)
      {
	printf("Error reallocating mem\n");
	exit(-1);
      }

  SceneEnvObjLib[NumberOfEnvObj] = obj;
  SceneEnvObjLib[NumberOfEnvObj]->ID = NumberOfObject;

  NumberOfEnvObj++;
  NumberOfObject++;
}


mslGLObject* RenderGL::WhichObject(int id)
{
  int i;

  for(i=0; i<NumberOfBody; i++)
    if(SceneBodyLib[i]->ID == id) return SceneBodyLib[i];

  for(i=0; i<NumberOfEnvObj; i++)
    if(SceneEnvObjLib[i]->ID == id) return SceneEnvObjLib[i];

  cout << "Object ID allocation error" << endl;
  exit(-1);
}


void RenderGL::Reset()
{
  // Use reset from base class
  Render::Reset();

  Fov = 45.0;      AspectRatio = 1.0;
  Near = 10.0;     Far = 100000.0;
  VupX = 0.0,      VupY = 1.0,          VupZ = 0.0;

  EyeX = (BoundingBoxMax[0] + BoundingBoxMin[0])/2.0;
  EyeY = (BoundingBoxMax[1] + BoundingBoxMin[1])/2.0;
  EyeZ = BoundingBoxMax[2] + (BoundingBoxMax[1]-BoundingBoxMin[1])/tan(38.0/180.0*PI);

  VpX = (BoundingBoxMax[0] + BoundingBoxMin[0])/2.0;
  VpY = (BoundingBoxMax[1] + BoundingBoxMin[1])/2.0;
  VpZ = (BoundingBoxMax[2] + BoundingBoxMin[2])/2.0;

  LightPosX = EyeX;
  LightPosY = EyeY + (BoundingBoxMax[1] - BoundingBoxMin[1]);
  LightPosZ = BoundingBoxMax[2] + (BoundingBoxMax[1]-BoundingBoxMin[1])/tan(38.0/180.0*PI)/2.0;

  Orientation[0] = 0.0;   Orientation[1] = 0.0;          Orientation[2] = 0.0;
  Position[0] = 0.0;    Position[1] = 0.0;           Position[2] = 0.0;
}


void RenderGL::ShowCoordinateFrame()
{
  MSLVector pos(3);
  MSLVector scenecenter(3);

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  glPushMatrix();

  scenecenter[0] = (RGL->BoundingBoxMin[0] + RGL->BoundingBoxMax[0])/2.0;
  scenecenter[1] = (RGL->BoundingBoxMin[1] + RGL->BoundingBoxMax[1])/2.0;
  scenecenter[2] = (RGL->BoundingBoxMin[2] + RGL->BoundingBoxMax[2])/2.0;

  glTranslatef(scenecenter[0], scenecenter[1], scenecenter[2]);

  GLfloat Diffuse[] = {1.0, 0.0, 0.0};
  GLfloat Ambient[] = {1.0, 0.0, 0.0};
  GLfloat Specular[] = {1.0, 0.0, 0.0};
  GLfloat Shininess[] = {1.0, 0.0, 0.0};

  glLineWidth(0.2);

  glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
  glMaterialfv(GL_FRONT, GL_AMBIENT, Ambient);
  glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, Shininess);

  glBegin(GL_LINES);

  // x axis
  glColor4f(1.0, 0.0, 0.0, 1.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(10.0, 0.0, 0.0);
  glVertex3f(10.0, 1.0, 0.0);
  glVertex3f(12.0, -1.0, 0.0);
  glVertex3f(10.0, -1.0, 0.0);
  glVertex3f(12.0, 1.0, 0.0);

  // y axis
  glColor4f(0.0, 1.0, 0.0, 1.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 10.0, 0.0);
  glVertex3f(0.707, 12.707, 0.0);
  glVertex3f(0.0, 12.0, 0.0);
  glVertex3f(-0.707, 12.707, 0.0);
  glVertex3f(0.0, 12.0, 0.0);
  glVertex3f(0.0, 11.0, 0.0);
  glVertex3f(0.0, 12.0, 0.0);

  // z axis
  glColor4f(0.0, 0.0, 1.0, 1.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 10.0);
  glVertex3f(-1.0, 1.0, 10.0);
  glVertex3f(1.0, 1.0, 10.0);
  glVertex3f(-1.0, -1.0, 10.0);
  glVertex3f(1.0, -1.0, 10.0);
  glVertex3f(1.0, 1.0, 10.0);
  glVertex3f(-1.0, -1.0, 10.0);

  glEnd();

  glLineWidth(1.0);
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);

  glPopMatrix();

}



void RenderGL::Init(){
  Render::Init();

  int argc = 2;
  char *argv[] = {"Dummy1","Dummy2"};

  glutInit(&argc,  argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize((GLint) WindowX, (GLint) WindowY);
  glutInitWindowPosition(200,0);
  MainWindow = glutCreateWindow("MSL Library   University of Illinois");

  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_NORMALIZE);

  GLfloat light_direction []= {0.5, 0.5, -1.5};
  GLfloat light_diffuse[] = {0.6, 0.4, 0.5, 1.0};
  GLfloat light_specular[] = {0.3, 0.6, 0.5, 1.0};
  GLfloat light_ambient[] = {AmbientLight, AmbientLight, AmbientLight, 1.0};
  glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light_direction);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  // setting the material properties
  GLfloat amb_diff [] = {0.3, 0.5, 0.7, 1.0};
  GLfloat specular [] = { 0.8, 0.5, 0.5, 1.0 };
  GLfloat shininess [] = { 60.0 };
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, amb_diff);
  glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, shininess);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);

  // read the configuration file to read appropriate model file
  InitData();

  glClearColor(0.0,  0.0,  0.0,  0.0);
  glViewport((GLint)0.0, (GLint)0.0, (GLsizei)WindowX, (GLsizei)WindowY);
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  gluPerspective(Fov, AspectRatio, Near, Far);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(EyeX, EyeY, EyeZ, VpX, VpY, VpZ, VupX, VupY, VupZ);

  glutReshapeFunc( GlutReshape );
  glutMouseFunc( GlutMouse );
  glutMotionFunc( GlutMouseMove );
  glutDisplayFunc( GlutDrawEnvironment );
  glutIdleFunc( GlutIdleProcessing );
  glutKeyboardFunc( GlutKeyboard );

}


void RenderGL::LoadConfig()
{

  int i;

  BodyTransform = MSLVector(9 * NumberOfBody);
  for(i=0; i<NumberOfBody; i++)
    {
      BodyTransform[9*i+0] = 0.0;  BodyTransform[9*i+1] = 0.0;
      BodyTransform[9*i+2] = 0.0;
      BodyTransform[9*i+3] = 0.0;  BodyTransform[9*i+4] = 0.0;
      BodyTransform[9*i+5] = 0.0;
      BodyTransform[9*i+6] = 1.0;  BodyTransform[9*i+7] = 1.0;
      BodyTransform[9*i+8] = 1.0;
    }
  READ_OPTIONAL_PARAMETER(BodyTransform);

  SetBodyState(BodyTransform);

  EnvTransform = MSLVector(9 * NumberOfEnvObj);
  for(i=0; i<NumberOfEnvObj; i++)
    {
      EnvTransform[9*i+0] = 0.0;  EnvTransform[9*i+1] = 0.0;
      EnvTransform[9*i+2] = 0.0;
      EnvTransform[9*i+3] = 0.0;  EnvTransform[9*i+4] = 0.0;
      EnvTransform[9*i+5] = 0.0;
      EnvTransform[9*i+6] = 1.0;  EnvTransform[9*i+7] = 1.0;
      EnvTransform[9*i+8] = 1.0;
    }
  READ_OPTIONAL_PARAMETER(EnvTransform);

  SetEnvState(EnvTransform);
}


void RenderGL::SetBodyState(const MSLVector& state)
{
  int i;
  MSLVector vpos(3), vori(3), sca(3);

  if(state.dim() - NumberOfBody * 6 == 0)
    {
      for(i=0; i<NumberOfBody; i++)
	{
	  vpos[0] = state[i*6];
	  vpos[1] = state[i*6+1];
	  vpos[2] = state[i*6+2];
	  vori[0] = state[i*6+3];
	  vori[1] = state[i*6+4];
	  vori[2] = state[i*6+5];

	  SceneBodyLib[i]->SetObjectPosition(vpos);
	  SceneBodyLib[i]->SetObjectOrientation(vori);
	}
    }
  else
    if(state.dim() - 9 * NumberOfBody == 0)
      {
	for(i=0; i<NumberOfBody; i++)
	  {
	    vpos[0] = state[i*9];
	    vpos[1] = state[i*9+1];
	    vpos[2] = state[i*9+2];
	    vori[0] = state[i*9+3];
	    vori[1] = state[i*9+4];
	    vori[2] = state[i*9+5];
	    sca[0] = state[i*9+6];
	    sca[1] = state[i*9+7];
	    sca[2] = state[i*9+8];

	    SceneBodyLib[i]->SetObjectPosition(vpos);
	    SceneBodyLib[i]->SetObjectOrientation(vori);
	    SceneBodyLib[i]->SetObjectScale(sca);
	  }
      }
  else
    cout << "BodyEnf File information error" << endl;
}


void RenderGL::SetEnvState(const MSLVector& state)
{
  int i;
  MSLVector vpos(3), vori(3), sca(3);

  if(state.dim() / 9 == NumberOfEnvObj)
    {
      for(i=0; i<NumberOfEnvObj; i++)
	{
	  vpos[0] = state[i*9];
	  vpos[1] = state[i*9+1];
	  vpos[2] = state[i*9+2];
	  vori[0] = state[i*9+3];
	  vori[1] = state[i*9+4];
	  vori[2] = state[i*9+5];
	  sca[0] = state[i*9+6];
	  sca[1] = state[i*9+7];
	  sca[2] = state[i*9+8];

	  SceneEnvObjLib[i]->SetObjectPosition(vpos);
	  SceneEnvObjLib[i]->SetObjectOrientation(vori);
	  SceneEnvObjLib[i]->SetObjectScale(sca);
	}
    }
  else
    cout << "EnvTransform file information error" << endl;
}



void RenderGL::InitData(){
  list<MSLTriangle> trlist;
  list<MSLPolygon> plist;
  list<string>::iterator fname;
  int i;
  mslGLObject * tobj;
  MSLMatrix R(3,3);

  AnimationActive = false;
  CurrentObject = -1;

  CurrentKeyboard = ORI_MOVE_MODE;

  Fov = 45.0;      AspectRatio = 1.0;
  Near = 1.0;     Far = 100000.0;
  VupX = 0.0,      VupY = 1.0,          VupZ = 0.0;

  LightPosX = 1.0; LightPosY = 1.0;     LightPosZ = 3.0;

  ChangeRate = 4.0;

  AnimationTimeScaleTmp = 0.0;

  Orientation[0] = 0.0;   Orientation[1] = 0.0;          Orientation[2] = 0.0;
  Position[0] = 0.0;      Position[1] = 0.0;             Position[2] = 0.0;

  // set the upperworld and lowerworld
  BoundingBoxMin[0] = S->LowerWorld[0];
  BoundingBoxMin[1] = S->LowerWorld[1];
  BoundingBoxMin[2] = S->LowerWorld[2];

  BoundingBoxMax[0] = S->UpperWorld[0];
  BoundingBoxMax[1] = S->UpperWorld[1];
  BoundingBoxMax[2] = S->UpperWorld[2];

  // set the eye position and view reference point accoring to the scene configuration
  EyeX = (BoundingBoxMax[0] + BoundingBoxMin[0])/2.0;
  EyeY = (BoundingBoxMax[1] + BoundingBoxMin[1])/2.0;
  EyeZ = BoundingBoxMax[2] + (BoundingBoxMax[1]-BoundingBoxMin[1])/tan(38.0/180.0*PI);

  VpX = (BoundingBoxMax[0] + BoundingBoxMin[0])/2.0;
  VpY = (BoundingBoxMax[1] + BoundingBoxMin[1])/2.0;
  VpZ = (BoundingBoxMax[2] + BoundingBoxMin[2])/2.0;

  // set the view length
  ViewLength = EyeZ - VpZ;

  // set light position accoring to the viewer position
  LightPosX = EyeX;
  LightPosY = EyeY + (BoundingBoxMax[1] - BoundingBoxMin[1]);
  LightPosZ = BoundingBoxMax[2] + (BoundingBoxMax[1]-BoundingBoxMin[1])/tan(38.0/180.0*PI)/2.0;

  // set the view coordinate system by the viewer parameters
  VCoordX = VCoordY = VCoordZ = MSLVector(3);
  VCoordZ = S->GlobalCameraDirection.norm();
  normalMSLVector(S->GlobalCameraZenith, S->GlobalCameraDirection, VCoordX);
  normalMSLVector(VCoordZ, VCoordX, VCoordY);

  /*
  R(0, 0) = VCoordX[0];   R(1, 0) = VCoordX[1];   R(2, 0) = VCoordX[2];
  R(0, 1) = VCoordY[0];   R(1, 1) = VCoordY[1];   R(2, 1) = VCoordY[2];
  R(0, 2) = VCoordZ[0];   R(1, 2) = VCoordZ[1];   R(2, 2) = VCoordZ[2];

  cout << "viewer orientation matrix: " << R << endl;

  VRpy = irpy(R);

  DefVCoordX = DefVCoordY = DefVCoordZ = MSLVector(3);
  DefVCoordZ[0] = 0.0;  DefVCoordZ[1] = 0.0;  DefVCoordZ[2] = -1.0;
  DefVCoordY[0] = 0.0;  DefVCoordY[1] = 1.0;  DefVCoordY[2] = 0.0;
  DefVCoordX[0] = -1.0;  DefVCoordX[1] = 0.0;  DefVCoordX[2] = 0.0;

  R(0, 0) = DefVCoordX[0];   R(1, 0) = DefVCoordX[1];   R(2, 0) = DefVCoordX[2];
  R(0, 1) = DefVCoordY[0];   R(1, 1) = DefVCoordY[1];   R(2, 1) = DefVCoordY[2];
  R(0, 2) = DefVCoordZ[0];   R(1, 2) = DefVCoordZ[1];   R(2, 2) = DefVCoordZ[2];

  cout << "Default viewer orientation matrix: " << R << endl;

  DefVRpy = irpy(R);

  RpyModification = (VRpy - DefVRpy) * 180 / PI;

  cout << "rpy angle: " << RpyModification << endl;

  SCoordZ[0] = 0.0;  SCoordZ[1] = 0.0;  SCoordZ[2] = 1.0;
  SCoordY[0] = 0.0;  SCoordY[1] = 1.0;  SCoordY[2] = 0.0;
  SCoordX[0] = 1.0;  SCoordX[1] = 0.0;  SCoordX[2] = 0.0;
  */

  // EnvList was initialized by Init in Render base class
  EnvIndex = vector<int>(EnvList.size());

  i = 0;
  forall(fname,EnvList) {

    if (fname->substr(fname->length()-3,3) == "obj")
      {

	cout << "model file name: " << *fname << endl;
	tobj = new mslGLObject;
	tobj->ReadModelFile(FilePath, *fname);
	AddEnvObject(tobj);
	EnvIndex[i] = -1;
      }
    else
      {
	std::ifstream fin((FilePath + *fname).c_str());
	if (S->GeomDim == 2)
	  {
	    fin >> plist;
	    trlist = PolygonsToTriangles(plist,5.0); // Defined in triangle.C
	  }
	else
	  fin >> trlist;
	fin.close();
	EnvIndex[i] = glGenLists(1);
	glNewList(EnvIndex[i], GL_COMPILE);
	InitGeometry(trlist);
	glEndList();
	trlist.clear();
	plist.clear();
      }
    i++;
  }

  // Bodies
  BodyIndex = vector<int>(BodyList.size());
  i = 0;
  forall(fname,BodyList) {
    if (fname->substr(fname->length()-3,3) == "obj")
      {
	cout << "model file name: " << *fname << endl;

	tobj = new mslGLObject;
	tobj->ReadModelFile(FilePath, *fname);
	AddBodyObject(tobj);
	BodyIndex[i] = -1;
      }
    else
      {
	std::ifstream fin2((FilePath + *fname).c_str());
	if (S->GeomDim == 2)
	  {
	    fin2 >> plist;
	    trlist = PolygonsToTriangles(plist,3.0); // Defined in 3Dtriangle.C
	  }
	else
	  fin2 >> trlist;
	fin2.close();
	BodyIndex[i] = glGenLists(1);
	glNewList(BodyIndex[i], GL_COMPILE);
	InitGeometry(trlist);
	glEndList();
	trlist.clear();
      }
    i++;
  }

  LoadConfig();

}



void RenderGL::InitGeometry(list<MSLTriangle> triangles){
  mslGLObject tobj;

  list<MSLTriangle>::iterator t;
  GLfloat d1[3],d2[3],norm[3];
  glPushMatrix();
  glBegin(GL_TRIANGLES);
  forall(t, triangles){
    d1[0] = (GLfloat)t->p2.xcoord() - (GLfloat)t->p1.xcoord();
    d1[1] = (GLfloat)t->p2.ycoord() - (GLfloat)t->p1.ycoord();
    d1[2] = (GLfloat)t->p2.zcoord() - (GLfloat)t->p1.zcoord();

    d2[0] = (GLfloat)t->p3.xcoord() - (GLfloat)t->p1.xcoord();
    d2[1] = (GLfloat)t->p3.ycoord() - (GLfloat)t->p1.ycoord();
    d2[2] = (GLfloat)t->p3.zcoord() - (GLfloat)t->p1.zcoord();

    NormCrossProduct(d1, d2, norm);

    glNormal3fv(norm);

    glVertex3f(t->p1.xcoord(), t->p1.ycoord(), t->p1.zcoord());
    glVertex3f(t->p2.xcoord(), t->p2.ycoord(), t->p2.zcoord());
    glVertex3f(t->p3.xcoord(), t->p3.ycoord(), t->p3.zcoord());
  }
  glEnd();
  glPopMatrix();
}




void RenderGL::DrawBodies(const MSLVector &x)
{

  int i, j;
  MSLVector q(6),mq;
  MSLVector tv(3);

  // Use the forward kinematics to place all body configs in a big MSLVector
  mq = x;

  j = 0;

  for (i = 0; i < S->NumBodies; i++) {
	// Get the configuration
    q[0] = mq[i*6]; q[1] = mq[i*6+1]; q[2] = mq[i*6+2];
    q[3] = mq[i*6+3]; q[4] = mq[i*6+4]; q[5] = mq[i*6+5];

    if(BodyIndex[i] != -1)
      {
	// draw the simple model objects

	// Change the colors for different bodies
	glColor3f(RGBRed[(i+1) % RENDERCOLORS],
		  RGBGreen[(i+1) % RENDERCOLORS],
		  RGBBlue[(i+1) % RENDERCOLORS]);

	// Dump the gl stuff
	glPushMatrix();

	glTranslatef((GLfloat)q[0],(GLfloat)q[1],(GLfloat)q[2]);
	glRotatef((GLfloat)(q[5]*57.286),0,0,1.0);
	glRotatef((GLfloat)(q[4]*57.296),0,1.0,0);
	glRotatef((GLfloat)(q[3]*57.296),1.0,0,0);

	glCallList(BodyIndex[i]);

	glPopMatrix();
      }
    else
      {
	// draw complex 3d model objects

	// if it is in the animation process, continously change the state,
	// otherwise, keep the current state

	if (AnimationActive)
	  {
	    tv[0] = q[0]; tv[1] = q[1]; tv[2] = q[2];
	    SceneBodyLib[j]->SetObjectPosition(tv);
	    tv[0] = q[3]*57.286; tv[1] = q[4]*57.286; tv[2] = q[5]*57.286;
	    SceneBodyLib[j]->SetObjectOrientation(tv);
	  }

	//  comment the object bounding box drawing
	if (BoundingBoxOn)
	  SceneBodyLib[j]->ObjectBoundingBoxDraw();
	SceneBodyLib[j]->ObjectDraw();
	j++;
      }
  }

}


void RenderGL::DrawEnv(){
  unsigned int i, j;

  j = 0;

  for (i = 0; i < EnvIndex.size(); i++)
    {
      if(EnvIndex[i] != -1)
	{
	  glPushMatrix();
	  glColor3f(RGBRed[0],RGBGreen[0],RGBBlue[0]);
	  glCallList(EnvIndex[i]);
	  glPopMatrix();
	}
      else
	{
	  if (BoundingBoxOn)
	    SceneEnvObjLib[j]->ObjectBoundingBoxDraw();
	  SceneEnvObjLib[j]->ObjectDraw();
	  j++;
	}
    }
}


void RenderGL::NormCrossProduct(float v1[3], float v2[3], float out[3])
{
  out[0] = v1[1]*v2[2] - v1[2]*v2[1];
  out[1] = v1[2]*v2[0] - v1[0]*v2[2];
  out[2] = v1[0]*v2[1] - v1[1]*v2[0];

  Normalize(out);
}


void RenderGL::Normalize(float v[3])
{
  GLfloat d = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

  v[0] /=d;
  v[1] /=d;
  v[2] /=d;
}



void RenderGL::MainLoop(Gui *g) {

  g->Finished = false;

  // Store this in the gui pointer G (for callbacks)
  G = g;

  glutMainLoop();
}




void RenderGL::DrawPath()
{
  int i, j;
  int BodyNum;
  MSLVector state1;
  list<MSLVector>::iterator state2;

  glPushMatrix();
  glDisable(GL_TEXTURE_2D);

  GLfloat Diffuse[] = {1.0, 0.0, 0.0};
  GLfloat Ambient[] = {1.0, 0.0, 0.0};
  GLfloat Specular[] = {1.0, 0.0, 0.0};
  GLfloat Shininess[] = {1.0, 0.0, 0.0};

  glDisable(GL_LIGHTING);
  glLineWidth(0.5);

  glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
  glMaterialfv(GL_FRONT, GL_AMBIENT, Ambient);
  glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, Shininess);

  glBegin(GL_LINES);

  state1 = FrameList.front();
  BodyNum = state1.dim() / 6;
  i = 0;
  forall(state2, FrameList)
    {
      if(i != 0 && i<NumFrames)
	{
	  for(j=0; j<BodyNum; j++)
	    {
	      glColor3f(RGBRed[(j+1) % RENDERCOLORS],
			RGBGreen[(j+1) % RENDERCOLORS],
			RGBBlue[(j+1) % RENDERCOLORS]);

	      glVertex3f(state1[6*j], state1[6*j+1], state1[6*j+2]);
	      glVertex3f(state2->operator[](6*j),
			 state2->operator[](6*j+1),
			 state2->operator[](6*j+2));
	    }
	  state1 = *state2;
	}
      i++;
    }

  glEnd();

  glLineWidth(1.0);

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);

  glPopMatrix();
}



void RenderGL::GlutDrawEnvironment() {

  int k, j;
  MSLVector c, conf(6);
  float vsca;
  MSLVector vt1(3);
  MSLVector scenecenter(3);

  scenecenter[0] = (RGL->BoundingBoxMin[0] + RGL->BoundingBoxMax[0])/2.0;
  scenecenter[1] = (RGL->BoundingBoxMin[1] + RGL->BoundingBoxMax[1])/2.0;
  scenecenter[2] = (RGL->BoundingBoxMin[2] + RGL->BoundingBoxMax[2])/2.0;

  RGL->SetLightPos();

  // LaValle added these three lines 1/8/2001
  GLfloat light_ambient[] = {RGL->AmbientLight, RGL->AmbientLight,
			     RGL->AmbientLight, 1.0};
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Advance the frame
  if (RGL->AnimationActive)
    RGL->SetCurrentAnimationFrame();

  if(RGL->AttachedCameraOn)
    {
      // get the current frameinformation
      list<MSLVector>::iterator fi;
      fi = RGL->FrameList.begin();
      for (k = 0; k < RGL->AnimationFrameIndex - 1; k++)
	fi++;
      c = *fi;

      for (j = 0; j < 6; j++)
	conf[j] = c[ 6 * RGL->S->AttachedCameraBody + j];

      // get the camera position and orientation in the scene coordinate
      vt1 = RGL->S->AttachedCameraPosition;
      vt1 = point_x_rotation(conf[3], vt1);
      vt1 = point_y_rotation(conf[4], vt1);
      vt1 = point_z_rotation(conf[5], vt1);

      vt1[0] = vt1[0] + conf[0];
      vt1[1] = vt1[1] + conf[1];
      vt1[2] = vt1[2] + conf[2];

      // get the camera position and orientation in the global coordinate
      vt1[0] = vt1[0] - scenecenter[0];
      vt1[1] = vt1[1] - scenecenter[1];
      vt1[2] = vt1[2] - scenecenter[2];

      vt1 = point_x_rotation(RGL->Orientation[0]*(PI/180.0), vt1);
      vt1 = point_y_rotation(RGL->Orientation[1]*(PI/180.0), vt1);
      vt1 = point_z_rotation(RGL->Orientation[2]*(PI/180.0), vt1);

      vt1[0] = vt1[0] + scenecenter[0];
      vt1[1] = vt1[1] + scenecenter[1];
      vt1[2] = vt1[2] + scenecenter[2];

      vt1[0] = vt1[0] + RGL->Position[0];
      vt1[1] = vt1[1] + RGL->Position[1];
      vt1[2] = vt1[2] + RGL->Position[2];

      RGL->EyeX = vt1[0];
      RGL->EyeY = vt1[1];
      RGL->EyeZ = vt1[2];

      vt1 = RGL->S->AttachedCameraDirection;

      // get the direction in the scene coordinate
      vt1 = point_x_rotation(conf[3], vt1);
      vt1 = point_y_rotation(conf[4], vt1);
      vt1 = point_z_rotation(conf[5], vt1);

      // get the direction in the global coordinate
      vt1 = point_x_rotation(RGL->Orientation[0]*(PI/180.0), vt1);
      vt1 = point_y_rotation(RGL->Orientation[1]*(PI/180.0), vt1);
      vt1 = point_z_rotation(RGL->Orientation[2]*(PI/180.0), vt1);

      vsca = RGL->ViewLength / vt1.length();

      RGL->VpX = RGL->EyeX + vt1[0]*vsca;
      RGL->VpY = RGL->EyeY + vt1[1]*vsca;
      RGL->VpZ = RGL->EyeZ + vt1[2]*vsca;

      vt1 = RGL->S->AttachedCameraZenith;
      // get the zenith in the scene coordinate
      vt1 = point_x_rotation(conf[3], vt1);
      vt1 = point_y_rotation(conf[4], vt1);
      vt1 = point_z_rotation(conf[5], vt1);

      // get the direction in the global coordinate
      vt1 = point_x_rotation(RGL->Orientation[0]*(PI/180.0), vt1);
      vt1 = point_y_rotation(RGL->Orientation[1]*(PI/180.0), vt1);
      vt1 = point_z_rotation(RGL->Orientation[2]*(PI/180.0), vt1);

      RGL->VupX = vt1[0];
      RGL->VupY = vt1[1];
      RGL->VupZ = vt1[2];

      RGL->Fov = 70.0;
    }
  else
    {
      vsca = RGL->ViewLength / RGL->S->GlobalCameraDirection.length();

      RGL->EyeX = RGL->S->GlobalCameraPosition[0];
      RGL->EyeY = RGL->S->GlobalCameraPosition[1];
      RGL->EyeZ = RGL->S->GlobalCameraPosition[2];

      RGL->VpX = RGL->EyeX + RGL->S->GlobalCameraDirection[0] * vsca;
      RGL->VpY = RGL->EyeY + RGL->S->GlobalCameraDirection[1] * vsca;
      RGL->VpZ = RGL->EyeZ + RGL->S->GlobalCameraDirection[2] * vsca;

      RGL->VupX = RGL->S->GlobalCameraZenith[0];
      RGL->VupY = RGL->S->GlobalCameraZenith[1];
      RGL->VupZ = RGL->S->GlobalCameraZenith[2];

      RGL->Fov = 45.0;
    }

  if (!RGL->MultipleViewsOn)
    {
      glViewport((GLint)0.0, (GLint)0.0, (GLsizei)RGL->WindowX, (GLsizei)RGL->WindowY);
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      gluPerspective(RGL->Fov,(GLfloat)RGL->WindowX / (GLfloat)RGL->WindowY,
		     RGL->Near, RGL->Far);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();

      gluLookAt(RGL->EyeX, RGL->EyeY, RGL->EyeZ, RGL->VpX,
		RGL->VpY, RGL->VpZ, RGL->VupX, RGL->VupY, RGL->VupZ);

      RGL->SceneRender();
    }
  else
    {

      glViewport((GLint)0.0, (GLint)0.0, (GLsizei)RGL->WindowX/2, (GLsizei)RGL->WindowY/2);
      glMatrixMode( GL_PROJECTION );
      glLoadIdentity();
      glOrtho(RGL->BoundingBoxMin[2]*1.2-scenecenter[2]*1.2,
	      RGL->BoundingBoxMax[2]*1.2-scenecenter[2]*1.2,
	      RGL->BoundingBoxMin[1]*1.2-scenecenter[1]*1.2,
	      RGL->BoundingBoxMax[1]*1.2-scenecenter[1]*1.2, 0.1, 100000.0);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(scenecenter[0]-1000.0, scenecenter[1], scenecenter[2],
		scenecenter[0], scenecenter[1], scenecenter[2], 0.0, 1.0, 0.0);

      RGL->SceneRender();

      glViewport((GLint)RGL->WindowX/2, (GLint)RGL->WindowY/2,
		 (GLsizei)RGL->WindowX/2, (GLsizei)RGL->WindowY/2);
      glMatrixMode( GL_PROJECTION );
      glLoadIdentity();
      glOrtho(RGL->BoundingBoxMin[0]*1.2-scenecenter[0]*1.2,
	      RGL->BoundingBoxMax[0]*1.2-scenecenter[0]*1.2,
	      RGL->BoundingBoxMin[2]*1.2-scenecenter[2]*1.2,
	      RGL->BoundingBoxMax[2]*1.2-scenecenter[2]*1.2, 0.1, 100000.0);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(scenecenter[0], scenecenter[1]+1000.0, scenecenter[2],
		scenecenter[0], scenecenter[1], scenecenter[2], 0.0, 0.0, -1.0);

      RGL->SceneRender();

      glViewport((GLint)0.0, (GLint)RGL->WindowY/2,
		 (GLsizei)RGL->WindowX/2, (GLsizei)RGL->WindowY/2);
      glMatrixMode( GL_PROJECTION );
      glLoadIdentity();
      glOrtho(RGL->BoundingBoxMin[0]*1.2-scenecenter[0]*1.2,
	      RGL->BoundingBoxMax[0]*1.2-scenecenter[0]*1.2,
	      RGL->BoundingBoxMin[1]*1.2-scenecenter[1]*1.2,
	      RGL->BoundingBoxMax[1]*1.2-scenecenter[1]*1.2, 0.1, 100000.0);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(scenecenter[0], scenecenter[1], scenecenter[2]+1000.0,
		scenecenter[0], scenecenter[1], scenecenter[2], 0.0, 1.0, 0.0);

      RGL->SceneRender();

      glViewport((GLint)RGL->WindowX/2, (GLint)0.0,
		 (GLsizei)RGL->WindowX/2, (GLsizei)RGL->WindowY/2);
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      gluPerspective(RGL->Fov,(GLfloat)RGL->WindowX / (GLfloat)RGL->WindowY,
		     RGL->Near, RGL->Far);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(RGL->EyeX, RGL->EyeY, RGL->EyeZ, RGL->VpX,
		RGL->VpY, RGL->VpZ, RGL->VupX, RGL->VupY, RGL->VupZ);

      RGL->SceneRender();

    }

  glutSwapBuffers();
}


void RenderGL::SceneRender()
{
  MSLVector scenecenter(3);

  glPushMatrix();

  scenecenter[0] = (BoundingBoxMin[0] + BoundingBoxMax[0])/2.0;
  scenecenter[1] = (BoundingBoxMin[1] + BoundingBoxMax[1])/2.0;
  scenecenter[2] = (BoundingBoxMin[2] + BoundingBoxMax[2])/2.0;

  // set the position and orientation of the scene
  glTranslatef(Position[0], Position[1], Position[2]);

  // rotate around the position the viewer looks at
  glTranslatef(scenecenter[0], scenecenter[1], scenecenter[2]);
  glRotatef(Orientation[2], 0.0, 0.0, 1.0);
  glRotatef(Orientation[1], 0.0, 1.0, 0.0);
  glRotatef(Orientation[0], 1.0, 0.0, 0.0);
  glTranslatef(-scenecenter[0], -scenecenter[1], -scenecenter[2]);

  // draw the coordinate
  //ShowCoordinateFrame();

  // Draw environment
  DrawEnv();

  DrawBodies(CurrentAnimationFrame);

  if (ShowPathOn)
    DrawPath();

  if (CurrentObject != -1)
    WhichObject(CurrentObject)->ObjectHighlight();

  if (BoundingBoxOn)
    DrawBoundingBox();

  glPopMatrix();

}


void RenderGL::SetLightPos()
{
  GLfloat lightpos[4];

  lightpos[0] = LightPosX;
  lightpos[1] = LightPosY;
  lightpos[2] = LightPosZ;
  lightpos[3] = 1.0;

  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
}


void RenderGL::SetSceneOrientationChange(const MSLVector& oric)
{

  Orientation[0] = Orientation[0] + oric[0];
  Orientation[1] = Orientation[1] + oric[1];
  Orientation[2] = Orientation[2] + oric[2];

  for(int i=0; i<3; i++)
    {
      while(Orientation[i]>=2*PI) Orientation[i] = Orientation[i] - 360.0;
      while(Orientation[i]<0) Orientation[i] = Orientation[i] + 360.0;
    }
}




void RenderGL::SetScenePositionChange(const MSLVector& posc)
{

  Position[0] = Position[0] + posc[0];
  Position[1] = Position[1] + posc[1];
  Position[2] = Position[2] + posc[2];
}



void RenderGL::DrawBoundingBox()
{

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  GLfloat Diffuse[] = {1.0, 0.0, 0.0};
  GLfloat Ambient[] = {1.0, 0.0, 0.0};
  GLfloat Specular[] = {1.0, 0.0, 0.0};
  GLfloat Shininess[] = {1.0, 0.0, 0.0};

  glPushMatrix();

  glLineWidth(1.0);

  glColor4f(0.41, 0.55, 0.137, 1.0);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
  glMaterialfv(GL_FRONT, GL_AMBIENT, Ambient);
  glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, Shininess);

  glBegin(GL_LINES);

  glNormal3f(-1.0, -1.0, 1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  glNormal3f(1.0, -1.0, 1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMin[2]);

  glNormal3f(-1.0, -1.0, 1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  glNormal3f(-1.0, 1.0, 1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMin[2]);

  glNormal3f(1.0, 1.0, 1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  glNormal3f(-1.0, 1.0, 1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMin[2]);

  glNormal3f(1.0, 1.0, 1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  glNormal3f(1.0, -1.0, 1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMin[2]);



  glNormal3f(1.0, 1.0, -1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  glNormal3f(-1.0, 1.0, -1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMax[2]);

  glNormal3f(1.0, 1.0, -1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  glNormal3f(1.0, -1.0, -1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMax[2]);

  glNormal3f(-1.0, -1.0, -1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  glNormal3f(1.0, -1.0, -1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMax[2]);

  glNormal3f(-1.0, -1.0, -1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  glNormal3f(-1.0, 1.0, -1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMax[2]);



  glNormal3f(1.0, -1.0, 1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  glNormal3f(1.0, -1.0, -1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMax[2]);

  glNormal3f(1.0, 1.0, -1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  glNormal3f(1.0, 1.0, 1.0);
  glVertex3f(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMin[2]);

  glNormal3f(-1.0, 1.0, 1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  glNormal3f(-1.0, 1.0, -1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMax[2]);

  glNormal3f(-1.0, -1.0, 1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  glNormal3f(-1.0, -1.0, 1.0);
  glVertex3f(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMax[2]);

  glEnd();

  glPopMatrix();

  glLineWidth(1.0);

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
}



void RenderGL::GlutReshape(int w, int h)
{
  RGL->WindowX = w;
  RGL->WindowY = h;
  RGL->WindowZ = ((float) w + (float) h)/2.0;

  glViewport(0,0,(GLsizei) RGL->WindowX, (GLsizei) RGL->WindowY);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(RGL->Fov,(GLfloat) RGL->WindowX / (GLfloat) RGL->WindowY, RGL->Near, RGL->Far);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(RGL->EyeX, RGL->EyeY, RGL->EyeZ, RGL->VpX,
	    RGL->VpY, RGL->VpZ, RGL->VupX, RGL->VupY, RGL->VupZ);
}


void RenderGL::GlutMouse(int button, int state, int x, int y)
{
  float x1, y1;

  x1 = ((float)x-RGL->WindowX/2.0)*(MaxX-MinX)/RGL->WindowX;
  y1 = ((float)y-RGL->WindowY/2.0)*(MaxY-MinY)/RGL->WindowY*-1.0;

  RGL->LastX = x1;
  RGL->LastY = y1;

  RGL->CurrentMouseButton = button;
  RGL->CurrentMouseState = state;
}


void RenderGL::GlutMouseMove( int x, int y )
{
  int i;
  float x1, y1;
  float delta_x, delta_y;
  MSLVector oric(3), posc(3);
  MSLVector sori(3);
  MSLMatrix mt;

  oric[0] = 0.0;  oric[1] = 0.0;  oric[2] = 0.0;
  posc[0] = 0.0;  posc[1] = 0.0;  posc[2] = 0.0;

  x1 = ((float)x-RGL->WindowX/2.0)*(MaxX-MinX)/RGL->WindowX;
  y1 = ((float)y-RGL->WindowY/2.0)*(MaxY-MinY)/RGL->WindowY*-1.0;

  delta_x = (x1 - RGL->LastX);
  delta_y = (y1 - RGL->LastY);

  if(RGL->CurrentKeyboard == ORI_MOVE_MODE)
    {
      // change the position and orientation of the whole scene

      // change the orientation around y and z axis
      if(RGL->CurrentMouseButton==GLUT_LEFT_BUTTON && RGL->CurrentMouseState==GLUT_DOWN)
	{

	  sori[0] = RGL->Orientation[0];
	  sori[1] = RGL->Orientation[1];
	  sori[2] = RGL->Orientation[2];

	  mt = free_rotate(RGL->VCoordY, rpy(sori*(PI/180.0)),
			   delta_x / (MaxX - MinX) * PI / 5.0);


	  mt = free_rotate(RGL->VCoordX, mt,
			   delta_y / (MaxY - MinY) * PI / 5.0);

	  oric = irpy(mt)*180.0/PI - sori;
	  RGL->SetSceneOrientationChange(oric);

	}

      // change the position along the x and y axis
      if(RGL->CurrentMouseButton==GLUT_MIDDLE_BUTTON && RGL->CurrentMouseState==GLUT_DOWN)
	{
	  posc = -delta_x * RGL->ChangeRate * RGL->VCoordX +
	    delta_y * RGL->ChangeRate * RGL->VCoordY;

	  RGL->SetScenePositionChange(posc*0.1);
	}

      // change the orientation around x axis and position along the z axis
      if(RGL->CurrentMouseButton==GLUT_RIGHT_BUTTON && RGL->CurrentMouseState==GLUT_DOWN)
	{
	  sori[0] = RGL->Orientation[0];
	  sori[1] = RGL->Orientation[1];
	  sori[2] = RGL->Orientation[2];

	  mt = free_rotate(RGL->VCoordZ, rpy(sori*(PI/180.0)),
			   delta_x / (MaxX - MinX) * PI / 5.0);

	  oric = irpy(mt)*180.0/PI - sori;
	  RGL->SetSceneOrientationChange(oric);

	  posc = 5.0 * delta_y / (MaxY - MinY) * PI * RGL->ChangeRate * RGL->VCoordZ;
	  RGL->SetScenePositionChange(posc);
	}
    }
  else
    {
      // change the scale, orientation and position of the current object

      // change the orientation around y and z axis
      if(RGL->CurrentMouseButton==GLUT_LEFT_BUTTON && RGL->CurrentMouseState==GLUT_DOWN)
	{
	  if(RGL->CurrentObject!=-1)
	    {
	      oric[0] = 0.0;
	      oric[1] = delta_x / (MaxX - MinX) * PI * RGL->ChangeRate;
	      oric[2] = delta_y / (MaxY - MinY) * PI * RGL->ChangeRate;

	      RGL->WhichObject(RGL->CurrentObject)->SetBodyOrientationChange(oric);
	    }
	}

      // change the position along x and y
      if(RGL->CurrentMouseButton==GLUT_MIDDLE_BUTTON && RGL->CurrentMouseState==GLUT_DOWN)
	{
	  if(RGL->CurrentObject!=-1)
	    {
	      RGL->WhichObject(RGL->CurrentObject)->Position[0] =
		RGL->WhichObject(RGL->CurrentObject)->Position[0] + delta_x * RGL->ChangeRate;
	      RGL->WhichObject(RGL->CurrentObject)->Position[1] =
		RGL->WhichObject(RGL->CurrentObject)->Position[1] + delta_y * RGL->ChangeRate;
	    }
	}

      // change the orientation around x or scale of the object or position along z
      if(RGL->CurrentMouseButton==GLUT_RIGHT_BUTTON && RGL->CurrentMouseState==GLUT_DOWN)
	{
	  if(RGL->CurrentObject!=-1)
	    {
	      oric[0] = delta_x / (MaxX - MinX) * PI * RGL->ChangeRate;
	      oric[1] = 0.0;             oric[2] = 0.0;
	      RGL->WhichObject(RGL->CurrentObject)->SetBodyOrientationChange(oric);

	      if(RGL->CurrentKeyboard == SCALE_MOVE_MODE)
		{
		  for(i=0; i<3; i++)
		    {
		      RGL->WhichObject(RGL->CurrentObject)->Scale[i] =
			RGL->WhichObject(RGL->CurrentObject)->Scale[i] *
			(1+delta_y/(MaxY - MinY));
		    }
		}
	      else
		{
		  RGL->WhichObject(RGL->CurrentObject)->Position[2] =
		    RGL->WhichObject(RGL->CurrentObject)->Position[2] + delta_y * RGL->ChangeRate;
		}
	    }
	}
    }

  RGL->LastX = x1;
  RGL->LastY = y1;

  glutPostRedisplay();
}



void RenderGL::GlutIdleProcessing()
{
  int i;

  // Handle the window events for the Gui
  for (i = 0; i < 20; i++) // This is a hack!!!
    RGL->G->HandleEvents();

  RGL->HandleEvents();

  // Allow exiting by pressing Exit button in Gui
  if (RGL->G->Finished)
    exit(-1);

  if ( glutGetWindow() != RGL->MainWindow )
    glutSetWindow(RGL->MainWindow);

  glutPostRedisplay();

}


void RenderGL::GlutKeyboard(unsigned char Key, int x, int y)
{
  switch(Key) {
  case 'a':
    RGL->ButtonHandle(74);
    break;
  case 'b':
    RGL->BoundingBoxOn = !(RGL->BoundingBoxOn);
    break;
  case 'd':
    RGL->ButtonHandle(73);
    break;
  case 'h':
    cout << endl;
    cout << "HELP FOR RENDERGL KEYBOARD CONTROL:" << endl;
    cout << "'a': Accelerate Animation Rate" << endl;
    cout << "'b': Show and Hide the Bounding Box" << endl;
    cout << "'d': Decelerate Animation Rate" << endl;
    cout << "'h': Show the Keyboard Control Help" << endl;
    cout << "'m': Change to Multiview Mode" << endl;
    cout << "'p': Pause or Continue the Animation" << endl;
    cout << "'q': Quit" << endl;
    cout << "'r': Reset the Rendering Options\n" << endl;
    break;
  case 'm':
    RGL->MultipleViewsOn = !(RGL->MultipleViewsOn);
    break;
  case 'p':
    RGL->ButtonHandle(72);
    break;
  case 'q':
    exit(0);
    break;
  case 'r':
    RGL->ButtonHandle(75);
    break;
  case 'v':
    RGL->AttachedCameraOn = !(RGL->AttachedCameraOn);
    break;
  };

  glutPostRedisplay();
}
