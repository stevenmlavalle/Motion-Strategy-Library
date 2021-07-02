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

// This file was written by Peng Cheng (chp@cs.iastate.edu)

#ifndef MSL_RENDERGLOBJ_H
#define MSL_RENDERGLOBJ_H

#include <stdio.h>
#include <ctype.h>
//#include <algorithm>
#include <stdlib.h>
#include <math.h>

#include <string>
//#include <algorithm>
using namespace std;
//  #include <unistd.h>     // needed to sleep.

#include "triangle.h"
#include "vector.h"
#include "matrix.h"
#include "point3d.h"
#include "util.h"

#include <GL/glut.h>
#include <GL/gl.h>	// Header File For The OpenGL32 Library
#include <GL/glu.h>	// Header File For The GLu32 Library

#define	BUFFER_SIZE	4096
#define	FACE_SIZE	2048
#define	CHUNK		2048
#define	MAX_MTL_FILES	512
#define MAXNAME_LENGTH  50

// !!!!!!!!!!!!!!!!!!!!!!!!!!! -- originally from chpdef.h -- 1/5/01
#define MinX                            -100.0
#define MinY                            -100.0 
#define MaxX                             100.0 
#define MaxY                             100.0
#define MinZ                             100.0
#define MaxZ                             300.0
#define RESET_ID                         200
#define RADIO_GROUP1                     201
#define RECORD_PATH_ID                   202
#define DISPLAY_PATH_ID                  203
#define NEW_PATH_ID                      204
#define VIEW_ORIENTATION_ID              205
#define BODY_SELECTION_ID                206
#define TEXTURE_ON_OFF_ID                207
#define SINGLE_MULTI_VIEW_ID             208
#define RELOAD_MODEL_ID                  209
#define TRANSPARENCY_ADJUST_ID           210
#define BEGIN_FLIGHT_ID                  211
#define PLANE_LAND_ID                    212
#define MODEL_FLY_ID                     213
#define CREATE_OBJECT_ID                 214
#define DELETE_OBJECT_ID                 215
#define CURRENT_OBJECT_ID                216
#define CREATE_OBJECT_OVER_ID            217
#define CREATE_POINT_ID                  218
#define CREATE_POINT_OVER_ID             219
#define CURRENT_TEXTURE_ID               220 
#define OBJECT_SELECTION_ID              221
#define PLAN_ID                          222
#define ANIMATE_ID                       223
#define SET_INITIAL_ID                   224
#define SAVE_GEOMETRY_ID                 225
#define SET_GOAL_ID                      226
#define ANIMATE_PAUSE_ID                 227
#define SAVE_CONFIG_ID                   228
#define AUTOMATIC_PLAN_ID                229
#define LIGHT_COMPONENT_SELECTION_ID     230
#define LIGHT_MODE_SELECTION_ID          231
#define ANIMATION_RATE_ID                232
#define LOAD_CONFIG_ID                   233
#define SINGLE_VIEW                      300
#define MULTI_VIEW                       301
#define OBSTACLE_MODE                    302
#define ROBOT_MODE                       303
#define Z_MOVE_MODE                      304
#define SCALE_MOVE_MODE                  305
#define ORI_MOVE_MODE                    306
#define SELECT_BOUNDINGBOX_ID            307
#define SHOW_BOUNDINGBOX_ID              308
#define HIDE_BOUNDINGBOX_ID              309
#define SUN_LIGHT_MODE                   310
#define EYE_LIGHT_MODE                   311 
//  !!!!!!!!!!!!!!!!!!!!!!!! -- originally from chpdef.h -- 1/5/01


//  !!!!!!!!!!!!!!!!!!!!!!!! -- come from chpMSLMatrix.h -- 1/5/01

// rotate MSLMatrix
void rotate_x_MSLMatrix(double sita, MSLMatrix& m);
void rotate_y_MSLMatrix(double sita, MSLMatrix& m);
void rotate_z_MSLMatrix(double sita, MSLMatrix& m);
// point rotation
MSLVector point_x_rotation(const double& sita, const MSLVector& p1);
MSLVector point_y_rotation(const double& sita, const MSLVector& p1);
MSLVector point_z_rotation(const double& sita, const MSLVector& p1);
// cross product of MSLVector v1 and MSLVector v2, v1 X v2 = v
void crossproduct(const MSLVector& v1, const MSLVector& v2, MSLVector& v);
// v = normalize the v1 X v2
void normalMSLVector(const MSLVector& v1, const MSLVector& v2, MSLVector& v);
// caculate the rpy angle from the orientation MSLMatrix
MSLVector irpy(const MSLMatrix& R);
// caculate the MSLMatrix corresponding to the rpy rotation
MSLMatrix rpy(const MSLVector& v);
// MSLVector v rotate around any unit MSLVector axis which orginates from the origin
MSLVector free_rotate(const MSLVector& axis, const MSLVector& v, double sita);
// MSLMatrix m rotate around any unit MSLVector axis which orginates from the origin
MSLMatrix free_rotate(const MSLVector& axis, const MSLMatrix& m, double sita);




//int strcasecmp __P ((__const char *__s1, __const char *__s2));

/* case insensitive string equality test */
#define UPPER(_str) transform(_str.begin(), _str.end(), _str.begin(), ::toupper)
#define	SAME(_a, _b)	(UPPER(string(_a)) == UPPER(string(_b)))

typedef struct{
  double x;
  double y;
  double z;
} mslGLVertex;

typedef struct{
  double x;
  double y;
  double z;
} mslGLNormal;

typedef struct{
  double x;
  double y;
  double z;
} mslGLTexCoord;


/* Image type - contains height, width, and data */
//! Used for texture mapping as part of RenderGL
class Image {
public:
    unsigned long sizeX;
    unsigned long sizeY;
    char *data;

    Image();
    ~Image();
};

//! An internal class, used only for RenderGL
class mslGLMaterial
{
public:
  int ID;
  char Name[MAXNAME_LENGTH];

  GLuint TextureHandle;
  Image * TextureImage;
  char  TextureName[MAXNAME_LENGTH];

  GLfloat Diffuse[3];
  GLfloat Specular[3];
  GLfloat Ambient[3];
  GLfloat Color[3];

  GLfloat Alpha;
  float Shininess;

  float Su;
  float Sv;

  char Reflect[MAXNAME_LENGTH];

  int AmbientOn;
  int SpecularOn;
  int DiffuseOn;
  int ShininessOn;
  int AlphaOn;
  int ReflectOn;
  int TwosideOn;
  int TextureOn;

  mslGLMaterial();
  ~mslGLMaterial();

  void SetMaterial();
  int ImageLoad(int id, string path, string filename);
  void Clear();

  void Print();
};


//! An internal class, used only for RenderGL
class mslGLFace
{
 public:
  int NumberOfPoint;
  int NumberOfNormal;
  int NumberOfTexCoord;

  mslGLVertex * VerticeCoord;
  mslGLNormal * NormalCoord;
  mslGLTexCoord * TextureCoord;

  int NormalOn;
  int TextureOn;
  int ColorOn;

  int MaterialID;

  mslGLFace();
  ~mslGLFace();

  void AddVertex(const mslGLVertex& ver);
  void AddNormal(const mslGLNormal& nor);
  void AddTexCoord(const mslGLTexCoord& tex);

  void AddVertex(const MSLVector& ver);
  void AddNormal(const MSLVector& nor);
  void AddTexCoord(const MSLVector& tex);

  void Clear();

  void PrintVertex();

  void DrawFace();
};



//! An internal class, used only for RenderGL
class mslGLObject
{
 public:

  int ID;
  string Name;

  int NumberOfMaterial;
  mslGLMaterial * ObjectMaterialLib;

  int NumberOfFace;
  mslGLFace * ObjectFaceLib; 
  
  float Position[3], Orientation[3];
  float Scale[3];

  float BoundingBoxMin[3];
  float BoundingBoxMax[3];

  //mslGLObject(string path);
  mslGLObject();
  ~mslGLObject();

  int ReadModelFile(const string& path, const string& filename);

  MSLPoint3d PointCurrentState(const MSLPoint3d& po, int mode);

  list<MSLTriangle> SetBoundingBoxTriangle(int mode);

  void LoadMaterialFile(const string& path, const string& name);
  int  SetCurrentMaterialID(char * name);
  void SetMaterial(int matid);
  void AddMaterial(const string& path, const mslGLMaterial& mat);
  void AddFace(const mslGLFace& face);
  void ParseTexture(char * next, mslGLMaterial * mat);
  void Clear();

  void ObjectDraw();
  void ObjectBoundingBoxDraw();
  void ObjectHighlight();

  void SetObjectPosition(const MSLVector& pos);
  void SetObjectOrientation(const MSLVector& ori);
  void SetObjectScale(const MSLVector& sca);

  void SetBodyPositionChange(const MSLVector& posc);
  void SetBodyOrientationChange(const MSLVector& oric);
  void SetBodyScaleChange(const MSLVector& scac);

  void PrintFace();
  void PrintMaterial();
  void PrintState();
};


//  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -- come from generalfunction.h -- 1/5/01
mslGLNormal NormalCompute(const mslGLVertex& v1, const mslGLVertex& v2, 
			  const mslGLVertex& v3);
//  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -- come from generalfunction.h -- 1/5/01


#endif

