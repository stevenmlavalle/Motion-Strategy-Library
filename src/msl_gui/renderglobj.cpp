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

#include <cstring>
#include <algorithm>

#include "msl/defs.h"

#include "msl_gui/renderglobj.h"

//****!!!!!!!!!!!!!!!!!!!!!!!!!!!! -- came from matrix.C -- 1/5/01
void rotate_x_matrix(double sita, MSLMatrix& m)
{
  m(0, 0)=1.0;  m(0,1)=0.0;   m(0,2)=0.0;    m(0,3)=0.0;
  m(1,0)=0.0;  m(1,1)=cos(sita);   m(1,2)=-sin(sita);    m(1,3)=0.0;
  m(2,0)=0.0;  m(2,1)=sin(sita);   m(2,2)=cos(sita);    m(2,3)=0.0;
  m(3,0)=0.0;  m(3,1)=0.0;   m(3,2)=0.0;    m(3,3)=1.0;
}

void rotate_y_matrix(double sita, MSLMatrix& m)
{
  m(0,0)=cos(sita);  m(0,1)=0.0;   m(0,2)=sin(sita);    m(0,3)=0.0;
  m(1,0)=0.0;  m(1,1)=1.0;   m(1,2)=0.0;    m(1,3)=0.0;
  m(2,0)=-sin(sita);  m(2,1)=0.0;   m(2,2)=cos(sita);    m(2,3)=0.0;
  m(3,0)=0.0;  m(3,1)=0.0;   m(3,2)=0.0;    m(3,3)=1.0;
}

void rotate_z_matrix(double sita, MSLMatrix& m)
{
  m(0,0)=cos(sita);  m(0,1)=-sin(sita);   m(0,2)=0.0;    m(0,3)=0.0;
  m(1,0)=sin(sita);  m(1,1)=cos(sita);   m(1,2)=0.0;    m(1,3)=0.0;
  m(2,0)=0.0;  m(2,1)=0.0;   m(2,2)=1.0;    m(2,3)=0.0;
  m(3,0)=0.0;  m(3,1)=0.0;   m(3,2)=0.0;    m(3,3)=1.0;
}

MSLVector point_x_rotation(const double& sita, const MSLVector& p1)
{
  MSLVector p2(3), p21(4), p11(4);
  MSLMatrix m(4,4);

  rotate_x_matrix(sita, m);
  p11[0] = p1[0];
  p11[1] = p1[1];
  p11[2] = p1[2];
  p11[3] = 1.0;

  p21 = m * p11;

  p2[0] = p21[0];
  p2[1] = p21[1];
  p2[2] = p21[2];

  return p2;
}

MSLVector point_y_rotation(const double& sita, const MSLVector& p1)
{
  MSLVector p2(3), p21(4), p11(4);
  MSLMatrix m(4,4);

  rotate_y_matrix(sita, m);
  p11[0] = p1[0];
  p11[1] = p1[1];
  p11[2] = p1[2];
  p11[3] = 1.0;

  p21 = m * p11;

  p2[0] = p21[0];
  p2[1] = p21[1];
  p2[2] = p21[2];

  return p2;
}

MSLVector point_z_rotation(const double& sita, const MSLVector& p1)
{
  MSLVector p2(3), p21(4), p11(4);
  MSLMatrix m(4,4);

  rotate_z_matrix(sita, m);
  p11[0] = p1[0];
  p11[1] = p1[1];
  p11[2] = p1[2];
  p11[3] = 1.0;

  p21 = m * p11;

  p2[0] = p21[0];
  p2[1] = p21[1];
  p2[2] = p21[2];

  return p2;
}



void crossproduct(const MSLVector& v1, const MSLVector& v2, MSLVector& v)
{
  v[0] = v1[1]*v2[2] - v1[2]*v2[1];
  v[1] = v1[2]*v2[0] - v1[0]*v2[2];
  v[2] = v1[0]*v2[1] - v1[1]*v2[0];
}


void normalMSLVector(const MSLVector& v1, const MSLVector& v2, MSLVector& v)
{
  crossproduct(v1, v2, v);
  v = v.norm();
}

MSLVector irpy(const MSLMatrix& R)
{
  MSLVector k(3);

   if (R(2,0) == 1) {
     k[0] = atan2(-R(0, 1), -R(0, 2));
     k[1] = -PI/2.0;
     k[2] = 0.0;
   } else if (R(2,0) == -1) {
     k[0] = atan2(R(0,1), R(0,2));
     k[1] = PI/2.0;
     k[2] = 0.0;
   } else {
     k[0] = atan2(R(2,1), R(2,2));
     k[1] = atan2(-R(2,0), sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
     k[2] = atan2(R(1,0), R(0,0));
   }

   return k;
}

MSLMatrix rpy(const MSLVector& a)
{

   MSLMatrix rot(4,4);
   double ca, sa, cb, sb, cc, sc;

   ca = cos(a[0]);
   sa = sin(a[0]);
   cb = cos(a[1]);
   sb = sin(a[1]);
   cc = cos(a[2]);
   sc = sin(a[2]);

   rot(0,0) = cb*cc;
   rot(0,1) = sa*sb*cc-ca*sc;
   rot(0,2) = ca*sb*cc+sa*sc;
   rot(1,0) = cb*sc;
   rot(1,1) = sa*sb*sc+ca*cc;
   rot(1,2) = ca*sb*sc-sa*cc;
   rot(2,0) = -sb;
   rot(2,1) = sa*cb;
   rot(2,2) = ca*cb;

   rot(0, 3) = rot(1, 3) = rot(2, 3) = rot(3, 0) = rot(3, 1) = rot(3, 2) = 0.0;
   rot(3, 3) = 1.0;

   return rot;

}

MSLVector free_rotate(const MSLVector& axis, const MSLVector& v, double sita)
{
  double rx, ry;
  double diagnal_xy;
  MSLVector vt(3);

  // get the rotation angle rx around x to make the axis to be in zx plane
  rx = atan2(axis[1], axis[2]);

  //!!!!!!!!!!!!!!! -- there might be some problem
  // because diagnal_xy might be negative
  diagnal_xy = sqrt(axis[1]*axis[1]+axis[2]*axis[2]);

  // get the rotate angle ry around y to make the axis to be withe z axis
  ry = atan2(axis[0], diagnal_xy);

  vt = v;

  vt = point_x_rotation(rx, vt);
  vt = point_y_rotation(-ry, vt);

  vt = point_z_rotation(sita, vt);

  vt = point_y_rotation(ry, vt);
  vt = point_x_rotation(-rx, vt);

  return vt;
}


MSLMatrix free_rotate(const MSLVector& axis, const MSLMatrix& m, double sita)
{
  double rx, ry;
  double diagnal_xy;
  MSLMatrix mt(4, 4), mr(4, 4);

  // get the rotation angle rx around x to make the axis to be in zx plane
  rx = atan2(axis[1], axis[2]);

  //!!!!!!!!!!!!!!! -- there might be some problem
  // because diagnal_xy might be negative
  diagnal_xy = sqrt(axis[1]*axis[1]+axis[2]*axis[2]);

  // get the rotate angle ry around y to make the axis to be withe z axis
  ry = atan2(axis[0], diagnal_xy);

  mt = m;

  rotate_x_matrix(rx, mr);
  mt = mr * mt;

  rotate_y_matrix(-ry, mr);
  mt = mr * mt;

  rotate_z_matrix(sita, mr);
  mt = mr * mt;

  rotate_y_matrix(ry, mr);
  mt = mr * mt;

  rotate_x_matrix(-rx, mr);
  mt = mr * mt;

  return mt;
}


//**!!!!!!!!!!!!!!!!!!!!!!!!!!! -- come from matrix.C -- 1/5/01


static  list<string> MaterialFileList;
static  int 	     numSkip	= 0;
static  int 	     numOther	= 0;

#define	GROW(_v, _t) \
    if (_v == NULL) \
    { \
	_v ## Available = CHUNK; \
	_v = (_t *) malloc(sizeof(_t)*_v ## Available); \
    } \
    else \
    if (_v ## Count >= _v ## Available) \
    { \
	_v ## Available *= 2; \
	_v = (_t *) realloc(_v, sizeof(_t)*_v ## Available); \
    }


static unsigned int getint(FILE *fp)
{
  int c, c1, c2, c3;

  /* get 4 bytes*/
  c = getc(fp);
  c1 = getc(fp);
  c2 = getc(fp);
  c3 = getc(fp);

  return ((unsigned int) c) +
    (((unsigned int) c1) << 8) +
    (((unsigned int) c2) << 16) +
    (((unsigned int) c3) << 24);
}

static unsigned int getshort(FILE *fp)
{
  int c, c1;

  /*get 2 bytes*/
  c = getc(fp);
  c1 = getc(fp);

  return ((unsigned int) c) + (((unsigned int) c1) << 8);
}


Image::Image()
{
  sizeX = (unsigned long) 256;
  sizeY = (unsigned long) 256;

  data = new char[10];
}

Image::~Image()
{
  //  if(data != NULL)
  //    free(data);
}


mslGLMaterial::mslGLMaterial()
{
  TextureImage = new Image;
  TextureHandle = 0;

  AmbientOn = 0;
  SpecularOn = 0;
  DiffuseOn = 0;
  ShininessOn = 0;
  AlphaOn = 0;
  ReflectOn = 0;
  TwosideOn = 0;
  TextureOn = 0;

  ID = 0;
}


mslGLMaterial::~mslGLMaterial()
{
  //  if(TextureImage != NULL)
  //    free(TextureImage);
}

void mslGLMaterial::SetMaterial()
{
  GLenum drawmode;

  if(TwosideOn)
    drawmode = GL_FRONT_AND_BACK;
  else
    drawmode = GL_FRONT;

  if(TextureOn)
    {
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, TextureHandle);
    }
  else
    glDisable(GL_TEXTURE_2D);

  if(DiffuseOn)
    {
      glMaterialfv(drawmode, GL_DIFFUSE, Diffuse);
      if(AlphaOn)
	{
	  glEnable(GL_BLEND);
	  glColor4f(Color[0], Color[1], Color[2], Alpha);
	}
      else
	{
	  glDisable(GL_BLEND);
	  glColor4f(Color[0], Color[1], Color[2], 1.0);
	}
    }

  if(AmbientOn)   glMaterialfv(drawmode, GL_AMBIENT, Ambient);
  if(SpecularOn)  glMaterialfv(drawmode, GL_SPECULAR, Specular);
  if(ShininessOn) glMaterialf(drawmode, GL_SHININESS, Shininess);
}


int mslGLMaterial::ImageLoad(int id, string path, string filename)
{
    FILE *file;
    unsigned long size;
    unsigned long i;
    unsigned short int planes;
    unsigned short int bpp;
    GLuint gutmp;
    char temp;
    void * mem;

    if ((file = fopen((path + filename).c_str(), "rb"))==NULL) {
      cout << "File " << filename << " Not Found!\n";
      TextureOn = 0;
      return 0;
    }
    else
      TextureOn = 1;

    /* seek through the bmp header, up to the width/height:*/
    fseek(file, 18, SEEK_CUR);


    //**!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // for SGI machine
    TextureImage->sizeX = getint (file);
    cout << "Width of" << filename <<" : " << TextureImage->sizeX << "\n";
    TextureImage->sizeY = getint (file);
    cout << "Height of" << filename <<" : " << TextureImage->sizeY << "\n";
    size = TextureImage->sizeX * TextureImage->sizeY * 3;
    planes = getshort(file);
    if (planes != 1) {
      cout << "Planes from " << filename << " is not 1: " <<planes << "\n";
      return 0;
    }
    bpp = getshort(file);
    if (bpp != 24) {
      cout << "Bpp from " << filename << " is not 24: " << bpp <<" \n";
      return 0;
    }

    /*
    //  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // for linux
    unsigned long sizex, sizey;

    if ((i = fread(&sizex, 4, 1, file)) != 1) {
      cout << "Error reading width from " << filename << ".\n";
      return 0;
    }
    else  TextureImage->sizeX = (unsigned long) sizex;
    //    cout << "Width of " << filename << " is: " << TextureImage->sizeX << "\n";
    if ((i = fread(&sizey, 4, 1, file)) != 1) {
      cout << "Error reading height from " <<  filename << endl;
      return 0;
    }
    else  TextureImage->sizeY = (unsigned long) sizey;
    //    cout << "Height of " << filename <<" : " << TextureImage->sizeY << "\n";

    size = TextureImage->sizeX * TextureImage->sizeY * 3;
    if ((fread(&planes, 2, 1, file)) != 1) {
      cout << "Error reading planes from " <<  filename << endl;
      return 0;
    }
    if (planes != 1) {
      cout << "Planes from " << filename << " is not 1: " <<planes << "\n";
      return 0;
    }
    if ((i = fread(&bpp, 2, 1, file)) != 1) {
      cout << "Error reading bpp from " <<  filename << endl;
      return 0;
    }
    if (bpp != 24) {
      cout << "Bpp from " << filename << " is not 24: " << bpp <<" \n";
      return 0;
    }
    //  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    */

    fseek(file, 24, SEEK_CUR);

    if((mem = realloc(TextureImage->data,
                      sizeof(char)*(size))) == 0) {
        printf("Error reallocating mem\n");
        exit(-1);
    }

    TextureImage->data = (char *) mem;

    if ((i = fread(TextureImage->data, size, 1, file)) != 1) {
      cout << "Error reading image data from " << filename <<" \n";
      return 0;
    }

    for (i=0;i<size;i+=3) { /* reverse all of the colors. (bgr -> rgb)*/
      temp = TextureImage->data[i];
      TextureImage->data[i] = TextureImage->data[i+2];
      TextureImage->data[i+2] = temp;
    }

    glGenTextures((GLsizei)1, &gutmp);
    TextureHandle = (GLuint) gutmp;

    glBindTexture(GL_TEXTURE_2D, TextureHandle);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, TextureImage->sizeX,
		 TextureImage->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE,
		 TextureImage->data);

    return 1;
}


void mslGLMaterial::Clear()
{
  TextureHandle = 0;

  TextureOn = 0;
  AmbientOn = 0;
  SpecularOn = 0;
  DiffuseOn = 0;
  ShininessOn = 0;
  AlphaOn = 0;
  ReflectOn = 0;
  TwosideOn = 0;
}

void mslGLMaterial::Print()
{
  cout << "Texture Name: " << Name << endl;
  cout << "Texture Handel: " << TextureHandle << endl;
  cout << "Material ID: " << ID << endl;

  if(DiffuseOn)
    cout << "Diffuse: " << Diffuse[0] << "  " << Diffuse[1] << " " << Diffuse[2] << endl;
  if(AmbientOn)
    cout << "Ambient: " << Ambient[0] << "  " << Ambient[1] << " " << Ambient[2] << endl;
  if(SpecularOn)
    cout << "Specular: " << Specular[0] << "  " << Specular[1] << " " << Specular[2] << endl;
  if(ShininessOn)
    cout << "Shininess: " << Shininess << endl;
}


mslGLFace::mslGLFace()
{
  VerticeCoord = NULL; //new mslGLVertex[2];
  NormalCoord = NULL ; //new mslGLNormal[2];
  TextureCoord = NULL; //new mslGLTexCoord[2];

  NumberOfPoint = 0;
  NumberOfNormal = 0;
  NumberOfTexCoord = 0;

  NormalOn = 1;         // open normal
  TextureOn = 1;        // open texure
  ColorOn = 1;          // open color

  MaterialID = -1;
}


mslGLFace::~mslGLFace()
{
  /*
  if(VerticeCoord != NULL)
    free(VerticeCoord);

  if(NormalCoord != NULL)
    free(NormalCoord);

  if(TextureCoord != NULL)
    free(TextureCoord);
  */
}

void mslGLFace::AddVertex(const mslGLVertex& ver)
{
  if(VerticeCoord == NULL)
    {
      if((VerticeCoord = (mslGLVertex *) malloc(sizeof(mslGLVertex))) == 0)
	{
	  printf("Error reallocating mem\n");
	  exit(-1);
	}
    }
  else
    if((VerticeCoord  = (mslGLVertex *) realloc(VerticeCoord,
					        sizeof(mslGLVertex)*(NumberOfPoint+1))) == 0)
      {
	printf("Error reallocating mem\n");
	exit(-1);
      }

  VerticeCoord[NumberOfPoint].x = ver.x;
  VerticeCoord[NumberOfPoint].y = ver.y;
  VerticeCoord[NumberOfPoint].z = ver.z;

  NumberOfPoint ++;
}

void mslGLFace::AddNormal(const mslGLNormal& nor)
{
  if(NormalCoord == NULL)
    {
      if((NormalCoord = (mslGLNormal *) malloc(sizeof(mslGLNormal))) == 0)
	{
	  printf("Error reallocating mem\n");
	  exit(-1);
	}
    }
  else
    if((NormalCoord  = (mslGLNormal *) realloc(NormalCoord,
					       sizeof(mslGLNormal)*(NumberOfNormal+1))) == 0)
      {
	printf("Error reallocating mem\n");
	exit(-1);
      }

  NormalCoord[NumberOfNormal].x = nor.x;
  NormalCoord[NumberOfNormal].y = nor.y;
  NormalCoord[NumberOfNormal].z = nor.z;

  NumberOfNormal ++;
}

void mslGLFace::AddTexCoord(const mslGLTexCoord& tex)
{
  if(TextureCoord == NULL)
    {
      if((TextureCoord = (mslGLTexCoord *) malloc(sizeof(mslGLTexCoord))) == 0)
	{
	  printf("Error reallocating mem\n");
	  exit(-1);
	}
    }
  else
    if((TextureCoord  = (mslGLTexCoord *) realloc(TextureCoord,
						  sizeof(mslGLTexCoord)*(NumberOfTexCoord+1))) == 0)
      {
	printf("Error reallocating mem\n");
	exit(-1);
      }

  TextureCoord[NumberOfTexCoord].x = tex.x;
  TextureCoord[NumberOfTexCoord++].y = tex.y;
}

void mslGLFace::AddVertex(const MSLVector& ver)
{
  void *mem;

  if((mem = realloc(VerticeCoord,
                    sizeof(mslGLVertex)*(NumberOfPoint+1))) == 0) {
    printf("Error reallocating mem\n");
    exit(-1);
  }

  VerticeCoord=(mslGLVertex *) mem;

  VerticeCoord[NumberOfPoint].x = ver[0];
  VerticeCoord[NumberOfPoint].y = ver[1];
  VerticeCoord[NumberOfPoint].z = ver[2];

  NumberOfPoint++;
}

void mslGLFace::AddNormal(const MSLVector& nor)
{
  void *mem;

  if((mem = realloc(NormalCoord,
                    sizeof(mslGLNormal)*(NumberOfNormal+1))) == 0) {
    printf("Error reallocating mem\n");
    exit(-1);
  }

  NormalCoord=(mslGLNormal *) mem;

  NormalCoord[NumberOfNormal].x = nor[0];
  NormalCoord[NumberOfNormal].y = nor[1];
  NormalCoord[NumberOfNormal].z = nor[2];

  NumberOfNormal ++;
}

void mslGLFace::AddTexCoord(const MSLVector& tex)
{
  void *mem;

  if((mem = realloc(TextureCoord,
                    sizeof(mslGLTexCoord)*(NumberOfTexCoord+1))) == 0) {
    printf("Error reallocating mem\n");
    exit(-1);
  }

  TextureCoord=(mslGLTexCoord *) mem;

  TextureCoord[NumberOfTexCoord].x = tex[0];
  TextureCoord[NumberOfTexCoord].y = tex[1];

  NumberOfTexCoord++;
}


void mslGLFace::Clear()
{
  NumberOfPoint = 0;
  NumberOfNormal = 0;
  NumberOfTexCoord = 0;

  /*
  if(VerticeCoord != NULL)
    free(VerticeCoord);
  if(NormalCoord != NULL)
    free(NormalCoord);
  if(TextureCoord != NULL)
    free(TextureCoord);
  */

  VerticeCoord = NULL; //new mslGLVertex;
  NormalCoord = NULL; //new mslGLNormal;
  TextureCoord = NULL; //new mslGLTexCoord;

  NormalOn = 1;         // open normal
  TextureOn = 1;        // open texure
  ColorOn = 1;          // open color

  MaterialID = -1;
}

void mslGLFace::PrintVertex()
{
  int i;
  mslGLVertex v;

  for(i=0; i<NumberOfPoint; i++)
    {
      v = VerticeCoord[i];
      cout << "(" << v.x << " " << v.y << " "<< v.z << ") ";
    }

  cout << endl;
}


void mslGLFace::DrawFace()
{
  int i;

  mslGLVertex v1, v2, v3;
  mslGLNormal n1, n2, n3, n;
  mslGLTexCoord t1, t2, t3;

  v1 = VerticeCoord[0];
  v2 = VerticeCoord[1];

  if(NormalOn)
    {
      n1 = NormalCoord[0];
      n2 = NormalCoord[1];
    }

  if(TextureOn)
    {
      t1 = TextureCoord[0];
      t2 = TextureCoord[1];
    }

  glBegin(GL_TRIANGLES);

  for(i=0; i<NumberOfPoint-2; i++)
    {
      v3 = VerticeCoord[i+2];
      if(NormalOn)  n3 = NormalCoord[i+2];
      if(TextureOn) t3 = TextureCoord[i+2];

      if((float)i/2.0==0)
	{
	  if(NormalOn)
	    {
	      glNormal3f(n1.x, n1.y, n1.z);
	      if(TextureOn) glTexCoord2f(t1.x, t1.y);
	      glVertex3f(v1.x, v1.y, v1.z);

	      glNormal3f(n2.x, n2.y, n2.z);
	      if(TextureOn) glTexCoord2f(t2.x, t2.y);
	      glVertex3f(v2.x, v2.y, v2.z);

	      glNormal3f(n3.x, n3.y, n3.z);
	      if(TextureOn) glTexCoord2f(t3.x, t3.y);
	      glVertex3f(v3.x, v3.y, v3.z);
	    }
	  else
	    {
	      n = NormalCompute(v1, v2, v3);
	      glNormal3f(n.x, n.y, n.z);

	      if(TextureOn) glTexCoord2f(t1.x, t1.y);
	      glVertex3f(v1.x, v1.y, v1.z);
	      if(TextureOn) glTexCoord2f(t2.x, t2.y);
	      glVertex3f(v2.x, v2.y, v2.z);
	      if(TextureOn) glTexCoord2f(t3.x, t3.y);
	      glVertex3f(v3.x, v3.y, v3.z);
	    }
	}
      else
	{
	  if(NormalOn)
	    {
	      glNormal3f(n2.x, n2.y, n2.z);
	      if(TextureOn) glTexCoord2f(t2.x, t2.y);
	      glVertex3f(v2.x, v2.y, v2.z);

	      glNormal3f(n1.x, n1.y, n1.z);
	      if(TextureOn) glTexCoord2f(t1.x, t1.y);
	      glVertex3f(v1.x, v1.y, v1.z);

	      glNormal3f(n3.x, n3.y, n3.z);
	      if(TextureOn) glTexCoord2f(t3.x, t3.y);
	      glVertex3f(v3.x, v3.y, v3.z);
	    }
	  else
	    {
	      n = NormalCompute(v2, v1, v3);
	      glNormal3f(n.x, n.y, n.z);

	      if(TextureOn) glTexCoord2f(t2.x, t2.y);
	      glVertex3f(v2.x, v2.y, v2.z);
	      if(TextureOn) glTexCoord2f(t1.x, t1.y);
	      glVertex3f(v1.x, v1.y, v1.z);
	      if(TextureOn) glTexCoord2f(t3.x, t3.y);
	      glVertex3f(v3.x, v3.y, v3.z);
	    }
	}

      v1 = v2; v2 = v3;
      n1 = n2; n2 = n3;
      t1 = t2; t2 = t3;
    }

  glEnd();
}


mslGLObject::mslGLObject()
{
  //  Name = new char[MAXNAME_LENGTH];

  ObjectMaterialLib = NULL;
  ObjectFaceLib = NULL;

  NumberOfMaterial = 0;
  NumberOfFace = 0;

  BoundingBoxMin[0] = 0.0;
  BoundingBoxMin[1] = 0.0;
  BoundingBoxMin[2] = 0.0;

  BoundingBoxMax[0] = 0.0;
  BoundingBoxMax[1] = 0.0;
  BoundingBoxMax[2] = 0.0;
}


mslGLObject::~mslGLObject()
{
}


void mslGLObject::Clear()
{

  ObjectMaterialLib = NULL;
  ObjectFaceLib = NULL;

  NumberOfMaterial = 0;
  NumberOfFace = 0;

  BoundingBoxMin[0] = 0.0;
  BoundingBoxMin[1] = 0.0;
  BoundingBoxMin[2] = 0.0;

  BoundingBoxMax[0] = 0.0;
  BoundingBoxMax[1] = 0.0;
  BoundingBoxMax[2] = 0.0;
}

int mslGLObject::ReadModelFile(const string& path, const string& fileName)
{
    FILE	 *objFile;

    int          currentMaterialID = -1;

    char	 buffer[BUFFER_SIZE];
    char	 token[BUFFER_SIZE];
    char	*next		= NULL;
    char	*backslash	= NULL;

    int		 width		= 0;

    int 	 numTris	= 0;
    int 	 numPolys	= 0;
    int 	 numGroups	= 0;

    mslGLVertex*         v       = NULL;
    unsigned int	 vCount	        = 0;
    unsigned int	 vAvailable	= 0;

    mslGLNormal*         n        = NULL;
    unsigned int	 nCount	        = 0;
    unsigned int	 nAvailable	= 0;

    mslGLTexCoord*       t       = NULL;
    unsigned int	 tCount	        = 0;
    unsigned int	 tAvailable	= 0;

    /* tmp count vars */
    int		 i, j;
    float       f1, f2, f3;
    MSLVector       tv1(3), tv2(2), tv3(9);

    Position[0] = 0.0;
    Position[1] = 0.0;
    Position[2] = 0.0;

    Orientation[0] = 0.0;
    Orientation[1] = 0.0;
    Orientation[2] = 0.0;

    Scale[0] = 1.0;
    Scale[1] = 1.0;
    Scale[2] = 1.0;

    if ((objFile = fopen((path + fileName).c_str(), "r")) == NULL)
      {
	cout << "can not open model file!" << endl;
	return 1;
      }

    Name = fileName;

    while (fgets(buffer, BUFFER_SIZE, objFile) != NULL)
    {

	while ((backslash = strchr(buffer, '\\')) != NULL)
	{

	    *backslash++ = ' ';
	    *backslash   = '\0';


	    if (fgets(backslash, (int)(BUFFER_SIZE - strlen(buffer)), objFile)
		== NULL)
		break;
	}

	/* find first non-"space" character in line */
	for (next = buffer; *next != '\0' && isspace(*next); next++)
	    /* EMPTY */ {};

	/* skip blank lines and comments ('$' is comment in "cow.obj") */
	if (*next == '\0' || *next == '#' || *next == '!' || *next == '$')
	    continue;

	/* extract token */
	sscanf(next, "%s%n", token, &width);
	next += width;

	if (SAME(token, "v"))
	{
	    sscanf(next, "%f %f %f", &f1, &f2, &f3);

	    GROW(v, mslGLVertex);
	    if(v == 0)
	      {
		cout << "memory allocation error!" << endl;
		exit(-1);
	      }

	    v[vCount].x = f1;
	    v[vCount].y = f2;
	    v[vCount].z = f3;

	    ++vCount;
	}
	else
	if (SAME(token, "vn"))
	{
	    sscanf(next, "%f %f %f", &f1, &f2, &f3);

	    GROW(n, mslGLNormal);
	    if(n == 0)
	      {
		cout << "memeory allocation error!" << endl;
		exit(-1);
	      }

	    n[nCount].x = f1;
	    n[nCount].y = f2;
	    n[nCount].z = f3;

	    ++nCount;
	}
	else
	if (SAME(token, "vt"))
	{
	    sscanf(next, "%f %f", &f1, &f2);

	    GROW(t, mslGLTexCoord);
	    if(t == 0)
	      {
		cout << "memeory allocation error!" << endl;
		exit(-1);
	      }

	    t[tCount].x = f1;
	    t[tCount].y = f2;

	    ++tCount;
	}
	else
	if (SAME(token, "g"))
 	{
	    ++numGroups;
	}
	else
	if (SAME(token, "f") ||
	    SAME(token, "fo"))
	{
	    int 	 count;
	    int		 textureValid = 1;
	    int		 normalsValid = 1;
	    int		 vi[FACE_SIZE];
	    int		 ti[FACE_SIZE];
	    int		 ni[FACE_SIZE];

	    char	*slash;
	    char	 vertexData[256];

	    mslGLFace       tFace;

	    for (count = 0; count < FACE_SIZE; count++)
	    {
		if (sscanf(next, "%s%n", vertexData, &width) != 1)
		    break;
		next += width;
		vi[count] = (int)strtol(vertexData, NULL, 10);
		ti[count] = 0;
		if ((slash = strchr(vertexData, '/')) == NULL ||
		    (ti[count] = (int)strtol(slash+1, NULL, 10)) == 0)
		    textureValid = 0;
		ni[count] = 0;
		if (slash == NULL || (slash = strchr(slash+1, '/')) == NULL ||
		    (ni[count] = (int)strtol(slash+1, NULL, 10)) == 0)
		    normalsValid = 0;

		/*
		 * form cannonical indices:
		 *   convert ".obj" 1-based indices to 0-based (subtract 1)
		 *   convert negative indices to positive (count from 0)
		 */
		if (vi[count] >= 0)
		    vi[count] -= 1;
		else
		    vi[count]  = vCount - vi[count];

		if (ti[count] >= 0)
		    ti[count] -= 1;
		else
		    ti[count]  = tCount - ti[count];

		if (ni[count] >= 0)
		    ni[count] -= 1;
		else
		    ni[count]  = nCount - ni[count];
	    }

	    for (i = 0; i < count; i++)
	      {
		tFace.AddVertex(v[vi[i]]);
	      }

	    tFace.ColorOn = 0;
	    tFace.MaterialID = currentMaterialID;

	    /* setup normal MSLVector information */
	    if (normalsValid)
	      {
		tFace.NormalOn = 1;
		for (i = 0; i < count; i++)
		  tFace.AddNormal(n[ni[i]]);
	      }
	    else
	      tFace.NormalOn = 0;

	    if (textureValid)
	      {
		tFace.TextureOn = 1;
		for (i = 0; i < count; i++)
		  tFace.AddTexCoord(t[ti[i]]);
	      }
	    else
	      tFace.TextureOn = 0;

	    if (count > 2)
	      {
		tFace.NumberOfPoint = count;
		AddFace(tFace);
		numTris += count - 2;
		numPolys++;

	      }
	}
	else
	if (SAME(token, "usemtl"))
	{
	    char	mtlName[MAXNAME_LENGTH];
	    sscanf(next, "%s", mtlName);
	    currentMaterialID = SetCurrentMaterialID(mtlName);
	}
	else
	  if (SAME(token, "mtllib"))
	    {
	      char	libName[MAXNAME_LENGTH];
	      sscanf(next, "%s", libName);
	      LoadMaterialFile(path, libName);
	    }
	  else
	    if (
		SAME(token, "bevel")	||
		SAME(token, "bmat")		||
		SAME(token, "bsp")		||
		SAME(token, "bzp")		||
		SAME(token, "c_interp")	||
		SAME(token, "cdc")		||
		SAME(token, "con")		||
		SAME(token, "cstype")	||
		SAME(token, "ctech")	||
		SAME(token, "curv")		||
		SAME(token, "curv2")	||
		SAME(token, "d_interp")	||
		SAME(token, "deg")		||
		SAME(token, "end")		||
		SAME(token, "hole")		||
		SAME(token, "l")		||
		SAME(token, "lod")		||
		SAME(token, "maplib")	||
		SAME(token, "mg")		||
		SAME(token, "o")		||
		SAME(token, "p")		||
		SAME(token, "param")	||
		SAME(token, "parm")		||
		SAME(token, "res")		||
		SAME(token, "s")		||
		SAME(token, "scrv")		||
		SAME(token, "shadow_obj")	||
		SAME(token, "sp")		||
		SAME(token, "stech")	||
		SAME(token, "step")		||
		SAME(token, "surf")		||
		SAME(token, "trace_obj")	||
		SAME(token, "trim")		||
		SAME(token, "usemap")	||
		SAME(token, "vp"))
	      {
		++numSkip;
	      }
#ifndef	STRICT_OBJ_FORMAT
	/*
	 * reset vertex data array counters -- this is not
	 * part of the OBJ format, but proves quite handy.
	 */
	    else
	      if (SAME(token, "RESET"))
		{
		  vCount = 0;
		  nCount = 0;
		  tCount = 0;
		}
#endif
	      else
		{
		  cout << "unrecognize format" << endl;
		  ++numOther;
		}
    }

    /* close Wavefront ".obj" file */
    fclose(objFile);

    if (v != NULL) free(v);
    if (n != NULL) free(n);
    if (t != NULL) free(t);

    for(i=0; i<NumberOfFace; i++)
      {
	for(j=0; j<ObjectFaceLib[i].NumberOfPoint; j++)
	  {
	    if(ObjectFaceLib[i].VerticeCoord[j].x>BoundingBoxMax[0])
	      BoundingBoxMax[0] = ObjectFaceLib[i].VerticeCoord[j].x;
	    if(ObjectFaceLib[i].VerticeCoord[j].y>BoundingBoxMax[1])
	      BoundingBoxMax[1] = ObjectFaceLib[i].VerticeCoord[j].y;
	    if(ObjectFaceLib[i].VerticeCoord[j].z>BoundingBoxMax[2])
	      BoundingBoxMax[2] = ObjectFaceLib[i].VerticeCoord[j].z;

	    if(ObjectFaceLib[i].VerticeCoord[j].x<BoundingBoxMin[0])
	      BoundingBoxMin[0] = ObjectFaceLib[i].VerticeCoord[j].x;
	    if(ObjectFaceLib[i].VerticeCoord[j].y<BoundingBoxMin[1])
	      BoundingBoxMin[1] = ObjectFaceLib[i].VerticeCoord[j].y;
	    if(ObjectFaceLib[i].VerticeCoord[j].z<BoundingBoxMin[2])
	      BoundingBoxMin[2] = ObjectFaceLib[i].VerticeCoord[j].z;
	  }
      }

    return 0;
}


MSLPoint3d mslGLObject::PointCurrentState(const MSLPoint3d& po, int mode)
{
  MSLVector vp1(3);

  vp1[0] = po.xcoord() * Scale[0];
  vp1[1] = po.ycoord() * Scale[1];
  vp1[2] = po.zcoord() * Scale[2];

  vp1 = point_x_rotation(Orientation[0]*PI/180.0, vp1);
  vp1 = point_y_rotation(Orientation[1]*PI/180.0, vp1);
  vp1 = point_z_rotation(Orientation[2]*PI/180.0, vp1);

  if(mode == OBSTACLE_MODE)
    {
      vp1[0] = vp1[0] + Position[0];
      vp1[1] = vp1[1] + Position[1];
      vp1[2] = vp1[2] + Position[2];
    }

  return MSLPoint3d(vp1[0],vp1[1],vp1[2]);
}



list<MSLTriangle> mslGLObject::SetBoundingBoxTriangle(int mode)
{
  list<MSLTriangle> BoundingBoxTriangle;

  BoundingBoxTriangle.clear();
  MSLPoint3d p1, p2, p3;

  p1 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p2 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p3 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  p2 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p3 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p3 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p3 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p3 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  p3 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p3 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  p3 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  p3 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMin[2]);
  p3 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMax[1], BoundingBoxMax[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p3 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  p1 = MSLPoint3d(BoundingBoxMax[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  p2 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMin[2]);
  p3 = MSLPoint3d(BoundingBoxMin[0], BoundingBoxMin[1], BoundingBoxMax[2]);
  p1 = PointCurrentState(p1, mode);
  p2 = PointCurrentState(p2, mode);
  p3 = PointCurrentState(p3, mode);
  BoundingBoxTriangle.push_front(MSLTriangle(p1, p2, p3));

  return BoundingBoxTriangle;
}



void mslGLObject::LoadMaterialFile(const string& path, const string& filename)
{
    FILE	*mtlFile;
    char	 buffer[BUFFER_SIZE];
    char	 token[BUFFER_SIZE];
    char	*next;
    char	*backslash;
    int		 width	= 0;
    //    int	         i;
    int 	 inProgress = 0;

    mslGLMaterial   tmat;
    list<string>::iterator  tstr;

    /* have we already loaded this file ? */
    forall(tstr, MaterialFileList)
      if (strcmp(filename.c_str(), (*tstr).c_str()) == 0)
	{
	    return;
	}

    /* remember file name for future reference */
    if (MaterialFileList.size() < MAX_MTL_FILES)
      {
	MaterialFileList.push_back(filename);
      }
    else
      {
	cout << "You have exceed the limit of the number of the material file" << endl;
      }

    /* open file */
    if ((mtlFile = fopen((path + filename).c_str(), "r")) == NULL)
      {
	cout << "Can not open the material file" << endl;
	return;
      }

    /* read Wavefront ".mtl" file */
    while (fgets(buffer, BUFFER_SIZE, mtlFile) != NULL)
    {
	while ((backslash = strchr(buffer, '\\')) != NULL)
	    if (fgets(backslash, BUFFER_SIZE - strlen(buffer), mtlFile) == NULL)
		break;

	for (next = buffer; *next != '\0' && isspace(*next); next++)
	    /* EMPTY */ {};
	if (*next == '\0' || *next == '#' || *next == '!' || *next == '$')
	    continue;
	sscanf(next, "%s%n", token, &width);
	next += width;

	/* identify token */
	if (SAME(token, "newmtl"))
	  {
	    if(inProgress)
	      {
		AddMaterial(path, tmat);
		//cout << "TextureName: " << ObjectMaterialLib[NumberOfMaterial-1].Name << endl;
	      }

	    tmat.Clear();
	    inProgress = 1;
	    sscanf(next, "%s", tmat.Name);
	  }
	else
	  if (SAME(token, "Ka"))
	    {
	      sscanf(next, "%f %f %f",
		     &tmat.Ambient[0],
		     &tmat.Ambient[1],
		     &tmat.Ambient[2]);
	      tmat.AmbientOn= 1;
	    }
	  else
	    if (SAME(token, "Kd"))
	      {
		sscanf(next, "%f %f %f",
		       &tmat.Diffuse[0],
		       &tmat.Diffuse[1],
		       &tmat.Diffuse[2]);
		tmat.Color[0] = tmat.Diffuse[0];
		tmat.Color[1] = tmat.Diffuse[1];
		tmat.Color[2] = tmat.Diffuse[2];
		tmat.DiffuseOn = 1;
	      }
	    else
	      if (SAME(token, "Ks"))
		{
		  sscanf(next, "%f %f %f",
			 &tmat.Specular[0],
			 &tmat.Specular[1],
			 &tmat.Specular[2]);
		  tmat.SpecularOn = 1;
		}
	      else
		if (SAME(token, "Tr"))
		  {
		    float alpha = 1.0f;
		    sscanf(next, "%f", &alpha);
		    tmat.Alpha = 1.0f - alpha;
		    tmat.AlphaOn = 1;
		  }
		else
		  if (SAME(token, "Ns"))
		    {
		      sscanf(next, "%f", &tmat.Shininess);

		      /* clamp shininess range */
		      if (tmat.Shininess <   0.0f)
			tmat.Shininess =   0.0f;
		      else
			if (tmat.Shininess > 128.0f)
			  tmat.Shininess = 128.0f;

		      tmat.ShininessOn = 1;
		    }
		  else
		    if (SAME(token, "map_Kd"))
		      {
			ParseTexture(next, &tmat);
			tmat.TextureOn = 1;
		      }
		    else
		      if (SAME(token, "refl"))
			{
			  strcpy(tmat.Reflect, "sphere");
			  ParseTexture(next, &tmat);
			  tmat.ReflectOn = 1;
			}
		      else
			if (
			    SAME(token, "Ni")	        ||
			    SAME(token, "Tf")		||
			    SAME(token, "bump")	        ||
			    SAME(token, "d")		||
			    SAME(token, "decal")	||
			    SAME(token, "illum")	||
			    SAME(token, "map_Ka")	||
			    SAME(token, "map_Ks")	||
			    SAME(token, "map_Ns")	||
			    SAME(token, "map_d")	||
			    SAME(token, "sharpness")	||
			    SAME(token, "vp"))
			  {
			    numSkip++;
			  }
#ifndef	STRICT_OBJ_FORMAT
	/*
	 * indicate that this material is two-sided
	 */
			else
			  if (SAME(token, "TWOSIDE"))
			    {
			      tmat.TwosideOn = 1;
			    }
#endif
			  else
			    {
			      cout << "unrecognized format" << endl;
			      numOther++;
			    }
    }

    if (inProgress)
      AddMaterial(path, tmat);

    /* close Wavefront ".mtl" file */
    fclose(mtlFile);
}



int mslGLObject::SetCurrentMaterialID(char * name)
{
  int i, id;

  id = -1;
  for(i=0; i<NumberOfMaterial; i++)
    {
      if(strcmp(ObjectMaterialLib[i].Name, name) == 0)
	{
	  id = ObjectMaterialLib[i].ID;
	}
    }

  return id;    // set to be not material

}


void mslGLObject::AddMaterial(const string& path, const mslGLMaterial& mat)
{
  if(ObjectMaterialLib == NULL)
    {
      if((ObjectMaterialLib = (mslGLMaterial *) malloc(sizeof(mslGLMaterial))) == 0)
	{
	  printf("Error reallocating mem\n");
	  exit(-1);
	}
    }
  else
    if((ObjectMaterialLib =
	(mslGLMaterial *) realloc(ObjectMaterialLib,
				  sizeof(mslGLMaterial)*(NumberOfMaterial+1))) == 0)
      {
	printf("Error reallocating mem\n");
	exit(-1);
      }

  ObjectMaterialLib[NumberOfMaterial] = mat;

  ObjectMaterialLib[NumberOfMaterial].ID = NumberOfMaterial;
  ObjectMaterialLib[NumberOfMaterial].Shininess = mat.Shininess;
  if(mat.TextureOn)
    ObjectMaterialLib[NumberOfMaterial].ImageLoad(NumberOfMaterial, path,  mat.TextureName);

  //  ObjectMaterialLib[NumberOfMaterial].Print();

  NumberOfMaterial++;
}


void mslGLObject::AddFace(const mslGLFace& face)
{
  if(ObjectFaceLib == NULL)
    {
      if((ObjectFaceLib = (mslGLFace *) malloc(sizeof(mslGLFace))) == 0)
	{
	  printf("Error reallocating mem\n");
	  exit(-1);
	}
    }
  else
    if((ObjectFaceLib = (mslGLFace *) realloc(ObjectFaceLib,
					      sizeof(mslGLFace)*(NumberOfFace+1))) == 0) {
      printf("Error reallocating mem\n");
      exit(-1);
    }

  ObjectFaceLib[NumberOfFace] = face;

  NumberOfFace++;
}


/*
 * parse wafefront material file texture definition
 */
void mslGLObject::ParseTexture (char *next, mslGLMaterial *m)
{
    int		 width = 0;

    /* set default texture scale factors */
    m->Su = 1.0f;
    m->Sv = 1.0f;

    /* parse texture name */
    sscanf(next, "%s%n", m->TextureName, &width);

    /* parse texture option strings */
    do
    {
	next += width;

	if (strcmp(m->TextureName, "-mm") == 0)
	    sscanf(next, "%f %f %s%n",     &m->Su, &m->Sv, m->TextureName, &width);
	else
	if (strcmp(m->TextureName, "-s") == 0)
	    sscanf(next, "%f %f %*f %s%n", &m->Su, &m->Sv, m->TextureName, &width);
	else
	if (strcmp(m->TextureName, "-t") == 0)
	    sscanf(next, "%f %f %*f %s%n", &m->Su, &m->Sv, m->TextureName, &width);
	else
	if (strcmp(m->TextureName, "-type") == 0)
	    sscanf(next, "%s %s%n", m->Reflect, m->TextureName, &width);
	else
	    break;
    } while (1);
}


void mslGLObject::ObjectDraw()
{
  int i;
  int currentmat;

  glPushMatrix();

  glTranslatef(Position[0], Position[1], Position[2]);
  glRotatef(Orientation[2], 1.0, 0.0, 0.0);
  glRotatef(Orientation[1], 0.0, 1.0, 0.0);
  glRotatef(Orientation[0], 0.0, 0.0, 1.0);
  glScalef(Scale[0], Scale[1], Scale[2]);

  currentmat = ObjectFaceLib[0].MaterialID;
  if(currentmat != -1)
    SetMaterial(currentmat);

  for(i=0; i<NumberOfFace; i++)
    {
      if(currentmat != ObjectFaceLib[i].MaterialID)
	{
	  currentmat = ObjectFaceLib[i].MaterialID;
	  if(currentmat != -1)
	    SetMaterial(currentmat);
	}

      ObjectFaceLib[i].DrawFace();
    }

  glPopMatrix();
}




void mslGLObject::ObjectBoundingBoxDraw()
{

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  GLfloat Diffuse[] = {1.0, 0.0, 0.0};
  GLfloat Ambient[] = {1.0, 0.0, 0.0};
  GLfloat Specular[] = {1.0, 0.0, 0.0};
  GLfloat Shininess[] = {1.0, 0.0, 0.0};

  glPushMatrix();

  glTranslatef(Position[0], Position[1], Position[2]);
  glRotatef(Orientation[2], 1.0, 0.0, 0.0);
  glRotatef(Orientation[1], 0.0, 1.0, 0.0);
  glRotatef(Orientation[0], 0.0, 0.0, 1.0);
  glScalef(Scale[0], Scale[1], Scale[2]);

  glLineWidth(0.2);

  glColor4f(1.0, 0.0, 1.0, 1.0);
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


void mslGLObject::SetMaterial(int matindex)
{
  ObjectMaterialLib[matindex].SetMaterial();

  //  ObjectMaterialLib[matindex].Print();
}



void mslGLObject::ObjectHighlight()
{

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  GLfloat Diffuse[] = {1.0, 0.0, 0.0};
  GLfloat Ambient[] = {1.0, 0.0, 0.0};
  GLfloat Specular[] = {1.0, 0.0, 0.0};
  GLfloat Shininess[] = {1.0, 0.0, 0.0};

  glPushMatrix();

  glTranslatef(Position[0], Position[1], Position[2]);
  glRotatef(Orientation[2], 1.0, 0.0, 0.0);
  glRotatef(Orientation[1], 0.0, 1.0, 0.0);
  glRotatef(Orientation[0], 0.0, 0.0, 1.0);
  glScalef(Scale[0], Scale[1], Scale[2]);

  glLineWidth(0.5);

  glColor4f(1.0, 1.0, 0.0, 1.0);
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

void mslGLObject::SetObjectPosition(const MSLVector& pos)
{
  Position[0] = pos[0];
  Position[1] = pos[1];
  Position[2] = pos[2];
}

void mslGLObject::SetObjectOrientation(const MSLVector& ori)
{
  Orientation[0] = ori[0];
  Orientation[1] = ori[1];
  Orientation[2] = ori[2];
}

void mslGLObject::SetObjectScale(const MSLVector& sca)
{
  Scale[0] = sca[0];
  Scale[1] = sca[1];
  Scale[2] = sca[2];
}


void mslGLObject::SetBodyPositionChange(const MSLVector& posc)
{
  Position[0] = Position[0] + posc[0];
  Position[1] = Position[1] + posc[1];
  Position[2] = Position[2] + posc[2];
}


void mslGLObject::SetBodyOrientationChange(const MSLVector& oric)
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

void mslGLObject::SetBodyScaleChange(const MSLVector& scac)
{
  Scale[0] = Scale[0] * scac[0];
  Scale[1] = Scale[1] * scac[1];
  Scale[2] = Scale[2] * scac[2];
}


void mslGLObject::PrintFace()
{
  int i;

  cout << "Name of Object: " << Name << endl;

  cout << "NumberOfFace: " << NumberOfFace << endl;

  for(i=0; i<NumberOfFace; i++)
    {
      cout << "face " << i << ":  ";
      ObjectFaceLib[i].PrintVertex();
    }
}

void mslGLObject::PrintMaterial()
{
  int i;

  for(i=0; i<NumberOfMaterial; i++)
    ObjectMaterialLib[i].Print();
}


void mslGLObject::PrintState()
{
  cout << "Position: " << Position[0]
       << " " << Position[1]
       << " " << Position[2]
       << endl;

  cout << "Orientation: " << Orientation[0]
       << " " << Orientation[1]
       << " " << Orientation[2]
       << endl;

  cout << "Scale: " << Scale[0]
       << " " << Scale[1]
       << " " << Scale[2]
       << endl;

}

//  !!!!!!!!!!!!!!!!!!!!!!!! -- come from generalfunctin.C -- 1/5/01

mslGLNormal NormalCompute(const mslGLVertex& v1, const mslGLVertex& v2,
			  const mslGLVertex& v3)
{
  mslGLNormal n;
  MSLVector vv1(3), vv2(3), vv(3);

  vv1[0] = v2.x - v1.x;
  vv1[1] = v2.y - v1.y;
  vv1[2] = v2.z - v1.z;

  vv2[0] = v3.x - v1.x;
  vv2[1] = v3.y - v1.y;
  vv2[2] = v3.z - v1.z;

  crossproduct(vv1, vv2, vv);

  n.x = vv[0];
  n.y = vv[1];
  n.z = vv[2];

  return n;
}

//  !!!!!!!!!!!!!!!!!!!!!!!! -- come from generalfunctin.C -- 1/5/01
