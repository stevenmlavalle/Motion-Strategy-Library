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

//#include <fstream.h>
#include <math.h>

#include "msl/geom_pqp.h"

// Define some useless stream defaults
istream& operator>> (istream& is, PQP_Model& p) {
  // Do nothing -- no reader available
  return is;
}

ostream& operator<< (ostream& os, const PQP_Model& p) {
  os << "PQP_Model ";
  return os;
}




// *********************************************************************
// *********************************************************************
// CLASS:     GeomPQP
//
// *********************************************************************
// *********************************************************************


GeomPQP::GeomPQP(string path = ""):Geom(path) {

  // Define empty transformation for obstacles
  TO[0]=TO[1]=TO[2]=0.0;
  RO[0][0]=RO[1][1]=RO[2][2]=1.0;
  RO[0][1]=RO[0][2]=RO[1][0]= RO[1][2]= RO[2][0]= RO[2][1]=0.0;

  NumBodies = 1; // Default
}




void GeomPQP::LoadEnvironment(string path){

  READ_OPTIONAL_PARAMETER(Obst);

  int i=0;
  list<MSLTriangle>::iterator t;

  Ob.BeginModel();
  PQP_REAL p1[3],p2[3],p3[3];
  forall(t,Obst) {
    p1[0] = (PQP_REAL) t->p1.xcoord();
    p1[1] = (PQP_REAL) t->p1.ycoord();
    p1[2] = (PQP_REAL) t->p1.zcoord();
    p2[0] = (PQP_REAL) t->p2.xcoord();
    p2[1] = (PQP_REAL) t->p2.ycoord();
    p2[2] = (PQP_REAL) t->p2.zcoord();
    p3[0] = (PQP_REAL) t->p3.xcoord();
    p3[1] = (PQP_REAL) t->p3.ycoord();
    p3[2] = (PQP_REAL) t->p3.zcoord();
    Ob.AddTri(p1,p2,p3,i);
    i++;
  }
  Ob.EndModel();
}



void GeomPQP::LoadRobot(string path){

  READ_OPTIONAL_PARAMETER(Robot);

  int i=0;
  list<MSLTriangle>::iterator t;

  Ro.BeginModel();
  PQP_REAL p1[3],p2[3],p3[3];
  forall(t,Robot){
    p1[0] = (PQP_REAL) t->p1.xcoord();
    p1[1] = (PQP_REAL) t->p1.ycoord();
    p1[2] = (PQP_REAL) t->p1.zcoord();
    p2[0] = (PQP_REAL) t->p2.xcoord();
    p2[1] = (PQP_REAL) t->p2.ycoord();
    p2[2] = (PQP_REAL) t->p2.zcoord();
    p3[0] = (PQP_REAL) t->p3.xcoord();
    p3[1] = (PQP_REAL) t->p3.ycoord();
    p3[2] = (PQP_REAL) t->p3.zcoord();
    Ro.AddTri(p1,p2,p3,i);
    i++;
  }
  Ro.EndModel();
}




// *********************************************************************
// *********************************************************************
// CLASS:     GeomPQP2D
//
// *********************************************************************
// *********************************************************************

GeomPQP2D::GeomPQP2D(string path = ""):GeomPQP(path) {

  LoadEnvironment(FilePath);
}


void GeomPQP2D::LoadEnvironment(string path)
{
  std::ifstream fin;

  Obst.clear();
  fin.open((FilePath+"Obst").c_str());
  if (fin) {
    fin >> ObstPolygons;
    // Make triangles to store in Obst
    Obst = PolygonsToTriangles(ObstPolygons,3.0);
  }
  fin.close();

  int i=0;
  list<MSLTriangle>::iterator t;

  Ob.BeginModel();
  PQP_REAL p1[3],p2[3],p3[3];
  forall(t,Obst) {
    p1[0] = (PQP_REAL) t->p1.xcoord();
    p1[1] = (PQP_REAL) t->p1.ycoord();
    p1[2] = (PQP_REAL) t->p1.zcoord();
    p2[0] = (PQP_REAL) t->p2.xcoord();
    p2[1] = (PQP_REAL) t->p2.ycoord();
    p2[2] = (PQP_REAL) t->p2.zcoord();
    p3[0] = (PQP_REAL) t->p3.xcoord();
    p3[1] = (PQP_REAL) t->p3.ycoord();
    p3[2] = (PQP_REAL) t->p3.zcoord();
    Ob.AddTri(p1,p2,p3,i);
    i++;
  }
  Ob.EndModel();

  //cout << "Obstacle region:\n" << ObstPolygons << "\n";
  //cout << "Number of polygons: " << ObstPolygons.size() << "\n";
}



void GeomPQP2D::LoadRobot(string path){
  std::ifstream fin;

  Robot.clear();
  fin.open((FilePath+"Robot").c_str());
  if (fin) {
    fin >> RobotPolygons;
    // Make triangles to store in Robot
    Robot = PolygonsToTriangles(RobotPolygons,3.0);
  }
  fin.close();

  int i=0;
  list<MSLTriangle>::iterator t;

  Ro.BeginModel();
  PQP_REAL p1[3],p2[3],p3[3];
  forall(t,Robot){
    p1[0] = (PQP_REAL) t->p1.xcoord();
    p1[1] = (PQP_REAL) t->p1.ycoord();
    p1[2] = (PQP_REAL) t->p1.zcoord();
    p2[0] = (PQP_REAL) t->p2.xcoord();
    p2[1] = (PQP_REAL) t->p2.ycoord();
    p2[2] = (PQP_REAL) t->p2.zcoord();
    p3[0] = (PQP_REAL) t->p3.xcoord();
    p3[1] = (PQP_REAL) t->p3.ycoord();
    p3[2] = (PQP_REAL) t->p3.zcoord();
    Ro.AddTri(p1,p2,p3,i);
    i++;
  }
  Ro.EndModel();
}



// *********************************************************************
// *********************************************************************
// CLASS:     GeomPQP2DRigid
//
// *********************************************************************
// *********************************************************************

GeomPQP2DRigid::GeomPQP2DRigid(string path = ""):GeomPQP2D(path) {

  // Compute the maximum deviates -- 2D with rotation
  double dmax = 0.0;
  double mag;
  list<MSLTriangle>::iterator tr;

  LoadRobot(path);

  forall(tr,Robot) {
    // Find maximum effective radius; ignore z distance here
    mag = sqrt(sqr(tr->p1.xcoord())+sqr(tr->p1.ycoord()));
    if (mag > dmax)
      dmax = mag;
    mag = sqrt(sqr(tr->p2.xcoord())+sqr(tr->p2.ycoord()));
    if (mag > dmax)
      dmax = mag;
  }

  MaxDeviates = MSLVector(1.0,1.0,dmax);

}


void GeomPQP2DRigid::SetTransformation(const MSLVector &q){

  // Set translation
  TR[0] = (PQP_REAL)q[0];
  TR[1] = (PQP_REAL)q[1];
  TR[2] = 0.0;

  // Set yaw rotation
  RR[0][0] = (PQP_REAL)(cos(q[2]));
  RR[0][1] = (PQP_REAL)(-sin(q[2]));
  RR[0][2] = 0.0;
  RR[1][0] = (PQP_REAL)(sin(q[2]));
  RR[1][1] = (PQP_REAL)(cos(q[2]));
  RR[1][2] = 0.0;
  RR[2][0] = 0.0;
  RR[2][1] = 0.0;
  RR[2][2] = 1.0;

}


bool GeomPQP2DRigid::CollisionFree(const MSLVector &q){

  SetTransformation(q);

  PQP_CollideResult cres;
  PQP_Collide(&cres,RR,TR,&Ro,RO,TO,&Ob,PQP_FIRST_CONTACT);

  return (cres.NumPairs() == 0);
}


double GeomPQP2DRigid::DistanceComp(const MSLVector &q){

  SetTransformation(q);

  PQP_DistanceResult dres;
  PQP_Distance(&dres,RR,TR,&Ro,RO,TO,&Ob,0.0,0.0);

  return dres.Distance();
}




MSLVector GeomPQP2DRigid::ConfigurationDifference(const MSLVector &q1,
						    const MSLVector &q2)
{
  MSLVector dq(3);

  dq[0] = q2[0] - q1[0];
  dq[1] = q2[1] - q1[1];

  if (fabs(q1[2]-q2[2]) < PI)
    dq[2] = q2[2] - q1[2];
  else {
    if (q1[2] < q2[2])
      dq[2] = -(2.0*PI - fabs(q1[2]-q2[2]));
    else
      dq[2] = (2.0*PI - fabs(q1[2]-q2[2]));
  }

  return dq;
}



// *********************************************************************
// *********************************************************************
// CLASS:     GeomPQP2DRigidMulti
//
// *********************************************************************
// *********************************************************************

GeomPQP2DRigidMulti::GeomPQP2DRigidMulti(string path = ""):GeomPQP2DRigid(path) {

  // FilePath is set in the base class
  LoadEnvironment(FilePath);
  LoadRobot(FilePath);

  READ_OPTIONAL_PARAMETER(CollisionPairs);
}



void GeomPQP2DRigidMulti::LoadRobot(string path) {
  int i,j;
  string fname;
  list<MSLPolygon> pl;

  char* s = new char[50];

  // First check how many robot parts there are Robot0, Robot1, ...
  i = 0;
  sprintf(s,"%sRobot%d",FilePath.c_str(),i);
  while (is_file(s)) {
    i++;
    sprintf(s,"%sRobot%d",FilePath.c_str(),i);
  }

  NumBodies = i;

  if (NumBodies == 0)
    cout << "ERROR: No robot files at " << FilePath << "\n";

  Robot = vector<list<MSLTriangle> >(NumBodies);
  Ro = vector<PQP_Model>(NumBodies);

  for (i = 0; i < NumBodies; i++) {

    sprintf(s,"%sRobot%d",FilePath.c_str(),i);
    std::ifstream fin(s);
    pl.clear();
    fin >> pl;

    Robot[i] = PolygonsToTriangles(pl,5.0);
    j=0;
    list<MSLTriangle>::iterator t;

    Ro[i].BeginModel();
    PQP_REAL p1[3],p2[3],p3[3];
    forall(t,Robot[i]){
      p1[0] = (PQP_REAL) t->p1.xcoord();
      p1[1] = (PQP_REAL) t->p1.ycoord();
      p1[2] = (PQP_REAL) t->p1.zcoord();
      p2[0] = (PQP_REAL) t->p2.xcoord();
      p2[1] = (PQP_REAL) t->p2.ycoord();
      p2[2] = (PQP_REAL) t->p2.zcoord();
      p3[0] = (PQP_REAL) t->p3.xcoord();
      p3[1] = (PQP_REAL) t->p3.ycoord();
      p3[2] = (PQP_REAL) t->p3.zcoord();
      Ro[i].AddTri(p1,p2,p3,j);
      j++;
    }
    Ro[i].EndModel();
  }
}


bool GeomPQP2DRigidMulti::CollisionFree(const MSLVector &q){
  int i,j;
  list<MSLVector>::iterator v;

  PQP_CollideResult cres;
  SetTransformation(q);

  // Check for collisions with obstacles
  for (i = 0; i < NumBodies; i++) {
    PQP_Collide(&cres,RR[i],TR[i],&Ro[i],RO,TO,&Ob,PQP_FIRST_CONTACT);
    if (cres.NumPairs() >= 1)
      return false;
  }

  // Check for pairwise collisions
  forall(v,CollisionPairs) {
    i = (int) v->operator[](0);
    j = (int) v->operator[](1);
    PQP_Collide(&cres,RR[i],TR[i],&Ro[i],RR[j],TR[j],&Ro[j],PQP_FIRST_CONTACT);
    if (cres.NumPairs() >= 1)
      return false;
  }

  return true;
}


double GeomPQP2DRigidMulti::DistanceComp(const MSLVector &q){
  int i,j;
  list<MSLVector>::iterator v;
  double dist = INFINITY;

  PQP_DistanceResult dres;
  SetTransformation(q);

  // Check for collisions with obstacles
  for (i = 0; i < NumBodies; i++) {
    PQP_Distance(&dres,RR[i],TR[i],&Ro[i],RO,TO,&Ob,0.0,0.0);
    if (dres.Distance() < dist)
      dist = dres.Distance();
  }

  // Check for pairwise collisions
  forall(v,CollisionPairs) {
    i = (int) v->operator[](0);
    j = (int) v->operator[](1);
    PQP_Distance(&dres,RR[i],TR[i],&Ro[i],RR[j],TR[j],&Ro[j],0.0,0.0);
    if (dres.Distance() < dist)
      dist = dres.Distance();
  }

  return dist;
}


void GeomPQP2DRigidMulti::SetTransformation(const MSLVector &q){

  int i;
  MSLVector qi(3);

  for (i = 0; i < NumBodies; i++) {
    // Get the configuration
    qi[0] = q[i*3]; qi[1] = q[i*3+1];
    qi[2] = q[i*3+2];

    // Set translation
    TR[i][0]=(PQP_REAL)qi[0];
    TR[i][1]=(PQP_REAL)qi[1];
    TR[i][2]=0.0;

    // Set yaw rotation
    RR[i][0][0] = (PQP_REAL)(cos(qi[2]));
    RR[i][0][1] = (PQP_REAL)(-sin(qi[2]));
    RR[i][0][2] = 0.0;
    RR[i][1][0] = (PQP_REAL)(sin(qi[2]));
    RR[i][1][1] = (PQP_REAL)(cos(qi[2]));
    RR[i][1][2] = 0.0;
    RR[i][2][0] = 0.0;
    RR[i][2][1] = 0.0;
    RR[i][2][2] = 1.0;
  }
}



// *********************************************************************
// *********************************************************************
// CLASS:     GeomPQP3DRigid
//
// *********************************************************************
// *********************************************************************

GeomPQP3DRigid::GeomPQP3DRigid(string path = ""):GeomPQP(path) {

  LoadEnvironment(FilePath);
  LoadRobot(FilePath);

  // Compute the maximum deviates -- 2D with rotation
  MaxDeviates = MSLVector(6);
  MaxDeviates[0] = 1.0;
  MaxDeviates[1] = 1.0;
  MaxDeviates[2] = 1.0;
  double dmax1 = 0.0;
  double dmax2 = 0.0;
  double dmax3 = 0.0;
  double mag;

  list<MSLTriangle>::iterator tr;

  forall(tr,Robot) {
    // Check roll deviations
    mag = sqrt(sqr(tr->p1.ycoord())+sqr(tr->p1.zcoord()));
    if (mag > dmax1)
      dmax1 = mag;
    mag = sqrt(sqr(tr->p2.ycoord())+sqr(tr->p2.zcoord()));
    if (mag > dmax1)
      dmax1 = mag;
    mag = sqrt(sqr(tr->p3.ycoord())+sqr(tr->p3.zcoord()));
    if (mag > dmax1)
      dmax1 = mag;
    // Check pitch deviations
    mag = sqrt(sqr(tr->p1.xcoord())+sqr(tr->p1.zcoord()));
    if (mag > dmax2)
      dmax2 = mag;
    mag = sqrt(sqr(tr->p2.xcoord())+sqr(tr->p2.zcoord()));
    if (mag > dmax2)
      dmax2 = mag;
    mag = sqrt(sqr(tr->p3.xcoord())+sqr(tr->p3.zcoord()));
    if (mag > dmax2)
      dmax2 = mag;
    // Check yaw deviations
    mag = sqrt(sqr(tr->p1.xcoord())+sqr(tr->p1.ycoord()));
    if (mag > dmax3)
      dmax3 = mag;
    mag = sqrt(sqr(tr->p2.xcoord())+sqr(tr->p2.ycoord()));
    if (mag > dmax3)
      dmax3 = mag;
    mag = sqrt(sqr(tr->p3.xcoord())+sqr(tr->p3.ycoord()));
    if (mag > dmax3)
      dmax3 = mag;
  }

  MaxDeviates[3] = dmax1;
  MaxDeviates[4] = dmax2;
  MaxDeviates[5] = dmax3;

  //cout << "MD: " << MaxDeviates << "\n";
}


bool GeomPQP3DRigid::CollisionFree(const MSLVector &q){

  SetTransformation(q);


  PQP_CollideResult cres;
  PQP_Collide(&cres,RR,TR,&Ro,RO,TO,&Ob,PQP_FIRST_CONTACT);

  return (cres.NumPairs() == 0);
}




double GeomPQP3DRigid::DistanceComp(const MSLVector &q){

  SetTransformation(q);

  PQP_DistanceResult dres;
  PQP_Distance(&dres,RR,TR,&Ro,RO,TO,&Ob,0.0,0.0);

  return dres.Distance();
}




MSLVector GeomPQP3DRigid::ConfigurationDifference(const MSLVector &q1,
						  const MSLVector &q2)
{
  MSLVector dq(6);

  dq[0] = q2[0] - q1[0];
  dq[1] = q2[1] - q1[1];
  dq[2] = q2[2] - q1[2];

  if (fabs(q1[3]-q2[3]) < PI)
    dq[3] = q2[3] - q1[3];
  else {
    if (q1[3] < q2[3])
      dq[3] = -(2.0*PI - fabs(q1[3]-q2[3]));
    else
      dq[3] = (2.0*PI - fabs(q1[3]-q2[3]));
  }

  if (fabs(q1[4]-q2[4]) < PI)
    dq[4] = q2[4] - q1[4];
  else {
    if (q1[4] < q2[4])
      dq[4] = -(2.0*PI - fabs(q1[4]-q2[4]));
    else
      dq[4] = (2.0*PI - fabs(q1[4]-q2[4]));
  }

  if (fabs(q1[5]-q2[5]) < PI)
    dq[5] = q2[5] - q1[5];
  else {
    if (q1[5] < q2[5])
      dq[5] = -(2.0*PI - fabs(q1[5]-q2[5]));
    else
      dq[5] = (2.0*PI - fabs(q1[5]-q2[5]));
  }

  return dq;
}



void GeomPQP3DRigid::SetTransformation(const MSLVector &q){

  // Set translation
  TR[0]=(PQP_REAL)q[0];
  TR[1]=(PQP_REAL)q[1];
  TR[2]=(PQP_REAL)q[2];

  // Set rotation
  RR[0][0]=(PQP_REAL)(cos(q[5])*cos(q[4]));
  RR[0][1]=(PQP_REAL)(cos(q[5])*sin(q[4])*sin(q[3])-sin(q[5])*cos(q[3]));
  RR[0][2]=(PQP_REAL)(cos(q[5])*sin(q[4])*cos(q[3])+sin(q[5])*sin(q[3]));
  RR[1][0]=(PQP_REAL)(sin(q[5])*cos(q[4]));
  RR[1][1]=(PQP_REAL)(sin(q[5])*sin(q[4])*sin(q[3])+cos(q[5])*cos(q[3]));
  RR[1][2]=(PQP_REAL)(sin(q[5])*sin(q[4])*cos(q[3])-cos(q[5])*sin(q[3]));
  RR[2][0]=(PQP_REAL)((-1)*sin(q[4]));
  RR[2][1]=(PQP_REAL)(cos(q[4])*sin(q[3]));
  RR[2][2]=(PQP_REAL)(cos(q[4])*cos(q[3]));

}





// *********************************************************************
// *********************************************************************
// CLASS:     GeomPQP3DRigidMulti
//
// *********************************************************************
// *********************************************************************

GeomPQP3DRigidMulti::GeomPQP3DRigidMulti(string path = ""):GeomPQP3DRigid(path) {

  LoadEnvironment(FilePath);
  LoadRobot(FilePath);

  READ_OPTIONAL_PARAMETER(CollisionPairs);
}



void GeomPQP3DRigidMulti::LoadRobot(string path){
  int i,j;
  string fname;

  char* s = new char[50];

  // First check how many robot parts there are Robot0, Robot1, ...
  i = 0;
  sprintf(s,"%sRobot%d",FilePath.c_str(),i);
  while (is_file(s)) {
    i++;
    sprintf(s,"%sRobot%d",FilePath.c_str(),i);
  }

  NumBodies = i;

  if (NumBodies == 0)
    cout << "ERROR: No robot files at " << FilePath << "\n";

  Robot = vector<list<MSLTriangle> >(NumBodies);
  Ro = vector<PQP_Model>(NumBodies);

  for (i = 0; i < NumBodies; i++) {

    sprintf(s,"%sRobot%d",FilePath.c_str(),i);
    std::ifstream fin(s);
    fin >> Robot[i];

    j=0;
    list<MSLTriangle>::iterator t;

    Ro[i].BeginModel();
    PQP_REAL p1[3],p2[3],p3[3];
    forall(t,Robot[i]){
      p1[0] = (PQP_REAL) t->p1.xcoord();
      p1[1] = (PQP_REAL) t->p1.ycoord();
      p1[2] = (PQP_REAL) t->p1.zcoord();
      p2[0] = (PQP_REAL) t->p2.xcoord();
      p2[1] = (PQP_REAL) t->p2.ycoord();
      p2[2] = (PQP_REAL) t->p2.zcoord();
      p3[0] = (PQP_REAL) t->p3.xcoord();
      p3[1] = (PQP_REAL) t->p3.ycoord();
      p3[2] = (PQP_REAL) t->p3.zcoord();
      Ro[i].AddTri(p1,p2,p3,j);
      j++;
    }
    Ro[i].EndModel();
  }
}


bool GeomPQP3DRigidMulti::CollisionFree(const MSLVector &q){
  int i,j;
  list<MSLVector>::iterator v;

  PQP_CollideResult cres;
  SetTransformation(q);

  // Check for collisions with obstacles
  for (i = 0; i < NumBodies; i++) {
    PQP_Collide(&cres,RR[i],TR[i],&Ro[i],RO,TO,&Ob,PQP_FIRST_CONTACT);
    if (cres.NumPairs() >= 1)
      return false;
  }

  // Check for pairwise collisions
  forall(v,CollisionPairs) {
    i = (int) v->operator[](0);
    j = (int) v->operator[](1);
    PQP_Collide(&cres,RR[i],TR[i],&Ro[i],RR[j],TR[j],&Ro[j],PQP_FIRST_CONTACT);
    if (cres.NumPairs() >= 1)
      return false;
  }

  return true;
}


double GeomPQP3DRigidMulti::DistanceComp(const MSLVector &q){
  int i,j;
  list<MSLVector>::iterator v;
  double dist = INFINITY;

  PQP_DistanceResult dres;
  SetTransformation(q);

  // Check for collisions with obstacles
  for (i = 0; i < NumBodies; i++) {
    PQP_Distance(&dres,RR[i],TR[i],&Ro[i],RO,TO,&Ob,0.0,0.0);
    if (dres.Distance() < dist)
      dist = dres.Distance();
  }

  // Check for pairwise collisions
  forall(v,CollisionPairs) {
    i = (int) v->operator[](0);
    j = (int) v->operator[](1);
    PQP_Distance(&dres,RR[i],TR[i],&Ro[i],RR[j],TR[j],&Ro[j],0.0,0.0);
    if (dres.Distance() < dist)
      dist = dres.Distance();
  }

  return dist;
}


void GeomPQP3DRigidMulti::SetTransformation(const MSLVector &q){

  int i;
  MSLVector qi(6);

  for (i = 0; i < NumBodies; i++) {
    // Get the configuration
    qi[0] = q[i*6]; qi[1] = q[i*6+1]; qi[2] = q[i*6+2];
    qi[3] = q[i*6+3]; qi[4] = q[i*6+4]; qi[5] = q[i*6+5];

    // Set translation
    TR[i][0]=(PQP_REAL)qi[0];
    TR[i][1]=(PQP_REAL)qi[1];
    TR[i][2]=(PQP_REAL)qi[2];

    // Set rotation
    RR[i][0][0]=(PQP_REAL)(cos(qi[5])*cos(qi[4]));
    RR[i][0][1]=(PQP_REAL)(cos(qi[5])*sin(qi[4])*sin(qi[3])-sin(qi[5])*cos(qi[3]));
    RR[i][0][2]=(PQP_REAL)(cos(qi[5])*sin(qi[4])*cos(qi[3])+sin(qi[5])*sin(qi[3]));
    RR[i][1][0]=(PQP_REAL)(sin(qi[5])*cos(qi[4]));
    RR[i][1][1]=(PQP_REAL)(sin(qi[5])*sin(qi[4])*sin(qi[3])+cos(qi[5])*cos(qi[3]));
    RR[i][1][2]=(PQP_REAL)(sin(qi[5])*sin(qi[4])*cos(qi[3])-cos(qi[5])*sin(qi[3]));
    RR[i][2][0]=(PQP_REAL)((-1)*sin(qi[4]));
    RR[i][2][1]=(PQP_REAL)(cos(qi[4])*sin(qi[3]));
    RR[i][2][2]=(PQP_REAL)(cos(qi[4])*cos(qi[3]));
  }
}
