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

#include "msl/model3d.h"
#include "msl/defs.h"


// *********************************************************************
// *********************************************************************
// CLASS:     Model3D
//
// *********************************************************************
// *********************************************************************


Model3D::Model3D(string path = ""):Model(path) {
}



// *********************************************************************
// *********************************************************************
// CLASS:     Model3DRigid
//
// *********************************************************************
// *********************************************************************


// Constructor
Model3DRigid::Model3DRigid(string path = ""):Model3D(path) {
  MSLVector u;

  StateDim = 6;
  InputDim = 6;

  READ_PARAMETER_OR_DEFAULT(LowerState,MSLVector(StateDim));
  READ_PARAMETER_OR_DEFAULT(UpperState,MSLVector(StateDim));

  // Make inputs
  Inputs.clear();
  u = MSLVector(6);
  u[0] = 1.0; u[1] = 0.0; u[2] = 0.0; u[3] = 0.0; u[4] = 0.0; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = -1.0; u[1] = 0.0; u[2] = 0.0; u[3] = 0.0; u[4] = 0.0; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = 1.0; u[2] = 0.0; u[3] = 0.0; u[4] = 0.0; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = -1.0; u[2] = 0.0; u[3] = 0.0; u[4] = 0.0; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = 0.0; u[2] = 1.0; u[3] = 0.0; u[4] = 0.0; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = 0.0; u[2] = -1.0; u[3] = 0.15; u[4] = 0.0; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = 0.0; u[2] = -1.0; u[3] = -0.15; u[4] = 0.0; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = 0.0; u[2] = 0.0; u[3] = 0.0; u[4] = 0.15; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = 0.0; u[2] = 0.0; u[3] = 0.0; u[4] = -0.15; u[5] = 0.0;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = 0.0; u[2] = 0.0; u[3] = 0.0; u[4] = 0.0; u[5] = 0.15;
  Inputs.push_back(u);

  u = MSLVector(6);
  u[0] = 0.0; u[1] = 0.0; u[2] = 0.0; u[3] = 0.0; u[4] = 0.0; u[5] = -0.15;
  Inputs.push_back(u);
}


MSLVector Model3DRigid::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx;

  dx = u;

  return dx;
}



double Model3DRigid::Metric(const MSLVector &x1, const MSLVector &x2) {

  double fd;

  fd = fabs(x1[3]-x2[3]);
  double dtheta1 = min(fd,2.0*PI - fd);
  fd = fabs(x1[4]-x2[4]);
  double dtheta2 = min(fd,2.0*PI - fd);
  fd = fabs(x1[5]-x2[5]);
  double dtheta3 = min(fd,2.0*PI - fd);

  return sqrt(sqr(x1[0] - x2[0]) + sqr(x1[1] - x2[1]) + sqr(x1[2] - x2[2]) +
  	      sqr(50.0/PI*dtheta1) + sqr(50.0/PI*dtheta2) +
  	      sqr(50.0/PI*dtheta3));
  //  return sqrt(sqr(x1[0] - x2[0]) + sqr(x1[1] - x2[1]) + sqr(x1[2] - x2[2]) +
  //	      sqr(dtheta1) + sqr(dtheta2) +
  //	      sqr(dtheta3));
}


MSLVector Model3DRigid::Integrate(const MSLVector &x, const MSLVector &u, const double &h)
{
  return EulerIntegrate(x,u,h);
}


MSLVector Model3DRigid::LinearInterpolate(const MSLVector &x1, const MSLVector &x2, const double &a){
  MSLVector v;

  v = (1.0-a)*x1 + a*x2;

  if (fabs(x2[3] - x1[3]) > PI) {
    if (x1[3] > x2[3])
      v[3] = (1.0-a)*x1[3] + a*(x2[3]+2.0*PI);
    else
      v[3] = (1.0-a)*(x1[3]+2.0*PI) + a*x2[3];
  }

  if (v[3] > PI)
    v[3] -= 2.0*PI;

  if (fabs(x2[4] - x1[4]) > PI) {
    if (x1[4] > x2[4])
      v[4] = (1.0-a)*x1[4] + a*(x2[4]+2.0*PI);
    else
      v[4] = (1.0-a)*(x1[4]+2.0*PI) + a*x2[4];
  }

  if (v[4] > PI)
    v[4] -= 2.0*PI;

  if (fabs(x2[5] - x1[5]) > PI) {
    if (x1[5] > x2[5])
      v[5] = (1.0-a)*x1[5] + a*(x2[5]+2.0*PI);
    else
      v[5] = (1.0-a)*(x1[5]+2.0*PI) + a*x2[5];
  }

  if (v[5] > PI)
    v[5] -= 2.0*PI;


  return v;

}





// *********************************************************************
// *********************************************************************
// CLASS:     Model3DRigidMulti
//
// *********************************************************************
// *********************************************************************


// Constructor
Model3DRigidMulti::Model3DRigidMulti(string path = ""):Model3DRigid(path) {
  MSLVector u;
  int i,j;

  READ_PARAMETER_OR_ERROR(NumBodies);

  StateDim = 6*NumBodies;
  InputDim = 6*NumBodies;

  READ_PARAMETER_OR_DEFAULT(LowerState,MSLVector(StateDim));
  READ_PARAMETER_OR_DEFAULT(UpperState,MSLVector(StateDim));

  u = MSLVector(StateDim);
  Inputs.clear();
  for (i = 0; i < StateDim; i++) {
    for (j = 0; j < StateDim; j++)
      u[j] = (i==j) ? 1.0 : 0.0;
    Inputs.push_back(u);
    for (j = 0; j < StateDim; j++)
      u[j] = (i==j) ? -1.0 : 0.0;
    Inputs.push_back(u);
  }
}




double Model3DRigidMulti::Metric(const MSLVector &x1, const MSLVector &x2) {

  double d,fd,dtheta1,dtheta2,dtheta3;
  int i;

  d = 0.0;

  for (i = 0; i < NumBodies; i++) {
    fd = fabs(x1[6*i+3]-x2[6*i+3]);
    dtheta1 = min(fd,2.0*PI - fd);
    fd = fabs(x1[6*i+4]-x2[6*i+4]);
    dtheta2 = min(fd,2.0*PI - fd);
    fd = fabs(x1[6*i+5]-x2[6*i+5]);
    dtheta3 = min(fd,2.0*PI - fd);
    d += sqr(x1[6*i] - x2[6*i]);
    d += sqr(x1[6*i+1] - x2[6*i+1]);
    d += sqr(x1[6*i+2] - x2[6*i+2]);
    d += sqr(dtheta1);
    d += sqr(dtheta2);
    d += sqr(dtheta3);
  }

  return sqrt(d);
}



MSLVector Model3DRigidMulti::LinearInterpolate(const MSLVector &x1, const MSLVector &x2, const double &a){
  MSLVector v;
  int i;

  v = (1.0-a)*x1 + a*x2;

  for (i = 0; i < NumBodies; i++) {
    if (fabs(x2[6*i+3] - x1[6*i+3]) > PI) {
      if (x1[6*i+3] > x2[6*i+3])
	v[6*i+3] = (1.0-a)*x1[6*i+3] + a*(x2[6*i+3]+2.0*PI);
      else
	v[6*i+3] = (1.0-a)*(x1[6*i+3]+2.0*PI) + a*x2[6*i+3];
    }

    if (v[6*i+3] > PI)
      v[6*i+3] -= 2.0*PI;

    if (fabs(x2[6*i+4] - x1[6*i+4]) > PI) {
      if (x1[6*i+4] > x2[6*i+4])
	v[6*i+4] = (1.0-a)*x1[6*i+4] + a*(x2[6*i+4]+2.0*PI);
      else
	v[6*i+4] = (1.0-a)*(x1[6*i+4]+2.0*PI) + a*x2[6*i+4];
    }

    if (v[6*i+4] > PI)
      v[6*i+4] -= 2.0*PI;

    if (fabs(x2[6*i+5] - x1[6*i+5]) > PI) {
      if (x1[6*i+5] > x2[6*i+5])
	v[6*i+5] = (1.0-a)*x1[6*i+5] + a*(x2[6*i+5]+2.0*PI);
      else
	v[6*i+5] = (1.0-a)*(x1[6*i+5]+2.0*PI) + a*x2[6*i+5];
    }

    if (v[6*i+5] > PI)
      v[6*i+5] -= 2.0*PI;
  }

  return v;

}




// *********************************************************************
// *********************************************************************
// CLASS:     Model3DRigidChain
// A 3D kinematic chain of bodies
//
// *********************************************************************
// *********************************************************************

Model3DRigidChain::Model3DRigidChain(string path = ""):Model3DRigid(path) {

  int i;
  MSLVector u;
  std::ifstream fin;

  READ_PARAMETER_OR_ERROR(NumBodies);
  READ_PARAMETER_OR_ERROR(StateDim);
  READ_PARAMETER_OR_ERROR(DH);

  InputDim = StateDim;

  StateIndices = vector<int>(StateDim);
  fin.open((FilePath + "StateIndices").c_str());
  if (fin) {
    for (i = 0; i < StateDim; i++) {
      fin >> StateIndices[i];
    }
  }
  else {
    cout << "Error: No StateIndices file was found\n";
    exit(-1);
  }
  fin.close();

  LowerState = MSLVector(StateDim);
  UpperState = MSLVector(StateDim);

  for (i = 0; i < StateDim; i++)
    LowerState[i] = -1.0;
  for (i = 0; i < StateDim; i++)
    UpperState[i] = 1.0;

  READ_OPTIONAL_PARAMETER(LowerState);
  READ_OPTIONAL_PARAMETER(UpperState);

  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  for (i = 0; i < StateDim; i++) {
    u = MSLVector(StateDim);
    u[i] = 0.1;
    Inputs.push_back(u);
    u = MSLVector(StateDim);
    u[i] = -0.1;
    Inputs.push_back(u);
  }

  READ_OPTIONAL_PARAMETER(Inputs);
}



MSLVector Model3DRigidChain::StateToConfiguration(const MSLVector &x) {
  MSLVector q;
  MSLVector A, Alpha, D, Theta;
  int i;
  MSLMatrix r(4,4), rn(4,4), ro(4,4);

  q = MSLVector(6*NumBodies);
  A = MSLVector(NumBodies);
  D = MSLVector(NumBodies);
  Alpha = MSLVector(NumBodies);
  Theta = MSLVector(NumBodies);

  for (i = 0; i < StateDim; i++ ) {
    if (StateIndices[i] != 0) {
      int y = StateIndices[i];
      DH[y-1] = x[i];
    }
  }

  for (i = 0; i < NumBodies; i++) {
       Alpha[i] = DH[i];
       Theta[i] = DH[NumBodies*1+i];
       A[i] = DH[NumBodies*2+i];
       D[i] = DH[NumBodies*3+i];
  }

  for (i = 0; i < 4 ; i ++ ) {
    for (int j = 0 ; j < 4 ; j ++ ) {
      if (i==j) {r[i][j] = 1.0;}
      else {r[i][j] = 0.0;}
    }
  }

  for (i = 0; i < NumBodies; i++) {
    ro=r;
    rn[0][0] = cos(Theta[i]);
    rn[0][1] = -sin(Theta[i]);
    rn[0][2] = 0.0;
    rn[0][3] = A[i];
    rn[1][2] = -sin(Alpha[i]);
    rn[2][2] = cos(Alpha[i]);
    rn[3][2] = 0.0;
    rn[1][3] = rn[1][2]*D[i];
    rn[2][3] = rn[2][2]*D[i];
    rn[3][3] = 1.0;
    rn[1][0] = -rn[0][1]*rn[2][2];
    rn[2][0] = -rn[0][1]*(-rn[1][2]);
    rn[3][0] = 0.0;
    rn[1][1] = rn[0][0]*rn[2][2];
    rn[2][1] = rn[0][0]*(-rn[1][2]);
    rn[3][1] = 0.0;

    //R = R*Rnew
    for (int k=0; k<4; k++){
      for (int l=0; l<4; l++) {
	r[k][l]=0.0 ;
	for (int m=0; m<4; m++){
	  r[k][l]+=ro[k][m]*rn[m][l];
	}
      }
    }

    q[6*i] = r[0][3];
    q[6*i+1] = r[1][3];
    q[6*i+2] = r[2][3];

    q[6*i+4] = atan2(  -r[2][0], sqrt(sqr(r[0][0])+sqr(r[1][0]))  );
    if ( (q[6*i+4]>0.0) && (cos(q[6*i+4])==0.0) ) {
      q[6*i+5] = 0.0;
      q[6*i+3] = atan2(r[0][1], r[1][1]);
    }
    else {
      if ( (q[6*i+4]<0.0) && (cos(q[6*i+4])==0.0) ) {
	q[6*i+5] = 0.0;
	q[6*i+3] = -atan2(r[0][1], r[1][1]);
      }
      else {
	q[6*i+5] = atan2(  r[1][0]/cos(q[6*i+4]), r[0][0]/cos(q[6*i+4])  );
	q[6*i+3] = atan2(  r[2][1]/cos(q[6*i+4]), r[2][2]/cos(q[6*i+4])  );
      }
    }
    if (q[6*i+3] < 0.0)
      q[6*i+3] += 2.0*PI; // Force the orientation into [0,2pi)
    if (q[6*i+4] < 0.0)
      q[6*i+4] += 2.0*PI; // Force the orientation into [0,2pi)
    if (q[6*i+5] < 0.0)
      q[6*i+5] += 2.0*PI; // Force the orientation into [0,2pi)
  }

  return q;
}



MSLVector Model3DRigidChain::LinearInterpolate(const MSLVector &x1,
					    const MSLVector &x2,
					    const double &a) {
  return (1.0-a)*x1 + a*x2;
}


MSLVector Model3DRigidChain::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(StateDim);

  dx = u;
  return dx;
}



double Model3DRigidChain::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho;
  MSLVector dtheta(StateDim);
  int i;

  rho = 0.0;
  for ( i= 0 ; i < StateDim; i++){
    if (StateIndices[i] > 2*NumBodies) {
      rho += sqr(x1[i] - x2[i]);
    }
    else {
      dtheta[i] = min(fabs(x1[i]-x2[i]),2.0*PI - fabs(x1[i]-x2[i]));
      rho += sqr(10.0/PI*dtheta[i]);
    }
  }
  rho = sqrt(rho);

  return rho;
}


// Make sure the joint position and limits are respected
bool Model3DRigidChain::Satisfied(const MSLVector &x)
{
  int i;

  for (i = 0; i < StateDim; i++)
    if ((x[i] > UpperState[i]) || (x[i] < LowerState[i]))
      return false;

  return true;
}


// *********************************************************************
// *********************************************************************
// CLASS:     Model3DRigidChainTree
// A 3D kinematic tree of bodies
//
// *********************************************************************
// *********************************************************************

Model3DRigidTree::Model3DRigidTree(string path = ""):Model3DRigid(path) {

  int i;
  MSLVector u;
  std::ifstream fin;

  READ_PARAMETER_OR_ERROR(NumBodies);
  READ_PARAMETER_OR_ERROR(StateDim);
  READ_PARAMETER_OR_ERROR(DH);

  InputDim = StateDim;

  StateIndices = vector<int>(StateDim);
  fin.open((FilePath + "StateIndices").c_str());
  if (fin) {
    for (i = 0; i < StateDim; i++) {
      fin >> StateIndices[i];
    }
  }
  else {
    cout << "Error: No StateIndices file was found\n";
    exit(-1);
  }
  fin.close();


  Parents = vector<int>(NumBodies);
  fin.open((FilePath + "Parents").c_str());
  if (fin) {
    for (i = 0; i < NumBodies; i++) {
      fin >> Parents[i];
    }
  }
  else {
    cout << "Error: No Parents file was found\n";
    exit(-1);
  }
  fin.close();


  for (i = 1; i < NumBodies; i++) {
    if (Parents[i] >= i) {
      cout << "There is a mistake in Parents\n";
      exit(-1);
    }
  }

  LowerState = MSLVector(StateDim);
  UpperState = MSLVector(StateDim);

  READ_OPTIONAL_PARAMETER(LowerState);
  READ_OPTIONAL_PARAMETER(UpperState);

  Inputs.clear();  // Otherwise its parent constructor will make some inputs
  for (i = 0; i < StateDim; i++) {
    u = MSLVector(StateDim);
    u[i] = 0.1;
    Inputs.push_back(u);
    u = MSLVector(StateDim);
    u[i] = -0.1;
    Inputs.push_back(u);
  }

  READ_OPTIONAL_PARAMETER(Inputs);
}



MSLVector Model3DRigidTree::StateToConfiguration(const MSLVector &x) {
  MSLVector q;
  MSLVector A, Alpha, D, Theta;
  int i;
  MSLMatrix r(4,4), rn(4,4), ro(4,4);

//  MSLMatrix Tr[NumBodies];
  MSLMatrix Tr[10]; // Temperarily, set the NumBodies=10 to make compiler happy

  q = MSLVector(6*NumBodies);
  A = MSLVector(NumBodies);
  D = MSLVector(NumBodies);
  Alpha = MSLVector(NumBodies);
  Theta = MSLVector(NumBodies);

  for (i = 0; i < StateDim; i++ ) {
    if (StateIndices[i] != 0) {
      int y = StateIndices[i];
      DH[y-1] = x[i];
    }
  }

  for (i = 0; i < NumBodies; i++){
       Alpha[i] = DH[i];
       Theta[i] = DH[NumBodies*1+i];
       A[i] = DH[NumBodies*2+i];
       D[i] = DH[NumBodies*3+i];
  }

  for (i = 0; i < 4 ; i ++ ) {
    for (int j = 0 ; j < 4 ; j ++ ) {
      if (i==j) {
	r[i][j] = 1.0;
      }
      else {
	r[i][j] = 0.0;
      }
    }
  }
  ro = r;

  for (i = 0; i < NumBodies; i++ ) {

    int ParInd = Parents[i];

    rn[0][0] = cos(Theta[i]);
    rn[0][1] = -sin(Theta[i]);
    rn[0][2] = 0.0;
    rn[0][3] = A[i];
    rn[1][2] = -sin(Alpha[i]);
    rn[2][2] = cos(Alpha[i]);
    rn[3][2] = 0.0;
    rn[1][3] = rn[1][2]*D[i];
    rn[2][3] = rn[2][2]*D[i];
    rn[3][3] = 1.0;
    rn[1][0] = -rn[0][1]*rn[2][2];
    rn[2][0] = -rn[0][1]*(-rn[1][2]);
    rn[3][0] = 0.0;
    rn[1][1] = rn[0][0]*rn[2][2];
    rn[2][1] = rn[0][0]*(-rn[1][2]);
    rn[3][1] = 0.0;

    //R = R*Rnew

    if (i != 0) { ro = Tr[ParInd];}

    for (int k=0; k<4; k++){
      for (int l=0; l<4; l++) {
	r[k][l]=0.0 ;
	for (int m=0; m<4; m++){
	    r[k][l]+=ro[k][m]*rn[m][l];

	}
      }
    }

    Tr[i] = r;

    //Finding State coordinates

    q[6*i] = r[0][3];
    q[6*i+1] = r[1][3];
    q[6*i+2] = r[2][3];

    q[6*i+4] = atan2(  -r[2][0], sqrt(sqr(r[0][0])+sqr(r[1][0]))  );
    if ( (q[6*i+4]>0.0) && (cos(q[6*i+4])==0.0) ) {
      q[6*i+5] = 0.0;
      q[6*i+3] = atan2(r[0][1], r[1][1]);
    }
    else{
      if ( (q[6*i+4]<0.0) && (cos(q[6*i+4])==0.0) ) {
	q[6*i+5] = 0.0;
	q[6*i+3] = -atan2(r[0][1], r[1][1]);
      }
      else {
	q[6*i+5] = atan2(  r[1][0]/cos(q[6*i+4]), r[0][0]/cos(q[6*i+4])  );
	q[6*i+3] = atan2(  r[2][1]/cos(q[6*i+4]), r[2][2]/cos(q[6*i+4])  );
      }
    }
    if (q[6*i+3] < 0.0)
      q[6*i+3] += 2.0*PI; // Force the orientation into [0,2pi)
    if (q[6*i+4] < 0.0)
      q[6*i+4] += 2.0*PI; // Force the orientation into [0,2pi)
    if (q[6*i+5] < 0.0)
      q[6*i+5] += 2.0*PI; // Force the orientation into [0,2pi)
  }
  return q;
}



MSLVector Model3DRigidTree::LinearInterpolate(const MSLVector &x1,
					    const MSLVector &x2,
					    const double &a) {
  return (1.0-a)*x1 + a*x2;
}


MSLVector Model3DRigidTree::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {

  MSLVector dx(StateDim);

  dx = u;
  return dx;
}



double Model3DRigidTree::Metric(const MSLVector &x1, const MSLVector &x2) {

  double rho;
  MSLVector dtheta(StateDim);
  int i;

  rho = 0.0;
  for (i = 0 ; i < StateDim; i++){
    if (StateIndices[i] > 2*NumBodies) {
      rho += sqr(x1[i] - x2[i]);
    }
    else {
      dtheta[i] = min(fabs(x1[i]-x2[i]),2.0*PI - fabs(x1[i]-x2[i]));
      rho += sqr(10.0/PI*dtheta[i]);
    }
  }
  rho = sqrt(rho);

  return rho;
}


// Make sure the joint position and limits are respected
bool Model3DRigidTree::Satisfied(const MSLVector &x)
{
  int i;

  for (i = 0; i < StateDim; i++)
    if ((x[i] > UpperState[i]) || (x[i] < LowerState[i]))
      return false;

  return true;
}
