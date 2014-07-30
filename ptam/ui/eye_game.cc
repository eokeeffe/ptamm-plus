// Copyright 2008 Isis Innovation Limited
#include "ui/eye_game.h"
#include "ui/open_gl.h"
#include <cvd/convolution.h>
#include <stdlib.h>
#include <typeinfo>

using namespace CVD;
using namespace std;
using namespace TooN;

namespace ptam {
EyeGame::EyeGame() {
  mdEyeRadius = 0.1;
  mdShadowHalfSize = 2.5 * mdEyeRadius;
  mbInitialised = false;
}

void EyeGame::DrawStuff(TooN::Vector<3> v3CameraPos) {
  if (!mbInitialised)
    Init();

  mnFrameCounter ++;

  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);
  glFrontFace(GL_CW);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_NORMALIZE);
  glEnable(GL_COLOR_MATERIAL);

  GLfloat af[4];
  af[0]=0.5; af[1]=0.5; af[2]=0.5; af[3]=1.0;
  glLightfv(GL_LIGHT0, GL_AMBIENT, af);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, af);
  af[0]=1.0; af[1]=0.0; af[2]=1.0; af[3]=0.0;
  glLightfv(GL_LIGHT0, GL_POSITION, af);
  af[0]=1.0; af[1]=1.0; af[2]=1.0; af[3]=1.0;
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, af);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);

  glMatrixMode(GL_MODELVIEW);

  for (int i = 0; i < 4; i++) {
    if (mnFrameCounter < 100)
      LookAt(i, 500.0 * (TooN::Vector<3>) TooN::makeVector( (i<2?-1:1)*(mnFrameCounter < 50 ? -1 : 1) * -0.4 , -0.1, 1) , 0.05 );
    else
      LookAt(i, v3CameraPos, 0.02 );

    glLoadIdentity();
    glMultMatrix(ase3WorldFromEye[i]);
    glScaled(mdEyeRadius, mdEyeRadius, mdEyeRadius);
    glCallList(mnEyeDisplayList);
  }

  glDisable(GL_LIGHTING);

  glLoadIdentity();
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, mnShadowTex);
  glEnable(GL_BLEND);
  glColor4f(0, 0, 0, 0.5);
  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);
  glVertex2d(-mdShadowHalfSize, -mdShadowHalfSize);
  glTexCoord2f(0, 1);
  glVertex2d(-mdShadowHalfSize,  mdShadowHalfSize);
  glTexCoord2f(1, 1);
  glVertex2d( mdShadowHalfSize,  mdShadowHalfSize);
  glTexCoord2f(1, 0);
  glVertex2d( mdShadowHalfSize, -mdShadowHalfSize);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
}

void EyeGame::Reset() {
  for (int i = 0; i < 4; i++)
    ase3WorldFromEye[i] = TooN::SE3<>();

  ase3WorldFromEye[0].get_translation()[0] = -mdEyeRadius;
  ase3WorldFromEye[1].get_translation()[0] = mdEyeRadius;
  ase3WorldFromEye[2].get_translation()[0] = -mdEyeRadius;
  ase3WorldFromEye[3].get_translation()[0] = mdEyeRadius;

  ase3WorldFromEye[0].get_translation()[1] = -mdEyeRadius;
  ase3WorldFromEye[1].get_translation()[1] = -mdEyeRadius;
  ase3WorldFromEye[2].get_translation()[1] = mdEyeRadius;
  ase3WorldFromEye[3].get_translation()[1] = mdEyeRadius;

  ase3WorldFromEye[0].get_translation()[2] = mdEyeRadius;
  ase3WorldFromEye[1].get_translation()[2] = mdEyeRadius;
  ase3WorldFromEye[2].get_translation()[2] = mdEyeRadius;
  ase3WorldFromEye[3].get_translation()[2] = mdEyeRadius;
  mnFrameCounter = 0;
}

void EyeGame::DrawEye() {
  int nSegments = 45;
  int nSlices = 45;

  double dSliceAngle = M_PI / (double)(nSlices);
  double dSegAngle = 2.0 * M_PI / (double)(nSegments);

  glColor3f(0.0, 0.0, 0.0);
  {  // North pole:
    double Z = sin(M_PI/2.0 - dSliceAngle);
    double R = cos(M_PI/2.0 - dSliceAngle);
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0, 0, 1);
    glVertex3f(0, 0, 1);
    for (int i = 0; i < nSegments; i++) {
      glNormal3f(R * sin((double)i * dSegAngle), R * cos((double)i * dSegAngle),  Z);
      glVertex3f(R * sin((double)i * dSegAngle), R * cos((double)i * dSegAngle),  Z);
    }
    glNormal3f(0, R, Z);
    glVertex3f(0, R, Z);
    glEnd();
  }

  int nBlueSlice = 3;
  int nWhiteSlice = 6;
  for (int j = 1; j < nSlices; j++) {
    if (j == nBlueSlice)
      glColor3f(0, 0, 1);
    if (j == nWhiteSlice)
      glColor4d(0.92, 0.9, 0.85,1);

    glBegin(GL_QUAD_STRIP);
    double zTop = sin(M_PI/2.0 - dSliceAngle * (double)j);
    double zBot = sin(M_PI/2.0 - dSliceAngle * (double)(j+1));
    double rTop = cos(M_PI/2.0 - dSliceAngle * (double)j);
    double rBot = cos(M_PI/2.0 - dSliceAngle * (double)(j+1));
    for(int i=0; i<nSegments;i++)
    {
      glNormal3f(rTop*sin((double)i*dSegAngle), rTop*cos((double)i*dSegAngle), zTop);
      glVertex3f(rTop*sin((double)i*dSegAngle), rTop*cos((double)i*dSegAngle), zTop);
      glNormal3f(rBot*sin((double)i*dSegAngle), rBot*cos((double)i*dSegAngle), zBot);
      glVertex3f(rBot*sin((double)i*dSegAngle), rBot*cos((double)i*dSegAngle), zBot);
    };
    glNormal3f(0,rTop, zTop);
    glVertex3f(0,rTop, zTop);
    glNormal3f(0,rBot, zBot);
    glVertex3f(0,rBot, zBot);
    glEnd();
  };

  {
    // South pole:
    double Z = sin(M_PI/2.0 - dSliceAngle);
    double R = cos(M_PI/2.0 - dSliceAngle);
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0,0,-1);
    glVertex3f(0,0,-1);
    for(int i=0; i<nSegments;i++)
    {
      glNormal3f(R * sin((double)i * -dSegAngle), R * cos((double)i * -dSegAngle),  -Z);
      glVertex3f(R * sin((double)i * -dSegAngle), R * cos((double)i * -dSegAngle),  -Z);
    }
    glNormal3f(0,R,-Z);
    glVertex3f(0,R,-Z);
    glEnd();
  };
}

void EyeGame::Init() {
  if (mbInitialised) return;
  mbInitialised = true;
  // Set up the display list for the eyeball.
  mnEyeDisplayList = glGenLists(1);cout << "Test here in EyeGame 111" << endl;
  glNewList(mnEyeDisplayList,GL_COMPILE);cout << "Test here in EyeGame 2222" << endl;
  DrawEye();cout << "Test here in Init EyeGame 3333" << endl;
  glEndList();cout << "Test here in Init EyeGame 4444" << endl;
  MakeShadowTex();
}

void EyeGame::LookAt(int nEye, TooN::Vector<3> v3, double dRotLimit) {
  TooN::Vector<3> v3E = ase3WorldFromEye[nEye].inverse() * v3;
  if (v3E * v3E == 0.0)
    return;

  normalize(v3E);
  TooN::Matrix<3> m3Rot = TooN::Identity;
  m3Rot[2] = v3E;
  m3Rot[0] -= m3Rot[2]*(m3Rot[0]*m3Rot[2]);
  TooN::normalize(m3Rot[0].ref());
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];

  TooN::SO3<> so3Rotator = m3Rot;
  TooN::Vector<3> v3Log = so3Rotator.ln();
  v3Log[2] = 0.0;
  double dMagn = sqrt(v3Log * v3Log);
  if (dMagn > dRotLimit) {
    v3Log = v3Log * ( dRotLimit / dMagn);
  }
  ase3WorldFromEye[nEye].get_rotation() = ase3WorldFromEye[nEye].get_rotation() * TooN::SO3<>::exp(-v3Log);
}

void EyeGame::MakeShadowTex(){
  const int nTexSize = 256;cout << "Test here in MakeShadowTex EyeGame 44a" << endl;
  Image<byte> imShadow(ImageRef(nTexSize, nTexSize));cout << "Test here in MakeShadowTex EyeGame 4b" << endl;
  double dFrac = 1.0 - mdEyeRadius / mdShadowHalfSize;cout << "Test here in MakeShadowTex EyeGame 4c" << endl;
  double dCenterPos = dFrac * nTexSize / 2 - 0.5;cout << "Test here in MakeShadowTex EyeGame 4d" << endl;
  cout << "dCenterPos is: " << typeid(dCenterPos).name() << '\n';
  cout << "nTexSize is: " << typeid(nTexSize).name() << '\n';
  cout << "dFrac is: " << typeid(dFrac).name() << '\n';
  double a = 20.0;
  cout << "a is: " << typeid(a).name() << '\n';

  ImageRef irCenter; cout << "Test here in MakeShadowTex EyeGame 4e00:" << dCenterPos << endl;
  irCenter.x =irCenter.y = (int)  dCenterPos;cout << "Test here in MakeShadowTex EyeGame 4f" << endl;
  int nRadius = int ((nTexSize / 2 - irCenter.x) * 1.05);cout << "Test here in MakeShadowTex EyeGame 4g" << endl;
  unsigned int nRadiusSquared = nRadius*nRadius;cout << "Test here in MakeShadowTex EyeGame 5555" << endl;
  ImageRef ir;
  for(ir.y = 0; 2 * ir.y < nTexSize; ir.y++)
    for(ir.x = 0; 2 * ir.x < nTexSize; ir.x++)
    {
      byte val = 0;
      if((ir - irCenter).mag_squared() < nRadiusSquared)
        val = 255;
      imShadow[ir] = val;
      imShadow[ImageRef(nTexSize - 1 - ir.x, ir.y)] = val;
      imShadow[ImageRef(nTexSize - 1 - ir.x, nTexSize - 1 - ir.y)] = val;
      imShadow[ImageRef(ir.x, nTexSize - 1 - ir.y)] = val;
    }
  cout << "Test here in MakeShadowTex EyeGame 6666" << endl;
  convolveGaussian(imShadow, 4.0);
  glGenTextures(1, &mnShadowTex);cout << "Test here in MakeShadowTex EyeGame 777" << endl;
  glBindTexture(GL_TEXTURE_2D,mnShadowTex);cout << "Test here in MakeShadowTex EyeGame 888" << endl;
  glTexImage2D(GL_TEXTURE_2D, 0,
               GL_ALPHA, nTexSize, nTexSize, 0,
               GL_ALPHA, GL_UNSIGNED_BYTE, imShadow.data());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);cout << "Test here in MakeShadowTex EyeGame 999" << endl;
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}
}  // namespace ptam




