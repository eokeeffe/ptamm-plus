// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// EyeGame.h
// Declares the EyeGame class
// EyeGame is a trivial AR app which draws some 3D graphics
// Draws a bunch of 3d eyeballs remniscient of the
// AVL logo
//
#ifndef PTAM_UI_EYEGAME_H_
#define PTAM_UI_EYEGAME_H_
#include <TooN/TooN.h>
#include "ui/open_gl.h"

namespace ptam {
class EyeGame {
 public:
  EyeGame();
  void DrawStuff(TooN::Vector<3> v3CameraPos);
  void Reset();
  void Init();


 protected:
  bool mbInitialised;
  void DrawEye();
  void LookAt(int nEye, TooN::Vector<3> v3, double dRotLimit);
  void MakeShadowTex();

  GLuint mnEyeDisplayList;
  GLuint mnShadowTex;
  double mdEyeRadius;
  double mdShadowHalfSize;
  TooN::SE3<> ase3WorldFromEye[4];
  int mnFrameCounter;
};
}  // namespace ptam
#endif
