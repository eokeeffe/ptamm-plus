// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef PTAM_TRACK_CALIB_IMAGE_H_
#define PTAM_TRACK_CALIB_IMAGE_H_
#include "math/atan_camera.h"
#include "track/calib_corner_patch.h"
#include <vector>
#include <TooN/se3.h>

namespace ptam {
const int N_NOT_TRIED = -1;
const int N_FAILED = -2;

struct CalibGridCorner {
  struct NeighborState {
    NeighborState() {val = N_NOT_TRIED;}
    int val;
  };

  CalibCornerPatch::Params Params;
  CVD::ImageRef irGridPos;
  NeighborState aNeighborStates[4];

  TooN::Matrix<2> GetSteps(std::vector<CalibGridCorner> &vgc);
  TooN::Matrix<2> mInheritedSteps;

  void Draw();

  double ExpansionPotential();
};

class CalibImage {
public:

  bool MakeFromImage(CVD::Image<CVD::byte> &im);
  TooN::SE3<> mse3CamFromWorld;
  void DrawImageGrid();
  void Draw3DGrid(ATANCamera &Camera, bool bDrawErrors);
  void GuessInitialPose(ATANCamera &Camera);

  struct ErrorAndJacobians {
    TooN::Vector<2> v2Error;
    TooN::Matrix<2,6> m26PoseJac;
    TooN::Matrix<2,NUMTRACKERCAMPARAMETERS> m2NCameraJac;
  };

  std::vector<ErrorAndJacobians> Project(ATANCamera &Camera);

  CVD::Image<CVD::byte> mim;

protected:
  std::vector<CVD::ImageRef> mvCorners;
  std::vector<CalibGridCorner> mvGridCorners;


  bool ExpandByAngle(int nSrc, int nDirn);
  int NextToExpand();
  void ExpandByStep(int n);
  CVD::ImageRef IR_from_dirn(int nDirn);

};
}  // namespace ptam
#endif

