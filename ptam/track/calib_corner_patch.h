// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef PTAM_TRACK_CALIB_CORNER_PATCH_H_
#define PTAM_TRACK_CALIB_CORNER_PATCH_H_
#include <TooN/TooN.h>
#include <cvd/image.h>
#include <cvd/byte.h>

namespace ptam {
class CalibCornerPatch {
 public:
  struct Params {
    Params();
    TooN::Matrix<2> m2Warp();
    TooN::Vector<2> v2Pos;
    TooN::Vector<2> v2Angles;
    double dMean;
    double dGain;
  };

  CalibCornerPatch(int nSideSize = 8);
  bool IterateOnImage(Params &params, CVD::Image<CVD::byte> &im);
  bool IterateOnImageWithDrawing(Params &params, CVD::Image<CVD::byte> &im);

 protected:
  void MakeTemplateWithCurrentParams();
  void FillTemplate(CVD::Image<float> &im, Params params);
  double Iterate(CVD::Image<CVD::byte> &im);
  Params mParams;
  CVD::Image<float> mimTemplate;
  CVD::Image<TooN::Vector<2> > mimGradients;
  CVD::Image<TooN::Vector<2> > mimAngleJacs;

  void MakeSharedTemplate();
  static CVD::Image<float> mimSharedSourceTemplate;

  double mdLastError;
};
}  // namespace ptam
#endif
