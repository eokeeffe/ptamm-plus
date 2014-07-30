// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage - A small and blurry representation of an image.
// used by the relocaliser.

#ifndef PTAM_CONSTRUCT_SMALLBLURRYIMAGE_H_
#define PTAM_CONSTRUCT_SMALLBLURRYIMAGE_H_
#include <cvd/image.h>
#include <cvd/byte.h>
#include <TooN/se2.h>
#include <TooN/se3.h>
#include "math/atan_camera.h"
#include "construct/keyframe.h"

namespace ptam {
class SmallBlurryImage {
 public:
  SmallBlurryImage();
  SmallBlurryImage(KeyFrame &kf, double dBlur = 2.5);
  void MakeFromKF(KeyFrame &kf, double dBlur = 2.5);
  void MakeJacs();
  double ZMSSD(SmallBlurryImage &other);
  std::pair<TooN::SE2<>,double> IteratePosRelToTarget(SmallBlurryImage &other,
                                                int nIterations = 10);
  static TooN::SE3<> SE3fromSE2(TooN::SE2<> se2, ATANCamera camera);

 protected:
  CVD::Image<CVD::byte> mimSmall;
  CVD::Image<float> mimTemplate;
  CVD::Image<TooN::Vector<2> > mimImageJacs;
  bool mbMadeJacs;
  static CVD::ImageRef mirSize;
};
}  // namespace ptam
#endif









