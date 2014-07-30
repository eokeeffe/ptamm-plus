// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef APPS_CALIBRATION_CAMERACALIBRATOR_H_
#define APPS_CALIBRATION_CAMERACALIBRATOR_H_
#include <vector>
#include <opencv2/core/core.hpp>
//#include <icg3d/sensor/videocapture_dispatch.h>
#include <gvars3/gvars3.h>
#include "track/calib_image.h"
#include "ui/gl_window2.h"

namespace ptam {
class CameraCalibrator {
public:
  CameraCalibrator();
  void Run();

  void Reset();
  void HandleFrame(CVD::Image<CVD::byte> imFrame);
  static void MainLoopCallback(void* pvUserData);
  void MainLoopStep();
  i3d::VideoCaptureDispatch<i3d::CameraFamily::PGR_FIREFLY_MV>* mVideoSource;

  cv::Ptr<GLWindow2> mGLWindow;
  ATANCamera mCamera;
  bool mbDone;

  std::vector<CalibImage> mvCalibImgs;
  void OptimizeOneStep();

  bool mbGrabNextFrame;
  GVars3::gvar3<int> mgvnOptimizing;
  GVars3::gvar3<int> mgvnShowImage;
  GVars3::gvar3<int> mgvnDisableDistortion;
  double mdMeanPixelError;

  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};
}  // namespace ptam
#endif
