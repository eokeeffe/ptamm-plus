// Copyright 2008 Isis Innovation Limited
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
#ifndef PTAM_TRACK_SYSTEM_H_
#define PTAM_TRACK_SYSTEM_H_

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

#include <icg3d/sensor/videocapture_dispatch.h>
#include "ui/gl_window2.h"
#include "construct/map.h"
#include "construct/map_maker.h"
#include "track/tracker.h"
#include "ui/ar_driver.h"
#include "ui/map_viewer.h"

namespace ptam {

class SlamSystem {
 public:
  SlamSystem();
  void Run();

  i3d::VideoCaptureDispatch<i3d::CameraFamily::PGR_FIREFLY_MV>* mVideoSource;
  boost::shared_ptr<GLWindow2> mGLWindow;
  CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;
  CVD::Image<CVD::byte> mimFrameBW;

  Map *mpMap;
  MapMaker *mpMapMaker;
  Tracker *mpTracker;
  ATANCamera *mpCamera;
  ARDriver *mpARDriver;
  MapViewer *mpMapViewer;

  bool mbDone;

  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};
}  // namespace ptam
#endif
