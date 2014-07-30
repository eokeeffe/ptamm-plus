// Copyright 2008 Isis Innovation Limited
#include "slam_system.h"

#include <time.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <gvars3/instances.h>

#include "construct/map_maker.h"
#include "math/atan_camera.h"
#include "track/tracker.h"
#include "ui/ar_driver.h"
#include "ui/map_viewer.h"
#include "ui/open_gl.h"

using namespace CVD;
using namespace std;

namespace ptam {
SlamSystem::SlamSystem() {
  GVars3::GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GVars3::GUI.RegisterCommand("quit", GUICommandCallBack, this);

  // First, check if the camera is calibrated.
  // If not, we need to run the calibration widget.
  TooN::Vector<NUMTRACKERCAMPARAMETERS> vTest;

  vTest = GVars3::GV3::get<TooN::Vector<NUMTRACKERCAMPARAMETERS> >(
        "Camera.Parameters", ATANCamera::mvDefaultParams, GVars3::HIDDEN);
  mpCamera = new ATANCamera("Camera");
  if (vTest == ATANCamera::mvDefaultParams) {
    printf("! Camera.Parameters is not set, need to run the CameraCalibrator tool\n");
    printf("  and/or put the Camera.Parameters= line into the appropriate .cfg file.\n");
    exit(1);
  }

  TooN::Vector<2> v2 = mpCamera->GetImageSize();
  CVD::ImageRef img_size(v2[0], v2[1]);
  mimFrameBW.resize(img_size);
  mimFrameRGB.resize(img_size);

  mGLWindow = boost::shared_ptr<GLWindow2>(new GLWindow2(img_size, "PTAM"));

  mpMap = new Map;
  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
  mpTracker = new Tracker(img_size, *mpCamera, *mpMap, *mpMapMaker);
  mpARDriver = new ARDriver(*mpCamera, img_size, *mGLWindow.get());
  mpMapViewer = new MapViewer(*mpMap, *mGLWindow.get());

  GVars3::GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GVars3::GUI.ParseLine("Menu.ShowMenu Root");
  GVars3::GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GVars3::GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GVars3::GUI.ParseLine("DrawAR=0");
  GVars3::GUI.ParseLine("DrawMap=0");
  GVars3::GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  GVars3::GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

  mbDone = false;
}

void SlamSystem::Run() {
  cv::Mat rgb_frame, gray_frame;
  while (!mbDone) {
    timeval tim;
    gettimeofday(&tim, NULL);
    double start = tim.tv_sec + (tim.tv_usec / 1000000.0);

    // We use two versions of each video frame:
    // One black and white (for processing by the tracker etc)
    // and one RGB, for drawing.

    // Grab new video frame...
    rgb_frame = mVideoSource->grabFrame();
    cv::resize(rgb_frame, rgb_frame, cv::Size(640, 480));
    CVD::SubImage<CVD::Rgb<CVD::byte> > cvd_rgb_frame(
          (CVD::Rgb<CVD::byte>*)rgb_frame.data,
          CVD::ImageRef(rgb_frame.cols, rgb_frame.rows), rgb_frame.cols);
    cv::cvtColor(rgb_frame, gray_frame, CV_RGB2GRAY);
    mimFrameRGB.copy_from(cvd_rgb_frame);
    CVD::SubImage<CVD::byte> cvd_gray_frame(gray_frame.data,
          CVD::ImageRef(gray_frame.cols, gray_frame.rows), rgb_frame.cols);
    mimFrameBW.copy_from(cvd_gray_frame);
    static bool bFirstFrame = true;
    if (bFirstFrame) {
      mpARDriver->Init();
      bFirstFrame = false;
    }

    mGLWindow->SetupViewport();
    mGLWindow->SetupVideoOrtho();
    mGLWindow->SetupVideoRasterPosAndZoom();

    if(!mpMap->IsGood())
      mpARDriver->Reset();

    static GVars3::gvar3<int> gvnDrawMap("DrawMap", 0, GVars3::HIDDEN | GVars3::SILENT);
    static GVars3::gvar3<int> gvnDrawAR("DrawAR", 0, GVars3::HIDDEN | GVars3::SILENT);

    bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
    bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;

    mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap);

    if (bDrawMap)
      mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
    else if(bDrawAR)
      mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose());

    //      mGLWindow->GetMousePoseUpdate();
    string sCaption;
    if (bDrawMap)
      sCaption = mpMapViewer->GetMessageForUser();
    else
      sCaption = mpTracker->GetMessageForUser();
    mGLWindow->DrawCaption(sCaption);
    mGLWindow->DrawMenus();
    mGLWindow->swap_buffers();
    mGLWindow->HandlePendingEvents();


    gettimeofday(&tim, NULL);
    double end = tim.tv_sec + (tim.tv_usec / 1000000.0);
//    printf("Test time spend in SlamSystem: %f\n",(end-start));
//    printf(" Test  fps in SlamSystem: %f\n",1.0/(end-start));
  }
}

void SlamSystem::GUICommandCallBack(void *ptr, string sCommand, string sParams) {
  if (sCommand == "quit" || sCommand == "exit")
    static_cast<SlamSystem*>(ptr)->mbDone = true;
}
}  // namespace ptam







