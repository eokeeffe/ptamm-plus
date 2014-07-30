// Copyright 2008 Isis Innovation Limited
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <gvars3/instances.h>
#include <TooN/SVD.h>
#include "ui/open_gl.h"
#include "camera_calibrator.h"
#include "config.h"

using namespace std;
using namespace i3d;
using namespace ptam;

int main(int argc, char **argv) {
  printf("PTAM CameraCalibrator \n");
  printf("  --------------- \n");
  printf("  Parsing calibrator_settings.cfg from: %s\n", CONFIG_FILE);
  string configfile = (argc > 1) ? argv[1] : CONFIG_FILE;
  GVars3::GUI.LoadFile(configfile);
  GVars3::GUI.StartParserThread();
  atexit(GVars3::GUI.StopParserThread); // Clean up readline when program quits

  VideoCaptureDispatch<CameraFamily::PGR_FIREFLY_MV> videodispatch;
  string vsource = GVars3::GV3::get<string>("Video.source");
  if (vsource.length() < 3) {
    if(!videodispatch.open(atoi(vsource.c_str())))
      return -1;
  } else {
    if(!videodispatch.open(vsource))
      return -1;
  }
  videodispatch.setProperty("Brightness", GVars3::GV3::get<int>("Video.brightness", 50, GVars3::SILENT));
  videodispatch.setProperty("Exposure", GVars3::GV3::get<int>("Video.exposure", 40, GVars3::SILENT));
  videodispatch.setProperty("Gain", GVars3::GV3::get<int>("Video.gain", 40, GVars3::SILENT));
  videodispatch.setProperty("Shutter", GVars3::GV3::get<int>("Video.shutter", 530, GVars3::SILENT));


  GVars3::GV3::get<TooN::Vector<NUMTRACKERCAMPARAMETERS> >(
        "Camera.Parameters", ATANCamera::mvDefaultParams, GVars3::SILENT);

  try {
    CameraCalibrator c;
    c.mVideoSource = &videodispatch;
    c.Run();
  } catch(CVD::Exceptions::All e) {
    printf("Oops! camera_calibrator.cc main()::%s\n", e.what.c_str());
  }
}

namespace ptam {
CameraCalibrator::CameraCalibrator()
  : mCamera("Camera") {
  mbDone = false;

  TooN::Vector<2> v2 = mCamera.GetImageSize();
  CVD::ImageRef img_size(v2[0], v2[1]);

  mGLWindow = cv::Ptr<GLWindow2>(new GLWindow2(img_size, "Camera Calibrator"));

  GVars3::GUI.RegisterCommand("CameraCalibrator.GrabNextFrame", GUICommandCallBack, this);
  GVars3::GUI.RegisterCommand("CameraCalibrator.Reset", GUICommandCallBack, this);
  GVars3::GUI.RegisterCommand("CameraCalibrator.ShowNext", GUICommandCallBack, this);
  GVars3::GUI.RegisterCommand("CameraCalibrator.SaveCalib", GUICommandCallBack, this);
  GVars3::GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GVars3::GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GVars3::GV3::Register(mgvnOptimizing, "CameraCalibrator.Optimize", 0, GVars3::SILENT);
  GVars3::GV3::Register(mgvnShowImage, "CameraCalibrator.Show", 0, GVars3::SILENT);
  GVars3::GV3::Register(mgvnDisableDistortion, "CameraCalibrator.NoDistortion", 0, GVars3::SILENT);
  GVars3::GUI.ParseLine("GLWindow.AddMenu CalibMenu");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuButton Live GrabFrame CameraCalibrator.GrabNextFrame");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuButton Live Reset CameraCalibrator.Reset");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuButton Live Optimize \"CameraCalibrator.Optimize=1\"");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuToggle Live NoDist CameraCalibrator.NoDistortion");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuSlider Opti \"Show Img\" CameraCalibrator.Show 0 10");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Show Next\" CameraCalibrator.ShowNext");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Grab More\" CameraCalibrator.Optimize=0 ");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuButton Opti Reset CameraCalibrator.Reset");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuToggle Opti NoDist CameraCalibrator.NoDistortion");
  GVars3::GUI.ParseLine("CalibMenu.AddMenuButton Opti Save CameraCalibrator.SaveCalib");

  Reset();
}

void CameraCalibrator::Run() {
  cv::Mat rgb_frame, gray_frame;
  while (!mbDone) {
    // We use two versions of each video frame:
    // One black and white (for processing by the tracker etc)
    // and one RGB, for drawing.

    TooN::Vector<2> v2 = mCamera.GetImageSize();
    CVD::ImageRef img_size(v2[0], v2[1]);
    CVD::Image<CVD::Rgb<CVD::byte> > imFrameRGB(img_size);
    CVD::Image<CVD::byte>  imFrameBW(img_size);

    // Grab new video frame...
    rgb_frame = mVideoSource->grabFrame();
    cv::resize(rgb_frame, rgb_frame, cv::Size(v2[0], v2[1]));
    CVD::SubImage<CVD::Rgb<CVD::byte> > cvd_rgb_frame(
          (CVD::Rgb<CVD::byte>*)rgb_frame.data,
          CVD::ImageRef(rgb_frame.cols, rgb_frame.rows), rgb_frame.cols*3);
    cv::cvtColor(rgb_frame, gray_frame, CV_RGB2GRAY);
    imFrameRGB.copy_from(cvd_rgb_frame);
    CVD::SubImage<CVD::byte> cvd_gray_frame(gray_frame.data,
          CVD::ImageRef(gray_frame.cols, gray_frame.rows), rgb_frame.cols);
    imFrameBW.copy_from(cvd_gray_frame);

    // Set up openGL
    mGLWindow->SetupViewport();
    mGLWindow->SetupVideoOrtho();
    mGLWindow->SetupVideoRasterPosAndZoom();

    if (mvCalibImgs.size() < 1)
      *mgvnOptimizing = 0;

    if (!*mgvnOptimizing) {
      GVars3::GUI.ParseLine("CalibMenu.ShowMenu Live");
      CVD::glDrawPixels(imFrameBW);

      CalibImage c;
      if (c.MakeFromImage(imFrameBW)) {
        if (mbGrabNextFrame) {
          mvCalibImgs.push_back(c);
          mvCalibImgs.back().GuessInitialPose(mCamera);
          mvCalibImgs.back().Draw3DGrid(mCamera, false);
          mbGrabNextFrame = false;
        };
      }
    } else {
      OptimizeOneStep();

      GVars3::GUI.ParseLine("CalibMenu.ShowMenu Opti");
      int nToShow = *mgvnShowImage - 1;
      if(nToShow < 0)
        nToShow = 0;
      if(nToShow >= (int) mvCalibImgs.size())
        nToShow = mvCalibImgs.size()-1;
      *mgvnShowImage = nToShow + 1;

      CVD::glDrawPixels(mvCalibImgs[nToShow].mim);
      mvCalibImgs[nToShow].Draw3DGrid(mCamera,true);
    }

    ostringstream ost;
    ost << "Camera Calibration: Grabbed " << mvCalibImgs.size() << " images." << endl;
    if (!*mgvnOptimizing) {
      ost << "Take snapshots of the calib grid with the \"GrabFrame\" button," << endl;
      ost << "and then press \"Optimize\"." << endl;
      ost << "Take enough shots (4+) at different angles to get points " << endl;
      ost << "into all parts of the image (corners too.) The whole grid " << endl;
      ost << "doesn't need to be visible so feel free to zoom in." << endl;
    } else {
      ost << "Current RMS pixel error is " << mdMeanPixelError << endl;
      ost << "Current camera params are  " << GVars3::GV3::get_var("Camera.Parameters") << endl;
      ost << "(That would be a pixel aspect ratio of "
          <<  mCamera.PixelAspectRatio() << ")" << endl;
      ost << "Check fit by looking through the grabbed images." << endl;
      ost << "RMS should go below 0.5, typically below 0.3 for a wide lens." << endl;
      ost << "Press \"save\" to save calibration to camera.cfg file and exit." << endl;
    }

    mGLWindow->DrawCaption(ost.str());
    mGLWindow->DrawMenus();
    mGLWindow->HandlePendingEvents();
    mGLWindow->swap_buffers();
  }
}

void CameraCalibrator::Reset() {
  *mCamera.mgvvCameraParams = ATANCamera::mvDefaultParams;
  if(*mgvnDisableDistortion) mCamera.DisableRadialDistortion();
  TooN::Vector<2> v2 = mCamera.GetImageSize();
  mCamera.SetImageSize(CVD::ImageRef(v2[0], v2[1]));
  mbGrabNextFrame = false;
  *mgvnOptimizing = false;
  mvCalibImgs.clear();
}

void CameraCalibrator::GUICommandCallBack(void* ptr, string sCommand, string sParams) {
  ((CameraCalibrator*) ptr)->GUICommandHandler(sCommand, sParams);
}

void CameraCalibrator::GUICommandHandler(string sCommand, string sParams) {  // Called by the callback func..
  if (sCommand == "CameraCalibrator.Reset") {
    Reset();
    return;
  }
  if (sCommand == "CameraCalibrator.GrabNextFrame") {
    mbGrabNextFrame = true;
    return;
  }
  if (sCommand == "CameraCalibrator.ShowNext") {
    int nToShow = (*mgvnShowImage - 1 + 1) % mvCalibImgs.size();
    *mgvnShowImage = nToShow + 1;
    return;
  }
  if (sCommand == "CameraCalibrator.SaveCalib") {
    cout << "  Camera calib is " << GVars3::GV3::get_var("Camera.Parameters") << endl;
    cout << "  Saving camera calib to camera.cfg..." << endl;
    ofstream ofs("camera.cfg");
    if (ofs.good()) {
      GVars3::GV2.PrintVar("Camera.Parameters", ofs);
      ofs.close();
      cout << "  .. saved."<< endl;
    } else {
      cout <<"! Could not open camera.cfg for writing." << endl;
      GVars3::GV2.PrintVar("Camera.Parameters", cout);
      cout <<"  Copy-paste above line to settings.cfg or camera.cfg! " << endl;
    }
    mbDone = true;
  }
  if (sCommand == "exit" || sCommand == "quit") {
    mbDone = true;
  }
}

void CameraCalibrator::OptimizeOneStep() {
  int nViews = mvCalibImgs.size();
  int nDim = 6 * nViews + NUMTRACKERCAMPARAMETERS;
  int nCamParamBase = nDim - NUMTRACKERCAMPARAMETERS;

  TooN::Matrix<> mJTJ(nDim, nDim);
  TooN::Vector<> vJTe(nDim);
  mJTJ = TooN::Identity; // Weak stabilizing prior
  vJTe = TooN::Zeros;

  if (*mgvnDisableDistortion) mCamera.DisableRadialDistortion();


  double dSumSquaredError = 0.0;
  int nTotalMeas = 0;

  for (int n = 0; n < nViews; n++) {
    int nMotionBase = n*6;
    vector<CalibImage::ErrorAndJacobians> vEAJ = mvCalibImgs[n].Project(mCamera);
    for (unsigned int i = 0; i < vEAJ.size(); i++) {
      CalibImage::ErrorAndJacobians &EAJ = vEAJ[i];
      // All the below should be +=, but the MSVC compiler doesn't seem to understand that. :(
      mJTJ.slice(nMotionBase, nMotionBase, 6, 6) =
          mJTJ.slice(nMotionBase, nMotionBase, 6, 6) + EAJ.m26PoseJac.T() * EAJ.m26PoseJac;
      mJTJ.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) =
          mJTJ.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) + EAJ.m2NCameraJac.T() * EAJ.m2NCameraJac;
      mJTJ.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
          mJTJ.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ.m26PoseJac.T() * EAJ.m2NCameraJac;
      mJTJ.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
          mJTJ.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ.m26PoseJac.T() * EAJ.m2NCameraJac;
      // Above does twice the work it needs to, but who cares..

      vJTe.slice(nMotionBase,6) =
          vJTe.slice(nMotionBase,6) + EAJ.m26PoseJac.T() * EAJ.v2Error;
      vJTe.slice(nCamParamBase,NUMTRACKERCAMPARAMETERS) =
          vJTe.slice(nCamParamBase,NUMTRACKERCAMPARAMETERS) + EAJ.m2NCameraJac.T() * EAJ.v2Error;

      dSumSquaredError += EAJ.v2Error * EAJ.v2Error;
      ++nTotalMeas;
    }
  };

  mdMeanPixelError = sqrt(dSumSquaredError / nTotalMeas);

  TooN::SVD<> svd(mJTJ);
  TooN::Vector<> vUpdate(nDim);
  vUpdate= svd.backsub(vJTe);
  vUpdate *= 0.1; // Slow down because highly nonlinear...
  for(int n=0; n<nViews; n++)
    mvCalibImgs[n].mse3CamFromWorld = TooN::SE3<>::exp(vUpdate.slice(n * 6, 6)) * mvCalibImgs[n].mse3CamFromWorld;
  mCamera.UpdateParams(vUpdate.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS));
}
}  // namespace ptam















