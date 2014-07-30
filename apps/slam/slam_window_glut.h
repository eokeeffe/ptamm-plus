#ifndef APPS_SLAM_SLAM_WINDOW_GLUT_H_
#define APPS_SLAM_SLAM_WINDOW_GLUT_H_
#include <iostream>
#include <time.h>
#include <sys/time.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "sensor/videocapture_dispatch.h"
#include "construct/map.h"
#include "construct/map_maker.h"
#include "track/tracker.h"
#include "ui/ar_render.h"
#include "ui/gl_window.h"
#include "util/opencv.h"

namespace ptam {

template<typename CameraType>
class SlamWindowCallback : public ptam::GLWindowCallback {
public:
  SlamWindowCallback() {
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

    mpMap = new Map;
    mpMapMaker = new MapMaker(*mpMap, *mpCamera);
    mpTracker = new Tracker(img_size, *mpCamera, *mpMap, *mpMapMaker);
  }

  i3d::VideoCaptureDispatch<i3d::CameraFamily::GENERIC>* video;

protected:
  virtual void get_video_frame(std::vector<unsigned char>& v_rgb_pixels) {
    static cv::Mat clone_rgb;
    try {
      timeval tim;
      gettimeofday(&tim, NULL);
      double start = tim.tv_sec + (tim.tv_usec / 1000000.0);

      // We use two versions of each video frame:
      // One black and white (for processing by the tracker etc)
      // and one RGB, for drawing.

      // Grab new video frame...
      cv::Mat rgb_frame = video->grabFrame();
      cv::resize(rgb_frame, rgb_frame, cv::Size(640, 480));
      clone_rgb = rgb_frame.clone();
      CVD::SubImage<CVD::Rgb<CVD::byte> > cvd_rgb_frame(
            (CVD::Rgb<CVD::byte>*)rgb_frame.data,
            CVD::ImageRef(rgb_frame.cols, rgb_frame.rows), rgb_frame.cols);
      cv::Mat gray_frame;
      cv::cvtColor(rgb_frame, gray_frame, CV_RGB2GRAY);
      mimFrameRGB.copy_from(cvd_rgb_frame);
      CVD::SubImage<CVD::byte> cvd_gray_frame(gray_frame.data,
            CVD::ImageRef(gray_frame.cols, gray_frame.rows), rgb_frame.cols);
      mimFrameBW.copy_from(cvd_gray_frame);

      mpTracker->TrackFrame(mimFrameBW);

      if (v_rgb_pixels.size() != 3*clone_rgb.cols*clone_rgb.rows)
        v_rgb_pixels.resize(3*clone_rgb.cols*clone_rgb.rows);
      SafeCopyImage(v_rgb_pixels, clone_rgb);

      static int frame_count=0;
      frame_count++;
      if (frame_count == 30) {
        frame_count = 0;
      }
      boost::this_thread::sleep(boost::posix_time::millisec(30));
    } catch (std::exception &e) {
      printf("SlamWindowCallback:: Oops: %s\n", e.what());
    }
  }

  virtual void get_camera_pose(std::vector<double>& v_matrix4x3_camerapose) {

  }

  virtual void on_display(){}

  virtual void keyboard(unsigned char c_key, int x, int y) {
    c_pressed_key = c_key;
    switch (c_key) {
    case 'r':
      mpTracker->Reset();
      break;
    case 27: //escape
      exit(0);
      break;
    case ' ':
      mpTracker->AskInitialTrack();
      break;
    default: ;

    }
    std::cout << "pressed:" << c_key << std::endl;
  }

 public:
  unsigned char c_pressed_key;

  CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;
  CVD::Image<CVD::byte> mimFrameBW;

  Map *mpMap;
  MapMaker *mpMapMaker;
  Tracker *mpTracker;
  ATANCamera *mpCamera;
};
}  // namespace ptam
#endif  // APPS_SLAM_SLAM_WINDOW_GLUT_H_
