#include <iostream>
#include <gvars3/instances.h>

#include "ui/open_gl.h"
#include "ui/draw_helpers.h"
#include "ui/ar_render.h"
#include "ui/gl_window.h"
#include "config.h"
#include "slam_window_glut.h"

using namespace std;
using namespace cv;
using namespace ptam;

static ARRender<ATANCamera>* p_ar_render;
class ARDrawable : public ptam::AbstractDrawable {
public:
  ARDrawable() {}
  void Draw() {
    if (slam_window->mpMap->IsGood()) {
      glDrawAugmentation(slam_window->mpTracker->GetCurrentPose());
    }
  }

  void Draw2D() {
    if (slam_window->mpMap->IsGood()) {
      glRenderGrid(slam_window->mpTracker->GetCurrentPose(), *slam_window->mpCamera);
      //      glDrawTrackedPoints();
    } else {
      glDrawTrails(slam_window->mpTracker->GetTrails());
    }
  }

  SlamWindowCallback<ATANCamera>* slam_window;
};

// sample program for slam  tracking
int main(int argc, char * argv[]) {
  try {
    printf("PTAM \n");
    printf("===========================\n");
    printf("Parsing settings.cfg ....\n");
    std::string cam_configfile = (argc > 1) ? argv[1] : CAMERA_CONFIG_FILE;
    std::string configfile = (argc > 2) ? argv[2] : CONFIG_FILE;
    GVars3::GUI.LoadFile(cam_configfile);
    GVars3::GUI.LoadFile(configfile);
    i3d::VideoCaptureDispatch<i3d::CameraFamily::GENERIC> videodispatch;
    std::string vsource = GVars3::GV3::get<string>("Video.source");
    if (vsource.length() < 3) {
      if(!videodispatch.open(atoi(vsource.c_str())))
        return -1;
      videodispatch.setProperty("Brightness", GVars3::GV3::get<int>("Video.brightness", 50, GVars3::SILENT));
      videodispatch.setProperty("Exposure", GVars3::GV3::get<int>("Video.exposure", 40, GVars3::SILENT));
      videodispatch.setProperty("Gain", GVars3::GV3::get<int>("Video.gain", 40, GVars3::SILENT));
      videodispatch.setProperty("Shutter", GVars3::GV3::get<int>("Video.shutter", 530, GVars3::SILENT));
    } else {
      if(!videodispatch.open(vsource))
        return -1;
    }

//    // Parsing console input
//    GVars3::GUI.StartParserThread();
//    atexit(GVars3::GUI.StopParserThread);

    ARDrawable drawable;

    SlamWindowCallback<ATANCamera> slamwindow_callback;
    slamwindow_callback.video = &videodispatch;

    drawable.slam_window = &slamwindow_callback;

    ARRender<ATANCamera> ar_render;
    ar_render.Configure(slamwindow_callback.mpCamera);
    ar_render.add_drawable(&drawable);
    ar_render.set_render_mode(ar_render.DISTORT);

    p_ar_render = &ar_render;

    ptam::GLWindow gl_window(&ar_render, &slamwindow_callback);
    gl_window.init("Slam Demo with Glut", 640, 480, argc, argv);
//    gl_window.toggle_full_screen();

  } catch (std::exception& e) {
    cout << "main():: Oops! " << e.what() << endl;
  }
  return 0;
}

