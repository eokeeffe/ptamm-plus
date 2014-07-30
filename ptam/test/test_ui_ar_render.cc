#include <iostream>
#include <gvars3/instances.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "math/atan_camera.h"
#include "ui/open_gl.h"
#include "ui/draw_helpers.h"
#include "ui/ar_render.h"
#include "ui/gl_window.h"
#include "util/opencv.h"
#include "config.h"

using namespace std;
using namespace cv;
using namespace ptam;

static ARRender<ATANCamera>* p_ar_render;

class SimpleWindowCallback : public ptam::GLWindowCallback {
public:
  SimpleWindowCallback() {}

protected:
  virtual void get_video_frame(std::vector<unsigned char>& v_rgb_pixels) {
    static cv::Mat img = cv::imread(DATA_PATH("data/closeup.jpg"));
    static cv::Mat rgb_img;
    cv::cvtColor(img, rgb_img, CV_BGR2RGB);

    if (v_rgb_pixels.size() != 3*rgb_img.cols*rgb_img.rows)
      v_rgb_pixels.resize(3*rgb_img.cols*rgb_img.rows);
    SafeCopyImage(v_rgb_pixels, rgb_img);
  }

  virtual void keyboard(unsigned char c_key, int x, int y) {
    c_pressed_key = c_key;
    switch (c_key) {
    case 27: //escape
      exit(0);
      break;
    default: ;

    }
    std::cout << "pressed:" << c_key << std::endl;
  }

 public:
  unsigned char c_pressed_key;
};

class ARDrawable : public ptam::AbstractDrawable {
public:
  ARDrawable() {}
  void Draw() {}

  void Draw2D() {
      glLineWidth(10);
      glColor3d(1, 0, 0);
      CVD::glRect(TooN::makeVector(10, 10),TooN::makeVector(50, 50));

      TooN::Vector<2> v1 = TooN::makeVector(10, 10);
      TooN::Vector<2> v2 = TooN::makeVector(100, 100);
      CVD::glLine(v1, v2);
  }
};

// sample program for slam  tracking
int main(int argc, char * argv[]) {
  try {
    ARDrawable drawable;

    SimpleWindowCallback simplewindow_callback;
    TooN::Vector<NUMTRACKERCAMPARAMETERS> vTest;

    vTest = GVars3::GV3::get<TooN::Vector<NUMTRACKERCAMPARAMETERS> >(
          "Camera.Parameters", ATANCamera::mvDefaultParams, GVars3::HIDDEN);
    ATANCamera* camera = new ATANCamera("Camera");
    ARRender<ATANCamera> ar_render;
    ar_render.Configure(camera);
    ar_render.add_drawable(&drawable);
    ar_render.set_render_mode(ar_render.DISTORT);

    p_ar_render = &ar_render;

    ptam::GLWindow gl_window(&ar_render, &simplewindow_callback);
    gl_window.init("Slam Demo with Glut", 640, 480, argc, argv);

  } catch (std::exception& e) {
    cout << "main():: Oops! " << e.what() << endl;
  }
  return 0;
}


