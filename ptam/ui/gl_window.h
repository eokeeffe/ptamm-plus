// authors: Thanh Nguyen<thanh@icg.tugraz.at>
// Copyright 2012, TU Graz
// GLWindow for handling OpenGL Window (GLUT) & events

#ifndef PTAM_UI_GLWINDOW_H_
#define PTAM_UI_GLWINDOW_H_

#include <string>
#include <vector>
#ifdef WIN32
#include <windows.h>
#endif

#include "ui/open_gl.h"
#include "ui/ar_render.h"

namespace ptam {
class ATANCamera;

class GLWindowCallback {
 public:
  GLWindowCallback() {}
  virtual ~GLWindowCallback() {}
  virtual void get_video_frame(std::vector<unsigned char> &v_rgb_pixels) {}
  virtual void get_camera_pose(std::vector<double>& v_matrix4x3_camerapose) {}
  virtual void on_display() {}
  virtual void keyboard(unsigned char c_key, int x, int y) {}
};

typedef ARRender<ATANCamera>  RenderType;
class GLWindow {
 public:
  struct Properties {
    Properties() {
      glut_window_id = 0;
      window_width = 640;
      window_height = 480;
      init_pos_x = 100;
      init_pos_y = 100;
      window_name = "Put Window's Name Here";
      b_full_screen = false;  // fullscreen or not
      pausing = false;  // pause or not
    }
    int glut_window_id;
    int window_width, window_height, init_pos_x, init_pos_y;
    std::string window_name;

    bool b_full_screen;  // fullscreen or not
    bool pausing;  // pause or not
  };

  // Explicit default constructor
  GLWindow() {
    arrender_ = NULL;
  }

  GLWindow(RenderType* arrender, GLWindowCallback* callback) {
    arrender_ = arrender;
    callback_ = callback;
  }

  // initialization for the GLWindow object
  void init(const char* ca_name, int width, int height,
            int argc = 1, char** argv = NULL) {
    properties.window_width = width;
    properties.window_height = height;
    properties.b_full_screen = true;
    properties.window_name = ca_name;

    // OpenGL stuff
    glutInit(&argc, argv);  // using glut for window
    gl_init();               // opengl (& glut) initialization
    glutFullScreen();
    glutMainLoop();         // for glut

    if (arrender_ != NULL)
      arrender_->Init();
  }

  void gl_init() {
    // glut window initialisation
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition(properties.init_pos_x, properties.init_pos_y);
    glutInitWindowSize(properties.window_width, properties.window_height);
    properties.glut_window_id =
        glutCreateWindow(properties.window_name.c_str());

    // glut callbacks setting
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
  }

  static void update() { glutSwapBuffers(); }

  // Render graphics (video frame & necessary objects)
  static void display() {
    callback_->on_display();
    static std::vector<unsigned char> v_rgb_pixels(
          properties.window_width*properties.window_height*3);
    callback_->get_video_frame(v_rgb_pixels);
    if (arrender_ != NULL) {
      arrender_->set_video_frame(&v_rgb_pixels);
      arrender_->Render();
    }
    update();
  }

  // for glut keyboard handling
  static void keyboard(unsigned char ckey, int x, int y) {
    callback_->keyboard(ckey, x, y);
    switch (ckey) {
    case 0x1b:  // ESC
      GLWindow::release();  // release object
      break;
    case 'w':
      GLWindow::toggle_full_screen();  // full screen or normal screen
      break;
    case 'p':
      pause();
      break;
    case 'o':
      arrender_->set_render_mode(RenderType::ORIGINAL);
      break;
    case 'd':
      arrender_->set_render_mode(RenderType::DISTORT);
      break;
    case 'u':
      arrender_->set_render_mode(RenderType::UNDISTORT);
      break;
    }
  }

  static void idle() {
    glutPostRedisplay();
  }

  static void reshape(const int w, const int h) {
    if (arrender_ != NULL)
      arrender_->resize_window(w, h);
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    update();
  }

  static void release() {
    exit(1);
  }

  static void toggle_full_screen() {
    if (properties.b_full_screen) {
      glutPositionWindow(properties.init_pos_x, properties.init_pos_y);
      glutReshapeWindow(properties.window_width, properties.window_height);
      arrender_->resize_window(properties.window_width,
                               properties.window_height);
    } else {
      glutFullScreen();
    }
    properties.b_full_screen = !properties.b_full_screen;
  }

  static void pause() {
    if (!properties.pausing) {
      glutDisplayFunc(GLWindow::display);
    } else {
      glutDisplayFunc(GLWindow::idle);
    }
    properties.pausing = !properties.pausing;
  }

  static Properties properties;
  static GLWindowCallback* callback_;
  static RenderType* arrender_;
};

GLWindow::Properties GLWindow::properties;
GLWindowCallback* GLWindow::callback_;
RenderType* GLWindow::arrender_;

}  // namespace ptam
#endif  // PTAM_UI_GLWINDOW_H_

