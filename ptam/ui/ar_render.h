// authors: Thanh Nguyen<thanh@icg.tugraz.at>
// Copyright 2012, TU Graz

#ifndef PTAM_UI_ARRENDER_H_
#define PTAM_UI_ARRENDER_H_

#include <vector>
#include <TooN/TooN.h>
#include <ui/open_gl.h>

namespace ptam {
// drawable object will inherate this
class AbstractDrawable {
 public:
  AbstractDrawable() {}
  virtual ~AbstractDrawable() {}
  virtual void Draw() {}
  virtual void Draw2D() {}
};  // class AbstractDrawable

// AR rendering grand class, for distortion/undistortion rendering
template<typename CameraType>
class ARRender {
 public :
  enum RenderMode {UNDISTORT = 0, DISTORT = 1, ORIGINAL = 2};

  ARRender() : u_system_framebuffer(0) {}
  virtual ~ARRender();

  virtual void Init();

  virtual void Render();

  virtual int width() {return currentviewport_width;}

  virtual int height() {return currentviewport_height;}

  virtual void resize_window(const int w, const int h) {
    currentviewport_width = w;
    currentviewport_height = h;
    // setup viewport
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    glViewport(0, 0, (GLint)w, (GLint)h);
  }

  virtual void Configure(CameraType* camera);

  virtual void add_drawable(AbstractDrawable* drawable) {
    drawables.push_back(drawable);
  }

  virtual void set_render_mode(RenderMode mode) {
    rendermode = mode;
  }

  virtual void set_video_frame(std::vector<unsigned char>* pixels) {
    v_videoframe_pixels = pixels;
  }

  virtual void set_camera_pose(const TooN::SE3<> &camerapose) {
    se3_camerapose = camerapose;
  }

  // convert coordinates from imageplane to current viewport coordinates
  virtual void convert_coordinate(double& x, double& y) {
    x = x*width()/frametexture_width;
    y = y*height()/frametexture_height;
  }

 protected :
  // set frustum
  virtual void set_frustum(double z_near = 0.001, double z_far = 1.e+6) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // figure out right frustum for whole FrameBuffer
    // (larger size than video frame, to cover texture)
    static const TooN::Vector<2>
        v2 = TooN::makeVector((framebuffer_width-frametexture_width) * 0.5,
           (framebuffer_height-frametexture_height) * 0.5);
    // note: this is in OpenGL coordinates (start bottom-left)
    static const TooN::Vector<2> bl =
        video_camera->UnProjectLinear(
          TooN::makeVector(-v2[0], -v2[1]));
    static const TooN::Vector<2> tr =
        video_camera->UnProjectLinear(TooN::makeVector(
                                        frametexture_width+v2[0],
                                        frametexture_height+v2[1]));
    double left = bl[0] * z_near;
    double right = tr[0] * z_near;
    double top = tr[1] * z_near;
    double bottom = bl[1] * z_near;
    ::glFrustum(left, right, bottom, top, z_near, z_far);
  }

  // generate texture mapping coordinates for distortion/undistortion rendering
  virtual void GenerateTextureMappingCoordinates();
  // FrameTexture(binded to default FBO),
  // FrameBuffer, FrameBufferTexture (binded to FrameBuffer)
  virtual bool GenerateFrameBuffers();
  // upload video frame to frame texture
  virtual void UploadVideoFrame();
  // map rendered frame to default frame buffer
  virtual void MapFBO2SystemFBO();

  // distorted coords in Vision coordinates (start upper-left)
  std::vector<TooN::Vector<2, float> > v_dist_texcoords;
  // undistorted vertexs in OpenGL coordinates (start lower-left)
  std::vector<TooN::Vector<2, float> > v_undist_vertexs;

  // undistorted coords in OpenGL coordinates
  std::vector<TooN::Vector<2, float> > v_undist_texcoords;
  // distorted vertex in OpenGL coordinates
  std::vector<TooN::Vector<2, float> > v_dist_vertexs;

  // sampling step used for texture mapping (coordinate generation)
  static const int x_sampling_step = 48;

  RenderMode rendermode;
  CameraType* video_camera;
  std::vector<AbstractDrawable*> drawables;
  std::vector<unsigned char>* v_videoframe_pixels;
  TooN::SE3<> se3_camerapose;

  GLuint u_system_framebuffer;
  // texture stuffs
  GLuint u_frametexture;
  int frametexture_width;
  int frametexture_height;

  // frame-buffer-object for AR rendering
  GLuint u_framebuffer;
  GLuint u_framebuffer_texture;
  GLuint u_framebuffer_depthbuffer;
  int framebuffer_width;
  int framebuffer_height;
  bool texture_initialized;

  // window stuffs
  int currentviewport_width;
  int currentviewport_height;
};
}  // namespace ptam
#endif  // PTAM_UI_ARRENDER_H_
