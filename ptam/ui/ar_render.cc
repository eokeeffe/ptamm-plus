// authors: Thanh Nguyen<thanh@icg.tugraz.at>
// Copyright 2012, TU Graz

#include "ui/ar_render.h"
#include <cvd/gl_helpers.h>
#include "math/atan_camera.h"

using std::cout;
using std::endl;

namespace ptam {

template<typename CameraType>
ARRender<CameraType>::~ARRender() {
  glDeleteTextures(1, &u_frametexture);
  glDeleteBuffers(1, &u_framebuffer);
  glDeleteTextures(1, &u_framebuffer_texture);
}

template<typename CameraType>
void ARRender<CameraType>::Init() {
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_COLOR_MATERIAL);

#ifdef WIN32
  int ifake = 1;
  char* ca_fake = "";
  glutInit(&ifake, &ca_fake);
  if (GLEW_OK != glewInit()) {
    cout << "GLEW init failed";
    exit(1);
  }

  if (!GLEW_EXT_framebuffer_object) {
    cout << "Graphics card doesn't support FBO !!!:(((" << endl;
    exit(1);
  }
#endif
}

template<typename CameraType>
void ARRender<CameraType>::Configure(CameraType* camera) {
  video_camera = camera;
  rendermode = DISTORT;
  TooN::Vector<2> v2 = video_camera->GetImageSize();
  frametexture_width = v2[0];
  frametexture_height = v2[1];
  // work out FrameBuffer size to cover undistorted video frame
  const TooN::Vector<2> tl =
      video_camera->ProjectLinear(video_camera->UnProject(
                                    TooN::makeVector(0.0, 0.0)));
  const TooN::Vector<2> br =
      video_camera->ProjectLinear(video_camera->UnProject(
         TooN::makeVector(frametexture_width, frametexture_height)));
  TooN::Vector<2> offset =
      br - TooN::makeVector(frametexture_width, frametexture_height);
  if (TooN::norm(offset) < TooN::norm(tl)) {
    offset = -tl;
  }
  double max_ratio = std::max(
      (frametexture_width + offset[0]) / frametexture_width,
      (frametexture_height+offset[1])/frametexture_height);
  max_ratio *= 1.2;  // larger to surely cover undistorted video frame texture

  framebuffer_width = frametexture_width*max_ratio;
  framebuffer_height = frametexture_height*max_ratio;
  GenerateTextureMappingCoordinates();
  texture_initialized = false;
}

template<typename CameraType>
void ARRender<CameraType>::Render() {
  if (!texture_initialized)
    GenerateFrameBuffers();

  UploadVideoFrame();
  if (rendermode == ORIGINAL)
    return;

  set_frustum();  // render knows what the frustum looks like :)

  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glRotated(180, 1, 0, 0);  // rotate around X-axis 180degree (vision to OpenGL)
  CVD::glMultMatrix(se3_camerapose);

  for (size_t i = 0; i < drawables.size(); ++i)
    drawables[i]->Draw();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  if (rendermode == ORIGINAL)
    return;

  // map FBO Texture to default FBO
  MapFBO2SystemFBO();

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, u_system_framebuffer);
  glViewport(0, 0, width(), height());
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glOrtho(0, width(), height(), 0, 0, 1);  // Vision coordinates (start at top-left)

  glDisable(GL_DEPTH_TEST);

  glScaled(double(width())/frametexture_width,
           double(height())/frametexture_height, 1.0);
  for (size_t i = 0; i < drawables.size(); ++i)
    drawables[i]->Draw2D();
}

template<typename CameraType>
void ARRender<CameraType>::UploadVideoFrame() {
  std::vector<unsigned char>& v_pixels = *v_videoframe_pixels;

  // uploads the image frame to the frame texture
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, u_frametexture);
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,
                  0, 0, 0,
                  frametexture_width, frametexture_height,
                  GL_RGB,
                  GL_UNSIGNED_BYTE,
                  &v_pixels.front());

  // map frame texture to FBO / window-default-provided FBO
  if (rendermode == ORIGINAL) {
    // binding to window-system-provided FrameBuffer
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, u_system_framebuffer);
    glViewport(0, 0, width(), height());
  } else {  // UNDISTORT / DISTORT
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, u_framebuffer);
    glViewport(0, 0, framebuffer_width, framebuffer_height);
  }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (rendermode == ORIGINAL)
    glOrtho(0, 1, 1, 0, 0, 1);  // Vision coordinates (start at top-left)
  else
    glOrtho(0, 1, 0, 1, 0, 1);  // OpenGL coordinates (start at bottom-left)

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, u_frametexture);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  if (rendermode == ORIGINAL) {
    GLfloat fa_texcoords[8] = {0, 0,
                               frametexture_width, 0,
                               frametexture_width, frametexture_height,
                               0, frametexture_height};
    GLfloat fa_vertex[8] = {0, 0, 1, 0, 1, 1, 0, 1};
    glTexCoordPointer(2, GL_FLOAT, 0, fa_texcoords);
    glVertexPointer(2, GL_FLOAT, 0, fa_vertex);
    glDrawArrays(GL_QUADS, 0, 4);
  } else {  // UNDISTORT / DISTORT
    glTexCoordPointer(2, GL_FLOAT,
                      sizeof(TooN::Vector<2, float>), &v_dist_texcoords.front()[0]);
    glVertexPointer(2, GL_FLOAT,
                    sizeof(TooN::Vector<2, float>), &v_undist_vertexs.front()[0]);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, v_dist_texcoords.size());
  }
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);

  glClearDepth(1);
  glClear(GL_DEPTH_BUFFER_BIT);
}

template<typename CameraType>
void ARRender<CameraType>::MapFBO2SystemFBO() {
  // map FBO Texture to default window-system-provided FBO
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, u_system_framebuffer);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glViewport(0, 0, width(), height());

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, 0, 1);  // OpenGL coordinates (start at bottom-left)

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const TooN::Vector<2> v2_offset = TooN::makeVector(
        (framebuffer_width-frametexture_width)*0.5,
        (framebuffer_height-frametexture_height)*0.5);
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, u_framebuffer_texture);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  if (rendermode == UNDISTORT) {
    GLfloat fa_texcoords[8] = {
      v2_offset[0], v2_offset[1],
      v2_offset[0], v2_offset[1] + frametexture_height,
      v2_offset[0] + frametexture_width, v2_offset[1]+frametexture_height,
      v2_offset[0] + frametexture_width, v2_offset[1]};
    GLfloat fa_vertex[8] = {0, 0, 0, 1, 1, 1, 1, 0};
    glTexCoordPointer(2, GL_FLOAT, 0, fa_texcoords);
    glVertexPointer(2, GL_FLOAT, 0, fa_vertex);
    glDrawArrays(GL_QUADS, 0, 4);
  } else if (rendermode == DISTORT) {
    glTexCoordPointer(2, GL_FLOAT,
                      sizeof(TooN::Vector<2, float>), &v_undist_texcoords.front()[0]);
    glVertexPointer(2, GL_FLOAT,
                    sizeof(TooN::Vector<2, float>), &v_dist_vertexs.front()[0]);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, v_dist_texcoords.size());
  }
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);

  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}

template<typename CameraType>
void ARRender<CameraType>::GenerateTextureMappingCoordinates() {
  // scaled by aspect ratio
  int y_step = static_cast<int>(
        x_sampling_step *
        (static_cast<double>(frametexture_height) / frametexture_width));
  y_step = (y_step < 2) ? 2: y_step;
  // offset between frame texture (video frame) & frame buffer texture (FBO)
  const TooN::Vector<2, float> v2_offset = TooN::makeVector((framebuffer_width-frametexture_width)*0.5,
                                  (framebuffer_height-frametexture_height)*0.5);
  for (int y = 0; y < y_step; y++) {
    size_t current_size = v_dist_texcoords.size();
    for (int x = 0; x <= x_sampling_step; x++)
      for (int yy = y; yy <= y+1; yy++) {
        TooN::Vector<2, float> v2_distorted;
        v2_distorted[0] =
            static_cast<float>(x*frametexture_width) / x_sampling_step;
        v2_distorted[1] = static_cast<float>(yy*frametexture_height) / y_step;
        // cope with border of frame.
        if (x == 0 || yy == 0)
          for (int i = 0; i < 2; i++)
            v2_distorted[i] = v2_distorted[i] - 3;
        if (x == x_sampling_step || yy == y_step)
          for (int i = 0; i < 2; i++)
            v2_distorted[i] = v2_distorted[i] + 3;
        TooN::Vector<2, float> v2_undistorted =
            video_camera->ProjectLinear(video_camera->UnProject(v2_distorted));
        if (y%2 == 0)  // odd row will start from tail of the previous even row
          current_size = v_dist_texcoords.size();
        // video frame (vision coordinates) to FBO (OpenGL coordinates)
        v_dist_texcoords.insert(v_dist_texcoords.begin() +
                                current_size, v2_distorted);
        v_undist_vertexs.insert(v_undist_vertexs.begin()+current_size,
         TooN::makeVector<float>((v2_undistorted[0]+v2_offset[0])/framebuffer_width,
            1 - (v2_undistorted[1]+v2_offset[1])/framebuffer_height));

        // FBO to default FBO
        v_undist_texcoords.insert(
              v_undist_texcoords.begin() + current_size,
              TooN::makeVector<float>(v2_undistorted[0] + v2_offset[0],
                              (v2_undistorted[1]+v2_offset[1])));
        v_dist_vertexs.insert(
              v_dist_vertexs.begin() + current_size,
              TooN::makeVector<float>(v2_distorted[0]/frametexture_width,
                              v2_distorted[1]/frametexture_height));
      }
  }
}

template<typename CameraType>
bool ARRender<CameraType>::GenerateFrameBuffers() {
  texture_initialized = true;
  // make texture function to be in GL_DECAL mode (replacing background colors)
  // http://glprogramming.com/red/chapter09.html
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  // generate frame texture
  glGenTextures(1, &u_frametexture);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, u_frametexture);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0,
               GL_RGBA, frametexture_width, frametexture_height, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, NULL);

  // generate frame buffer texture
  glGenTextures(1, &u_framebuffer_texture);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, u_framebuffer_texture);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0,
               GL_RGBA, framebuffer_width, framebuffer_height, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  // generate render buffer (depth buffer)
  glGenRenderbuffersEXT(1, &u_framebuffer_depthbuffer);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, u_framebuffer_depthbuffer);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24,
                           framebuffer_width, framebuffer_height);

  // generate frame buffer object
  glGenFramebuffersEXT(1, &u_framebuffer);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, u_framebuffer);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                            GL_TEXTURE_RECTANGLE_ARB, u_framebuffer_texture, 0);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                               GL_RENDERBUFFER_EXT, u_framebuffer_depthbuffer);

  GLenum n = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  if (n == GL_FRAMEBUFFER_COMPLETE_EXT)
    return true;  // All good
  printf("ARRender:: glCheckFrameBufferStatusExt returned an error.\n");
  return false;
}

// instantiate to trigger the rest =============================================
template ARRender<ATANCamera>::ARRender();
template ARRender<ATANCamera>::~ARRender();
template void ARRender<ATANCamera>::Init();
template void ARRender<ATANCamera>::Render();
template void ARRender<ATANCamera>::Configure(ATANCamera* camera);
}  // namespace ptam
