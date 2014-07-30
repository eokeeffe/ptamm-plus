#ifndef PTAM_UI_DRAW_HELPERS_H_
#define PTAM_UI_DRAW_HELPERS_H_
#include <list>
#include <cvd/gl_helpers.h>
#include <ui/open_gl.h>

namespace ptam {

TooN::Vector<3> get_level_color(int level) {
  switch (level) {
  case 0:
    return TooN::makeVector(1.0, 0.0, 0.0);
  case 1:
    return TooN::makeVector(1.0, 1.0, 0.0);
  case 2:
    return TooN::makeVector(0.0, 1.0, 0.0);
  case 3:
    return TooN::makeVector(0.0, 0.0, 0.7);
  default:
    return TooN::makeVector(1.0, 1.0, 0.7);
  }
}

// draw list of correspondences between two initial stereo frames.
template<typename TrailType>
inline void glDrawTrails(const std::list<TrailType>& trails) {
  glPointSize(5);
  glLineWidth(2);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
//  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  glEnable(GL_BLEND);
  glBegin(GL_LINES);

  for (typename std::list<TrailType>::const_iterator i = trails.begin(); i != trails.end();) {
    typename std::list<TrailType>::const_iterator next = i; next++;
    const TrailType trail = *i;
    if(!trail.found)
      glColor3f(0, 1, 1); // Failed trails flash purple before dying.
    else
      glColor3f(1, 1, 0);
    glVertex(trail.irInitialPos);
    if(trail.found) glColor3f(1, 0, 0);
    glVertex(trail.irCurrentPos);
    i = next;
  }
  glEnd();
}

// Draw the reference grid to give the user an idea of wether tracking is OK or not.
template<typename CameraModel>
inline void glRenderGrid(TooN::SE3<> pose_cfw, CameraModel& camera,
                         bool did_coarse = false) {
  // The colour of the ref grid shows if the coarse stage of tracking was used
  // (it's turned off when the camera is sitting still to reduce jitter.)
  if (did_coarse)
    glColor4f(.0, 0.5, .0, 0.6);
  else
    glColor4f(0, 0, 0, 0.6);

  // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
  const int half_cellsize = 8;
  const int ntot = half_cellsize * 2 + 1;
  std::vector<std::vector<TooN::Vector<2> > > vertices(ntot);
  for (int i = 0; i < ntot; i++){
    vertices[i].resize(ntot);
    for (int j = 0; j < ntot; j++) {
      TooN::Vector<3> v3;
      v3[0] = (i - half_cellsize) * 0.1;
      v3[1] = (j - half_cellsize) * 0.1;
      v3[2] = 0.0;
      TooN::Vector<3> v3Cam = pose_cfw * v3;
      if (v3Cam[2] < 0.001)
        v3Cam[2] = 0.001;
      vertices[i][j] = camera.Project(TooN::project(v3Cam));
    }
  }
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for (int i = 0; i < ntot; i++) {
    glBegin(GL_LINE_STRIP);
    for (int j = 0; j < ntot; j++)
      CVD::glVertex(vertices[i][j]);
    glEnd();

    glBegin(GL_LINE_STRIP);
    for (int j = 0; j < ntot; j++)
      CVD::glVertex(vertices[j][i]);
    glEnd();
  };

  glLineWidth(1);
  glColor3f(1, 0, 0);
}

template<typename TrackDataPtr>
inline void glDrawTrackedPoints(std::vector<TrackDataPtr> iteration_set) {
  glPointSize(6);
  glEnable(GL_BLEND);
  glEnable(GL_POINT_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_POINTS);
  for (typename std::vector<TrackDataPtr>::reverse_iterator it = iteration_set.rbegin();
       it != iteration_set.rend();
       it++) {
    if (!(*it)->bFound)
      continue;
    glColor(get_level_color((*it)->nSearchLevel));
    glVertex((*it)->template  v2Image);
  }
  glEnd();
  glDisable(GL_BLEND);
}

inline void glDrawCube(
    const TooN::Vector<3> &color = TooN::makeVector(0, 0, 0.9)) {
  glBegin(GL_QUADS);  // Start Drawing The Cube
  CVD::glColor(color * 1.2);  // Set The Color To Green
  glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Quad (Top)
  glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Quad (Top)
  glVertex3f(-1.0f, 1.0f, 1.0f);  // Bottom Left Of The Quad (Top)
  glVertex3f(1.0f, 1.0f, 1.0f);  // Bottom Right Of The Quad (Top)
  CVD::glColor(color * 0.8);  // Set The Color To Orange
  glVertex3f(1.0f, -1.0f, 1.0f);  // Top Right Of The Quad (Bottom)
  glVertex3f(-1.0f, -1.0f, 1.0f);  // Top Left Of The Quad (Bottom)
  glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Left Of The Quad (Bottom)
  glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Right Of The Quad (Bottom)
  CVD::glColor(color * 1);  // Set The Color To Red
  glVertex3f(1.0f, 1.0f, 1.0f);  // Top Right Of The Quad (Front)
  glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Right Of The Quad (Front)
  glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Left Of The Quad (Front)
  glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Left Of The Quad (Front)
  CVD::glColor(color * 1.2);  // Set The Color To Yellow
  glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Left Of The Quad (Back)
  glVertex3f(1.0f, 1.0f, -1.0f);  // Top Left Of The Quad (Back)
  glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Right Of The Quad (Back)
  glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Right Of The Quad (Back)
  CVD::glColor(color * 0.8);  // Set The Color To Blue
  glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Right Of The Quad (Left)
  glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Quad (Left)
  glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Left Of The Quad (Left)
  glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Right Of The Quad (Left)
  CVD::glColor(color * 1);  // Set The Color To Violet
  glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Quad (Right)
  glVertex3f(1.0f, 1.0f, 1.0f);  // Top Left Of The Quad (Right)
  glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Left Of The Quad (Right)
  glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Right Of The Quad (Right)
  glEnd();
}

inline void glDrawAugmentation(const  TooN::SE3<>& pose) {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glRotated(180, 1, 0, 0);
  CVD::glMultMatrix(pose);
  glTranslated(0, 0, 0);

  // draw nice box
  glClear(GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glPushMatrix();
  glScaled(0.01, 0.01, 0.1);
  glTranslated(0, 0, 1);
  glDrawCube();
  glPopMatrix();
  glPushMatrix();
  glScaled(0.1, 0.01, 0.01);
  glTranslated(0, 0, -1);
  glDrawCube(TooN::makeVector(0.8, 0, 0));
  glPopMatrix();
  glPushMatrix();
  glScaled(0.01, 0.1, 0.01);
  glTranslated(0, 0, -1);
  glDrawCube(TooN::makeVector(0.0, 0.8, 0));
  glPopMatrix();
  glDisable(GL_DEPTH_TEST);

  glPopMatrix();
}
}  // namespace ptam
#endif // PTAM_UI_DRAW_HELPERS_H_
