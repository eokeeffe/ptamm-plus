// Copyright 2008 Isis Innovation Limited
#ifndef PTAM_UI_OPENGL_H_
#define PTAM_UI_OPENGL_H_

#ifndef GL_GLEXT_PROTOTYPES
  #define GL_GLEXT_PROTOTYPES
#endif

#ifdef __linux__
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>
#endif

#if defined(__APPLE__) || defined(MACOSX)
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#include <GLUT/glut.h>
#endif

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <cvd/gl_helpers.h>
#endif
