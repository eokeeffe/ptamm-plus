cmake_minimum_required(VERSION 2.8)
project(calibration)

#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)

## application specific
common_build_setting()

##
if(NOT PTAM_DIR)
    set(PTAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/PTAM)
    set(PTAM_INCLUDE_DIRS ${PTAM_DIR}/include)
    set(CMAKE_MODULE_PATH "${PTAM_DIR}/../cmake/modules")
    include(Config)
    include(Util)
endif(NOT PTAM_DIR)

set(EXTRA_INC_DIRS ${EXTRA_INC_DIRS} ${PTAM_DIR})
set(PTAM_BUILD_DIR ${CMAKE_BINARY_DIR}/PTAM)
link_directories(${PTAM_BUILD_DIR})

##==============================================================================
# This is the path where the images and testdata are stored
SET( CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR})
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_SOURCE_DIR}/config.h )
##==============================================================================
##  OpenCV
find_package(OpenCV REQUIRED)
set(EXTRA_INC_DIRS
    ${EXTRA_INC_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
set(EXTRA_LIBRARIES
    ${EXTRA_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
message("Found OpenCV libs:${OpenCV_LIBRARIES}")
message("OpenCV INC DIRS:
    ${OpenCV_INCLUDE_DIRS}")

##==============================================================================
set(ICG3D_DIR /Users/thanhnguyen/dev/hilti/trunk)
set(ICG3D_INC_DIRS "${ICG3D_DIR};${ICG3D_DIR}/icg3d")
set(ICG3D_LIB_DIR ${ICG3D_DIR}/lib)
set(ICG3D_LIBRARIES icg3d_core icg3d_sensor)

set(EXTRA_INC_DIRS ${EXTRA_INC_DIRS} ${ICG3D_INC_DIRS})
set(EXTRA_LIBRARIES ${EXTRA_LIBRARIES} ${ICG3D_LIBRARIES})
link_directories(${ICG3D_LIB_DIR})

##==============================================================================
include_directories(${EXTRA_INC_DIRS})
# declare to builb objects
if(WIN32)
  link_directories($ENV{OPENGL_ROOT}/lib)
  set(ALL_LIBRARIES ${PTAM_LIBRARIES} ${XSENS_LIBRARIES} ${EXTRA_LIBRARIES} opengl32 glu32 glew32 freeglut)
  add_definitions(/DCVD_DISABLE_PTHREAD  /DCVD_DISABLE_TOON)
else(WIN32)
  if(APPLE)
    set(ALL_LIBRARIES  ${PTAM_LIBRARIES} ${XSENS_LIBRARIES} ${EXTRA_LIBRARIES} "-framework OpenGL;-framework GLUT;")
  else(APPLE)
    set(ALL_LIBRARIES ${PTAM_LIBRARIES} ${XSENS_LIBRARIES} ${EXTRA_LIBRARIES} GL GLU GLEW glut)
  endif(APPLE)
endif(WIN32)

##==============================================================================
common_build_routine(calibrator camera_calibrator.cc)

