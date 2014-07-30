cmake_minimum_required(VERSION 2.8)
project(ptam_test)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

# common settings
common_build_setting()

##==============================================================================
# This is the path where the images and testdata are stored
set(DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR})
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_SOURCE_DIR}/config.h )

##==============================================================================
if(NOT PTAM_DIR)
set (PTAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../PTAM)
set (PTAM_INCLUDE_DIRS ${PTAM_DIR}/include)
set(CMAKE_MODULE_PATH "${PTAM_DIR}/../cmake/modules")
include(Config)
include(Util)
endif(NOT PTAM_DIR)

set(EXTRA_INC_DIRS ${EXTRA_INC_DIRS} ${PTAM_DIR})
set(PTAM_BUILD_DIR ${CMAKE_BINARY_DIR}/PTAM)
link_directories(${PTAM_BUILD_DIR})

if(WIN32)
link_directories($ENV{BOOST_ROOT}/stage/lib)
endif(WIN32)

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
include_directories(${EXTRA_INC_DIRS})

##==============================================================================
if(WIN32)
  link_directories($ENV{OPENGL_ROOT}/lib)
  set(ALL_LIBRARIES ${PTAM_LIBRARIES} ${EXTRA_LIBRARIES} opengl32 glu32 glew32 freeglut)
  add_definitions(/DCVD_DISABLE_PTHREAD)
else(WIN32)
  if(APPLE)
    set(ALL_LIBRARIES  ${PTAM_LIBRARIES} ${EXTRA_LIBRARIES} "-framework OpenGL;-framework GLUT;")
  else(APPLE)
    set(ALL_LIBRARIES ${PTAM_LIBRARIES} ${EXTRA_LIBRARIES} GL GLU GLEW glut)
  endif(APPLE)
endif(WIN32)

include_directories(${EXTRA_INC_DIRS})

##==============================================================================
##  executables

# ptam_construct

# ptam_track

# ptam_ui
common_build_routine(test_ui_ar_render test_ui_ar_render.cc)
target_link_libraries(test_ui_ar_render ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )


