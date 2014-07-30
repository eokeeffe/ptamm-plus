## Straightforawrd FindOpenCV for opencv versions(1.1, 1.2, 2.0, 2.1, 2.2, 2.3, 2.4)
#
# required (otherwise it will assume some possible directories, ex: /user/local/):
#   - OpenCV_ROOT_DIR
# optional:
#   - CMAKE_OSX_ARCHITECTURES  (default: upto local machine; or it can be: i386 x86_64 ppc ppc64) to specify Target architect (32-bit or 64-bit or ...)
#           example: -D CMAKE_OSX_ARCHITECTURES=i386;x86_64;ppc
# assumptions:
#   - inc dir: /OpenCV_ROOT_DIR/include/opencv/cv.h
#   - lib dir: /OpenCV_ROOT_DIR/lib/
# output:
#   - OpenCV_INCLUDE_DIRS
#   - OpenCV_LIBRARIES
# example:
#   ~/cmake -D OpenCV_ROOT_DIR=/opt/local/ /path-to-CMakeLists.txt/
#   ~cmake -D OpenCV_ROOT_DIR=/Users/thanhnguyen/deps/OpenCV-2.1.0/build/i386/release/ /path-to-CMakeLists.txt/

if(NOT OpenCV_ROOT_DIR)
  if(DEFINED ENV{OpenCV_ROOT_DIR})
    set(OpenCV_ROOT_DIR $ENV{OpenCV_ROOT_DIR})
    set(OpenCV_INCLUDE_DIRS $ENV{OpenCV_ROOT_DIR}/include)
  else(DEFINED ENV{OpenCV_ROOT_DIR})
    find_path(OpenCV_INCLUDE_DIRS
        NAMES
        /opencv/cv.h
        PATHS
        /opt/local/include
        /user/local/incude
        )
    if(OpenCV_INCLUDE_DIRS)
      set(OpenCV_ROOT_DIR ${OpenCV_INCLUDE_DIRS}/..)
    endif(OpenCV_INCLUDE_DIRS)
  endif(DEFINED ENV{OpenCV_ROOT_DIR})
endif(NOT OpenCV_ROOT_DIR)

# find incdir
if(NOT OpenCV_INCLUDE_DIRS)
find_path(OpenCV_INCLUDE_DIRS
  NAMES /opencv/cv.h
  PATHS ${OpenCV_ROOT_DIR}/include)
endif(NOT OpenCV_INCLUDE_DIRS)

set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}/opencv ${OpenCV_INCLUDE_DIRS}/opencv2)

# library linkdir suffixes appended to OpenCV_ROOT_DIR
SET(OpenCV_LIBDIR_SUFFIXES
  lib
  lib/Debug
  lib/Release
  )

# find sbsolute path to a core lib (to make sure there is something)
find_library(OPENCV_FIND_VERSION_1AND2
 NAMES cv cv120 cv200 cv210
 PATHS ${OpenCV_ROOT_DIR}
 PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
set(OpenCV_VERSION_22Plus 1)

if(OPENCV_FIND_VERSION_1AND2)
  find_library(A_KEY_LIB
  NAMES cv cv120 cv200 cv210
  PATHS ${OpenCV_ROOT_DIR}
  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
  set(OpenCV_VERSION_22Plus 0)
else(OPENCV_FIND_VERSION_1AND2)
   find_library(A_KEY_LIB
   NAMES opencv_core opencv_core220 opencv_core231 opencv_core241 opencv_core242
   PATHS ${OpenCV_ROOT_DIR}
   PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
endif(OPENCV_FIND_VERSION_1AND2)

message("A_KEY_LIB:${A_KEY_LIB}")
message("OpenCV_ROOT_DIR:${OpenCV_ROOT_DIR}")
# Logic selecting required libs
if(A_KEY_LIB AND OpenCV_INCLUDE_DIRS)
  set(OpenCV_FOUND ON)
  set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} )
  if(OpenCV_VERSION_22Plus)
    set(OpenCV_LIB_NAMES opencv_calib3d opencv_contrib opencv_core opencv_features2d opencv_flann opencv_gpu
                         opencv_highgui opencv_imgproc opencv_legacy opencv_ml opencv_nonfree opencv_objdetect
                         opencv_photo opencv_stitching opencv_ts opencv_video opencv_videostab)
    foreach(NAME ${OpenCV_LIB_NAMES} )
      find_library(OpenCV_${NAME}_LIB
                   NAMES ${NAME} ${NAME}220 ${NAME}231 ${NAME}241 ${NAME}242
                   PATHS ${OpenCV_ROOT_DIR}
                   PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
      set(OpenCV_LIBRARIES ${OpenCV_LIBRARIES} ${OpenCV_${NAME}_LIB})
    endforeach(NAME)

  elseif(NOT OpenCV_VERSION_22Plus)
    FIND_LIBRARY(OpenCV_CV_LIB
      NAMES cv cv120 cv200 cv210
      PATHS ${OpenCV_ROOT_DIR}
      PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES}  )
    FIND_LIBRARY(OpenCV_CVAUX_LIB
      NAMES cvaux cvaux120 cvaux200 cvaux210
      PATHS ${OpenCV_ROOT_DIR}
      PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES}  )
    FIND_LIBRARY(OpenCV_CXCORE_LIB
      NAMES cxcore cxcore120 cxcore200 cxcore210
      PATHS ${OpenCV_ROOT_DIR}
      PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
    FIND_LIBRARY(OpenCV_CXTS_LIB
      NAMES cxts cxts120 cxts200 cxts210
      PATHS ${OpenCV_ROOT_DIR}
      PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
    FIND_LIBRARY(OpenCV_HIGHGUI_LIB
      NAMES highgui highgui120 highgui200 highgui210
      PATHS ${OpenCV_ROOT_DIR}
      PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES}  )
    FIND_LIBRARY(OpenCV_ML_LIB
      NAMES ml ml120 ml200 ml210
      PATHS ${OpenCV_ROOT_DIR}
      PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} )
    set(OpenCV_LIBRARIES ${OpenCV_CV_LIB} ${OpenCV_CVAUX_LIB} ${OpenCV_CXCORE_LIB} ${OpenCV_HIGHGUI_LIB} ${OpenCV_ML_LIB} ${OpenCV_CXTS_LIB})
  endif(OpenCV_VERSION_22Plus)
endif(A_KEY_LIB AND OpenCV_INCLUDE_DIRS)

mark_as_advanced(
  OpenCV_ROOT_DIR
  OpenCV_INCLUDE_DIRS
  OpenCV_LIBRARIES
  )

# display help message
if(NOT OpenCV_FOUND)
  # make FIND_PACKAGE friendly
  if(NOT OpenCV_FIND_QUIETLY)
    if(OpenCV_FIND_REQUIRED)
      message(FATAL_ERROR "OpenCV required but some headers or libs not found. Please specify it's location with OpenCV_ROOT_DIR env. variable.")
    else(OpenCV_FIND_REQUIRED)
      message(STATUS "ERROR: OpenCV was not found.")
    endif(OpenCV_FIND_REQUIRED)
  endif(NOT OpenCV_FIND_QUIETLY)
endif(NOT OpenCV_FOUND)
