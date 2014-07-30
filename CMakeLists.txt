cmake_minimum_required(VERSION 2.8)

project(ptam_and_apps)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

if(WIN32)
  add_definitions(-DGL_GLEXT_PROTOTYPES=1 -DUSE_EIGEN_OPENGLSUPPORT
                  -D_ITERATOR_DEBUG_LEVEL=0  # this is for boost built compatability (with thirtparty libs:freeglut, glew, etc)
                  -DEIGEN_DONT_ALIGN_STATICALLY  # this is unusual
                  -DHAVE_GLEW   # this is used in GL/gl.h to include GL/glew.h ontop (quick solution for glew)
                  )
  if(CMAKE_BUILD_TYPE STREQUAL Debug)
    #add_definitions(-D_ITERATOR_DEBUG_LEVEL=2)
  else()
    add_definitions(/O2 /arch:SSE2 /arch:SSE3 -D_SECURE_SCL=0)
    if(CMAKE_CXX_FLAGS MATCHES "/RTC1")
     string(REPLACE "/RTC1" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
     message(STATUS ${PROJECT_NAME}  " CMAKE_CXX_FLAGS removing /RTC1")
    endif()
  endif()

  set(EXTRA_INC_DIRS ${EXTRA_INC_DIRS} $ENV{OPENGL_ROOT}/include)
  STRING (REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
else(WIN32)
  add_definitions(-DGL_GLEXT_PROTOTYPES=1 -DUSE_EIGEN_OPENGLSUPPORT)
  if(NOT(CMAKE_BUILD_TYPE STREQUAL Debug))
    add_definitions(-O3 -msse3)
  endif()
endif(WIN32)
## Add directories of own CMake modules
# e.g. define ICG_CMAKE_MODULE_PATH in your environment
file(TO_CMAKE_PATH "$ENV{CMAKE_MODULE_PATH}" OT_CMAKE_MODULE_PATH)
set(CMAKE_MODULE_PATH ${OT_CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules )
include(Config)
include(Util)
message("PTAM cmake modules: " ${CMAKE_MODULE_PATH} )

##-----------------------------------------------------------------------------
# Optional Configuration
option(WITH_UI "Build Library with UI" ON)

##-----------------------------------------------------------------------------
##  LibCVD
find_package(CVD REQUIRED)
set(EXTRA_INC_DIRS
    ${EXTRA_INC_DIRS}
    ${CVD_INCLUDE_DIR}
)
set(EXTRA_LIBRARIES
    ${EXTRA_LIBRARIES}
    ${CVD_LIBRARY}
)
message("Found CVD libs: ${CVD_LIBRARY}")
message("CVD INC DIR: ${CVD_INCLUDE_DIR}")

##-----------------------------------------------------------------------------
## TooN
find_package(TooN REQUIRED)

set(EXTRA_INC_DIRS
    ${EXTRA_INC_DIRS}
    ${TooN_INCLUDE_DIR}
)

set(EXTRA_LIBRARIES ${EXTRA_LIBRARIES} blas lapack)

##-----------------------------------------------------------------------------
## GVars3
find_package(GVars3 REQUIRED)

set(EXTRA_INC_DIRS
    ${EXTRA_INC_DIRS}
    ${GVars3_INCLUDE_DIR}
)
set(EXTRA_LIBRARIES
    ${EXTRA_LIBRARIES}
    ${GVars3_LIBRARY}
)

##-----------------------------------------------------------------------------
## Boost
set(Boost_ADDITIONAL_VERSIONS "1.50.0")
set(Boost_USE_STATIC_LIBS        ON)

find_package(Boost REQUIRED COMPONENTS thread chrono date_time filesystem system serialization)

if(Boost_FOUND)
set(EXTRA_INC_DIRS
    ${EXTRA_INC_DIRS}
    ${Boost_INCLUDE_DIRS}
)
set(EXTRA_LIBRARIES
    ${EXTRA_LIBRARIES}
    ${Boost_LIBRARIES})
message("Found Boost Version ")
message("boost libs: ${Boost_LIBRARIES}")
endif(Boost_FOUND)

if(WITH_UI)
##-----------------------------------------------------------------------------
# dc1394_2
find_package(DC1394)
if(DC1394_2_FOUND)
  add_definitions(-DHAVE_DC1394_2)
  set(EXTRA_INC_DIRS
    ${EXTRA_INC_DIRS}
    ${DC1394_2_INCLUDE_DIR})
  set(EXTRA_LIBRARIES
    ${EXTRA_LIBRARIES}
    ${DC1394_LIBRARIES})
endif(DC1394_2_FOUND)

endif(WITH_UI)

# the lib
set(PTAM_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PTAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ptam)
set(EXTRA_INC_DIRS  ${EXTRA_INC_DIRS} ${PTAM_DIR})
set(PTAM_LIBRARIES ptam_construct ptam_track ptam_math ptam_sensor ptam_ui)

message("EXTRA_INC_DIRS:${EXTRA_INC_DIRS}")

add_subdirectory(ptam)

# applications
add_subdirectory(apps)
