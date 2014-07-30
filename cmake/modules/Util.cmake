# Contains common (useful) cmake macros or routines

# useful routine for building a binary
# requires ${ALL_LIBRARRIES}
MACRO(common_build_routine name src)
    set(srcs ${src} ${ARGN})
    add_executable(${name} ${srcs})
    if(ALL_LIBRARIES)
      add_dependencies(${name} ${ALL_LIBRARIES})
      target_link_libraries(${name} ${ALL_LIBRARIES})
    endif(ALL_LIBRARIES)
ENDMACRO(common_build_routine)


# commong build settings (mostly to deal with Windows )
MACRO(common_build_setting)
if(WIN32)
  add_definitions(-DGL_GLEXT_PROTOTYPES=1    # OpenGL extension
                  -DUSE_EIGEN_OPENGLSUPPORT  # to use OpenGL util functions
                  -D_ITERATOR_DEBUG_LEVEL=0  # this is for boost built compatability (with thirtparty libs:freeglut, glew, etc)
                  -DEIGEN_DONT_ALIGN_STATICALLY  # this is unusual
                  -DHAVE_GLEW   # this is used in GL/gl.h to include GL/glew.h ontop (quick solution for glew)
                  )
  if(CMAKE_BUILD_TYPE STREQUAL Debug)
    #add_definitions(-D_ITERATOR_DEBUG_LEVEL=2)
  else()
    add_definitions(/O2 /arch:SSE2 /arch:SSE3 -D_SECURE_SCL=0)
    # this deals with unexpected Windows default compiler flags
    if(CMAKE_CXX_FLAGS MATCHES "/RTC1")
     string(REPLACE "/RTC1" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
     message(STATUS ${PROJECT_NAME}  " CMAKE_CXX_FLAGS removing /RTC1")
    endif()
  endif()
  set(EXTRA_INC_DIRS ${EXTRA_INC_DIRS} $ENV{OPENGL_ROOT}/include)
  STRING (REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  # trick from http://www.cmake.org/pipermail/cmake/2010-December/041274.html
  # to build INCREMENTAL on 
  SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "/INCREMENTAL:YES")
else(WIN32)
  add_definitions(-DGL_GLEXT_PROTOTYPES=1 -DUSE_EIGEN_OPENGLSUPPORT)
  if(CMAKE_BUILD_TYPE STREQUAL Debug)
    add_definitions(-g -O0)
  else()
    add_definitions(-O3 -msse3)
  endif()
endif(WIN32)
ENDMACRO(common_build_setting)
