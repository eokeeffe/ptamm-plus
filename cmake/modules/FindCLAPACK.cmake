# - try to find clapack library and include files
#  CLAPACK_LIBRARIES, the libraries to link against
#  CLAPACK_FOUND, If false, do not try to use CLAPACK.

find_library( CLAPACK_LIBRARY
  NAMES clapack
  PATHS
  $ENV{CLAPACK_ROOT_DIR}/lib
  ${CLAPACK_ROOT_DIR}/lib
  /user/lib
  /opt/local/lib
  )

set( CLAPACK_FOUND "NO" )
if(CLAPACK_LIBRARY)
  set( CLAPACK_LIBRARIES ${CLAPACK_LIBRARY} )
  set( CLAPACK_FOUND "YES" )
endif(CLAPACK_LIBRARY)

mark_as_advanced(
  CLAPACK_LIBRARIES
  )
