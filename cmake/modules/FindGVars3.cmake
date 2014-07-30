# - Find GVars3 (GVars3 is hosted at http://mi.eng.cam.ac.uk/~er258/gvars3/)
#
#  This code sets the following variables (OUTPUT):
#
#  GVars3_FOUND:            indicate the lib is found
#  GVars3_INCLUDE_DIR:      path to the TaG header files
#  GVars3_LIBRARIES:        libs required
#

find_path(GVars3_INCLUDE_DIR NAMES gvars3/instances.h
  PATHS
  "$ENV{GVars3_ROOT_DIR}/include"
  /usr/local/include/)

if(GVars3_INCLUDE_DIR)
 set(GVars3_FOUND ON)
endif(GVars3_INCLUDE_DIR)

find_library(GVars3_LIBRARY NAMES GVars3 PATHS $ENV{GVars3_ROOT_DIR} /usr/local/lib)

set(GVars3_LIBRARIES ${GVars3_LIBRARY})

mark_as_advanced(
  GVars3_INCLUDE_DIR
  GVars3_LIBRARIES
)
