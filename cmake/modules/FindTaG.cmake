# - Find TaG (TaG is hosted at http://mi.eng.cam.ac.uk/~er258/tag/)
#
#  This code sets the following variables (OUTPUT):
#
#  TaG_FOUND:            indicate the lib is found
#  TaG_INCLUDE_DIR:      path to the TaG header files
#  Tag_LIBRARIES:        libs required
#

find_path(TaG_INCLUDE_DIR NAMES /tag/helpers.h
  PATHS
  "$ENV{TaG_ROOT_DIR}/include"
  /usr/local/include/)

if(TaG_INCLUDE_DIR)
 set(TaG_FOUND ON)
endif()

find_library(TaG_LIBRARY NAMES toontag PATHS $ENV{TaG_ROOT_DIR} /usr/local/lib)

set(TaG_LIBRARIES ${TaG_LIBRARY})

mark_as_advanced(
  TaG_INCLUDE_DIR
  TaG_LIBRARIES
)
