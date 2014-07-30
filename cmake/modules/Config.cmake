#
# Place to declare some common configuration for this project only
#

# Where to find modules, used when finding packages.
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/;")
message("Reset CMAKE_MODULE_PATH to:${CMAKE_MODULE_PATH}")

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/local/lib)
message("Reset CMAKE_LIBRARY_PATH to:${CMAKE_LIBRARY_PATH}")

link_directories(${CMAKE_LIBRARY_PATH})
