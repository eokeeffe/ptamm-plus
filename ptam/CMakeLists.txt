cmake_minimum_required(VERSION 2.8)

project(ptam)

include_directories(${EXTRA_INC_DIRS})

# ==============================================================================
add_subdirectory(construct)
add_subdirectory(math)
add_subdirectory(sensor)
add_subdirectory(track)

option(WITH_UI "Build Library with UI" ON)
if(WITH_UI)
  add_subdirectory(ui)
endif(WITH_UI)

add_subdirectory(util)

# ==============================================================================
# Optional Configuration
option(WITH_TEST "Build Library with Tests" ON)

if(WITH_TEST)
  #enable_testing() # has to be in root CMakeLists.txt for target 'test' to work!
  add_subdirectory(test)
endif(WITH_TEST)
