project(apps)


if(WITH_UI)
##-----------------------------------------------------------------------------
# Optional Apps

option(APP_CALIBRATION "Build Calibration" ON)
if(APP_CALIBRATION)
  add_subdirectory(calibration)
endif(APP_CALIBRATION)

option(APP_SLAM "Build Slam" ON)
if(APP_SLAM)
  add_subdirectory(slam)
endif(APP_SLAM)

endif(WITH_UI)
