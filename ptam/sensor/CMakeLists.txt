cmake_minimum_required(VERSION 2.8)

project(ptam_sensor)

# =============================== Headers =============================
set(PTAM_SENSOR_H
    video_source.h
)

# =============================== Sources =============================
set(PTAM_SENSOR_SRC
    #video_source_linux_dv.cc
    video_source_linux_v4l.cc
)

# =============================== Library ======================================
add_library(ptam_sensor ${PTAM_SENSOR_SRC} ${PTAM_SENSOR_H})
target_link_libraries(ptam_sensor ${EXTRA_LIBRARIES})
