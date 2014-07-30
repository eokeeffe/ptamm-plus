cmake_minimum_required(VERSION 2.8)

project(ptam_math)

# =============================== Headers =============================
set(PTAM_MATH_H
    atan_camera.h
)

# =============================== Sources =============================
set(PTAM_MATH_SRC
    atan_camera.cc
)

# =============================== Library ======================================
add_library(ptam_math ${PTAM_MATH_SRC} ${PTAM_MATH_H})
target_link_libraries(ptam_math ${EXTRA_LIBRARIES})
