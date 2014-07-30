cmake_minimum_required(VERSION 2.8)

project(ptam_construct)

# =============================== Headers =============================
set(PTAM_CONSTRUCT_H
    bundle.h
    keyframe.h
    lapack_grap.h
    map_maker.h
    map_point.h
    map.h
    shi_tomasi.h
    small_blurry_image.h
)

# =============================== Sources =============================
set(PTAM_CONSTRUCT_SRC
    bundle.cc
    keyframe.cc
    map_maker.cc
    map_point.cc
    map.cc
    shi_tomasi.cc
    small_blurry_image.cc
)

# =============================== Library ======================================
add_library(ptam_construct ${PTAM_CONSTRUCT_SRC} ${PTAM_CONSTRUCT_H})
target_link_libraries(ptam_construct ${EXTRA_LIBRARIES})
