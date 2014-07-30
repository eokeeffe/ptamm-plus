cmake_minimum_required(VERSION 2.8)

project(ptam_track)

# =============================== Headers =============================
set(PTAM_TRACK_H
    calib_corner_patch.h
    calib_image.h
    homography_init.h
    level_helpers.h
    m_estimator.h
    mini_patch.h
    patch_finder.h
    relocaliser.h
    small_matrix_opts.h
    tracker_data.h
    tracker.h
)

# =============================== Sources =============================
set(PTAM_TRACK_SRC
    calib_corner_patch.cc
    calib_image.cc
    homography_init.cc
    mini_patch.cc
    patch_finder.cc
    relocaliser.cc
    tracker.cc
)

# =============================== Library ======================================
add_library(ptam_track ${PTAM_TRACK_SRC} ${PTAM_TRACK_H})
target_link_libraries(ptam_track ${EXTRA_LIBRARIES})
