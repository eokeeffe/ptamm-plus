cmake_minimum_required(VERSION 2.8)

project(ptam_ui)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

# =============================== Headers =============================
set(PTAM_UI_H
    ar_driver.h
    ar_render.h
    draw_helpers.h
    eye_game.h
    gl_window_menu.h
    gl_window2.h
    gl_window.h
    map_viewer.h
    open_gl.h
)

# =============================== Sources =============================
set(PTAM_UI_SRC
    ar_driver.cc
    ar_render.cc
    eye_game.cc
    gl_window_menu.cc
    gl_window2.cc
    map_viewer.cc
)

# =============================== Library ======================================
add_library(ptam_ui ${PTAM_UI_SRC} ${PTAM_UI_H})
target_link_libraries(ptam_ui ${EXTRA_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
