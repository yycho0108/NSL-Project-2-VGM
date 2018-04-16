cmake_minimum_required(VERSION 3.1)
project(nsl_vgm)
set (CMAKE_CXX_STANDARD 11)

add_definitions( -DFREEGLUT_EXPORTS -DFREEGLUT_STATIC
                 -DGLUT_DISABLE_ATEXIT_HACK -D_CRT_SECURE_NO_WARNINGS )

find_package(Box2D REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

add_definitions     ( ${BOX2D_DEFINITIONS} )
include_directories ( ${BOX2D_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR} )
link_directories    ( ${BOX2D_LIBRARY_DIRS} )

add_executable(nsl_vgm main.cpp Render.cpp)
target_link_libraries(nsl_vgm liquidfun glut ${OPENGL_LIBRARIES}) #${BOX2D_LIBRARY})
