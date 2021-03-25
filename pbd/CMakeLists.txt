cmake_minimum_required (VERSION 3.10)
project(pbd)

set(SOURCES ${SOURCES}
    pbd.cpp
    pbd.h
    pbd_constraints.cpp
    pbd_constraints.h
    pbd_rb_dynamics.cpp
    pbd_rb_dynamics.h
    pbd_collision.cpp
    pbd_collision.h
)

add_library(pbd ${SOURCES})
target_link_libraries(pbd engine)

