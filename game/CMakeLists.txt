cmake_minimum_required (VERSION 3.10)
project(game)

include_directories(${CMAKE_SOURCE_DIR}/engine)
include_directories(${CMAKE_SOURCE_DIR}/platform/include)

set(SOURCES ${SOURCES}
    ShipController.cpp
    ShipController.h
    WingController.cpp
    WingController.h
    MainShip.cpp
    MainShip.h
    Level.cpp
    Level.h
    Text.cpp
    Text.h
    Enemy.cpp
    Enemy.h
    Path.cpp
    Path.h
    Time.cpp
    Time.h
    Billboard.cpp
    Billboard.h

)

add_library(game ${SOURCES})
target_link_libraries(game engine)

