#!/bin/sh

cmake -B ./build/ -S . -DCMAKE_BUILD_TYPE=Debug -DUSE_TRACY=ON -DWITH_ASAN=OFF -DUSE_IMGUI=ON
cmake --build ./build --parallel

