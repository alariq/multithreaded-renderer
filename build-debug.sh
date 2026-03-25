#!/bin/sh

cmake -B ./build/ -S . -DCMAKE_BUILD_TYPE=Debug -DUSE_TRACY=OFF -DWITH_ASAN=ON -DUSE_IMGUI=ON
cmake --build ./build --parallel

