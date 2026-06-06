#!/bin/sh

cmake -B ./build/ -S . -DCMAKE_BUILD_TYPE=Debug -DUSE_TRACY=ON -DWITH_ASAN=ON -DUSE_IMGUI=ON
cmake --build ./build --parallel

export LSAN_OPTIONS=suppressions=asan.supp
