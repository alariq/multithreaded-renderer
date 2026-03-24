#!/bin/sh

cmake -B ./build/ -S . -DCMAKE_BUILD_TYPE=Debug -DUSE_TRACY=OFF -DWITH_ASAN=ON
cmake --build ./build --parallel

