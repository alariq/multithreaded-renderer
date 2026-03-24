#!/bin/sh

cmake -B ./build/ -S . -DCMAKE_BUILD_TYPE=Release -DUSE_TRACY=OFF -DWITH_ASAN=OFF
cmake --build ./build --parallel

