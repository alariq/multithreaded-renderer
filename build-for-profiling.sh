#!/bin/sh

cmake -B ./build/ -S . -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_TRACY=ON -DWITH_ASAN=OFF -DTRACY_CALLSTACK=ON -DTRACY_CALLSTACK=16
cmake --build ./build --parallel

