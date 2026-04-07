#!/bin/bash


# https://github.com/aras-p/ClangBuildAnalyzer.git


rm -rf /tmp/clang-traces
mkdir /tmp/clang-traces
ClangBuildAnalyzer --start /tmp/clang-traces
./build-debug.sh
ClangBuildAnalyzer --stop /tmp/clang-traces traces-capture.bin
ClangBuildAnalyzer --analyze traces-capture.bin


