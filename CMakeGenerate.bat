@echo off

REM Use: CMakeGenerate [cmake parameters]
REM Example: CMakeGenerate -D<var>=<value>...
REM Currently only one platform is supported: NX64

SET TARGET_BUILD_PLATFORM=x64
cmake -G "Visual Studio 15 2017" -A %TARGET_BUILD_PLATFORM% %* -B cmake-%TARGET_BUILD_PLATFORM% -S .  -DCMAKE_PREFIX_PATH=../../3rdparties
GOTO END

:END
PAUSE
