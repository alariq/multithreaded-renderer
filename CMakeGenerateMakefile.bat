@echo off

REM Use: CMakeGenerate [cmake parameters]
REM Example: CMakeGenerate -D<var>=<value>...
REM Currently only one platform is supported: NX64

rem call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat"
call "c:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" 

cmake -G "NMake Makefiles" %* -B cmake-nmake -S .  -DCMAKE_PREFIX_PATH=d:/prog/3rdparty
GOTO END

:END
PAUSE
