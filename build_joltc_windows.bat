@echo off
setlocal

set "CONFIG=Distribution"
set "SHARED=OFF"

git submodule init joltc
if errorlevel 1 exit /b %errorlevel%

git submodule update joltc
if errorlevel 1 exit /b %errorlevel%

if not exist build mkdir build

cmake joltc -B build ^
    -DJPH_SAMPLES=OFF ^
    -DJPH_TESTS=OFF ^
    -DJPH_INSTALL=OFF ^
    -DJPH_BUILD_SHARED=%SHARED% ^
    -DINTERPROCEDURAL_OPTIMIZATION=OFF
if errorlevel 1 exit /b %errorlevel%

cmake --build build --config %CONFIG%
if errorlevel 1 exit /b %errorlevel%

if not exist lib mkdir lib

move build\lib\%CONFIG%\* lib\
if errorlevel 1 exit /b %errorlevel%

endlocal
