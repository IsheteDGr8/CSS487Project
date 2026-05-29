@echo off
setlocal

if defined OpenCV_DIR (
    cmake -S . -B build-vs2022 -G "Visual Studio 17 2022" -A x64 -DOpenCV_DIR="%OpenCV_DIR%"
) else (
    cmake -S . -B build-vs2022 -G "Visual Studio 17 2022" -A x64
)

if errorlevel 1 exit /b 1

cmake --build build-vs2022 --config Release
if errorlevel 1 exit /b 1

echo Built build-vs2022\Release\RoadSignDetector.exe
