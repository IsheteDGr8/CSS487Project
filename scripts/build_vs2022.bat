@echo off
setlocal

REM Build RoadSignDetector with Visual Studio 2022 (CMake).
REM Output: build\Debug\RoadSignDetector.exe or build\Release\RoadSignDetector.exe

if defined OpenCV_DIR (
    cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DOpenCV_DIR="%OpenCV_DIR%"
) else (
    cmake -S . -B build -G "Visual Studio 17 2022" -A x64
)

if errorlevel 1 exit /b 1

cmake --build build --config Debug
if errorlevel 1 exit /b 1

echo Built build\Debug\RoadSignDetector.exe
echo Run: scripts\run_demo.bat
