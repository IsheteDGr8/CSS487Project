@echo off
setlocal

REM Runs the full folder + dashcam demo from the project root.
REM Optional flags: --signs-only  --lights-only  --both

if not exist build\Debug\RoadSignDetector.exe (
    call scripts\build_vs2022.bat
    if errorlevel 1 exit /b 1
)

cd /d "%~dp0.."
build\Debug\RoadSignDetector.exe %*
