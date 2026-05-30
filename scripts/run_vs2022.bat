@echo off
setlocal

if "%~1"=="" (
    echo Usage: scripts\run_vs2022.bat ^<image_path^> [output_directory]
    exit /b 1
)

if not exist build-vs2022\Release\RoadSignDetector.exe (
    call scripts\build_vs2022.bat
    if errorlevel 1 exit /b 1
)

if "%~2"=="" (
    build-vs2022\Release\RoadSignDetector.exe "%~1"
) else (
    build-vs2022\Release\RoadSignDetector.exe "%~1" "%~2"
)
