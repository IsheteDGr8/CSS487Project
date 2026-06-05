#!/usr/bin/env bash
set -euo pipefail

# Build RoadSignDetector on macOS (CMake + OpenCV 4).
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
echo "Built build/RoadSignDetector"
