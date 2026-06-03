#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "Usage: ./scripts/run_macos.sh <image_path> [output_directory]"
    exit 1
fi

if [ ! -x build/RoadSignDetector ]; then
    ./scripts/build_macos.sh
fi

if [ "$#" -ge 2 ]; then
    ./build/RoadSignDetector "$1" "$2"
else
    ./build/RoadSignDetector "$1"
fi
