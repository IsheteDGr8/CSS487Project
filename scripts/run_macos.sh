#!/usr/bin/env bash
set -euo pipefail

# Runs the full folder + dashcam demo from the project root.
# Optional flags: --signs-only  --lights-only  --both

if [ ! -x build/RoadSignDetector ]; then
    ./scripts/build_macos.sh
fi

cd "$(dirname "$0")/.."
./build/RoadSignDetector "$@"
