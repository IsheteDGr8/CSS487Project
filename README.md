# Road Sign and Traffic Light Detection

Classical computer-vision project (OpenCV 4, C++17) that detects **US-style road signs**
and **traffic lights** without machine learning.

**Team:** Ishaan (road signs) and Manish Ram (traffic lights)

## Features

| Component | Classes | Detects |
|---|---|---|
| Road signs | `ColorSegmenter`, `ShapeAnalyzer` | Construction, guide, service, warning, regulatory (stop, do-not-enter, speed limit) |
| Traffic lights | `HsvMaskSegmenter`, `RoadObjectDetector`, `TrafficLightFinder` | Red, yellow, and green bulbs |

The demo runs **folder-based still-image tests** followed by a **dashcam video**. Pipeline
modes keep sign and traffic-light detectors separate during static tests to avoid false
positives, then combine both on video.

## Requirements

- Visual Studio 2022 with C++ desktop development
- OpenCV 4 for Windows (`OpenCV_DIR` → folder containing `OpenCVConfig.cmake`)
- CMake 3.20+

## Build (Windows)

Set OpenCV if needed:

```bat
set OpenCV_DIR=C:\opencv\build
```

Build:

```bat
scripts\build_vs2022.bat
```

Or open this folder in VS Code / Visual Studio and press **F7** (CMake build to `build/`).

## Run (Windows)

From the project root:

```bat
scripts\run_demo.bat
```

Optional pipeline overrides:

```bat
scripts\run_demo.bat --signs-only
scripts\run_demo.bat --lights-only
scripts\run_demo.bat --both
```

### Visual Studio Code

Use **Run and Debug** → **RoadSignDetector (full demo)** (F5). Working directory must be
the project root so `data/road_sign_data/` resolves correctly.

### Controls

| Phase | Action |
|---|---|
| Still-image tests | Press **any key** to advance |
| Dashcam video | Press **ESC** to exit |

## Build and run (macOS)

```bash
./scripts/build_macos.sh
./scripts/run_macos.sh
```

## Test data layout

```
data/road_sign_data/
  construction signs/
  guide signs/
  service signs/
  warning signs/
  regulatory signs/     stop_*, no_entry_*, speed_limit_*
  traffic lights/
  dashcam.mp4
```

## Project layout

```
include/RoadSignDetector/
  ColorSegmenter.h          HSV masks for sign colors
  ShapeAnalyzer.h           Shape-based sign classification
  DetectionTypes.hpp        Shared enums and Detection struct
  HsvMaskSegmenter.hpp      HSV masks for traffic-light colors
  RoadObjectDetector.hpp    Traffic-light facade (dual-path)
  TrafficLightFinder.hpp    Bulb finder (HSV + Hough)
  DetectionSelection.hpp    Top-N detection filtering
  DisplayOnTerminal.hpp     Console table output (legacy CLI)
  DebugImageWriter.hpp      Annotated PNG output (legacy CLI)

src/
  main.cpp                  Demo entry point
  ColorSegmenter.cpp
  ShapeAnalyzer.cpp
  RoadObjectDetector.cpp
  TrafficLightFinder.cpp
  HsvMaskSegmenter.cpp
  DetectionSelection.cpp
  DisplayOnTerminal.cpp
  DebugImageWriter.cpp
  TextReader.cpp            OCR helpers (used by legacy Manish pipeline)

scripts/
  build_vs2022.bat          Build on Windows
  run_demo.bat              Run full demo on Windows
  build_macos.sh / run_macos.sh

docs/
  TECHNICAL_WRITEUP.md      Report template for Canvas submission
  architecture-diagram.svg  Pipeline overview
```

## Detection pipeline

### Road signs (Ishaan)

1. `ColorSegmenter` builds per-color HSV masks (red, yellow, blue, orange, green, white).
2. `ShapeAnalyzer` finds contours, checks shape rules, and draws labels.
3. Each test folder runs **only its matching detector** (e.g. regulatory folder does not
   scan for blue service signs in the background).

### Traffic lights (Manish)

1. `TrafficLightFinder` searches the full frame for bright circular bulbs.
2. `HsvMaskSegmenter` masks feed contour + Hough analysis in `RoadObjectDetector`.
3. Top **two** detections by confidence are drawn (matching original demo behavior).

### Dashcam video

Runs regulatory + warning + service sign detectors plus traffic-light detection (`Both` mode).

## Sign labels produced

| Category | Label on frame |
|---|---|
| Regulatory | `STOP SIGN`, `DO NOT ENTER`, `SPEED LIMIT ##` |
| Warning | `WARNING SIGN` |
| Construction | `CONSTRUCTION SIGN` |
| Guide | `GUIDE SIGN` |
| Service | `SERVICE SIGN` |
| Traffic light | `RED_TRAFFIC_LIGHT`, `YELLOW_TRAFFIC_LIGHT`, `GREEN_TRAFFIC_LIGHT` |

## Submission checklist

- [ ] Working source (this repo, `Ishaan` branch)
- [ ] Test images (`data/road_sign_data/`)
- [ ] Batch scripts or documented IDE run (above)
- [ ] Technical write-up (`docs/TECHNICAL_WRITEUP.md`)
- [ ] Presentation slides
- [ ] Zip submitted on Canvas by one team member

## References

- OpenCV 4 documentation: https://docs.opencv.org/4.x/
- US MUTCD sign shapes and colors (assumed for heuristic rules)
