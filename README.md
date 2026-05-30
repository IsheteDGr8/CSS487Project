# Road Sign and Traffic Light Detection

OpenCV project for detecting road signs and traffic lights

The current `Manish` branch contains contour extraction, area filtering, shape
analysis, Hough circle checks, and rule-based heuristics.

## Build on macOS

Open a terminal in this folder:

```bash
./scripts/build_macos.sh
```

Run the detector:

```bash
./build/RoadSignDetector path/to/image.jpg output
```

The optional output folder receives:

- `annotated.png`
- `red_mask.png`
- `yellow_mask.png`
- `green_mask.png`
- `blue_mask.png`

## Build on Visual Studio 2022 Community

Install OpenCV 4 for Windows and set `OpenCV_DIR` to the folder containing
`OpenCVConfig.cmake`. Example:

```bat
set OpenCV_DIR=C:\opencv\build
```

Then run:

```bat
scripts\build_vs2022.bat
```

Run the detector:

```bat
build-vs2022\Release\RoadSignDetector.exe path\to\image.jpg output
```

Visual Studio can also open this folder directly as a CMake project.

## Run Tests

The smoke test creates synthetic masks for a stop sign, yield sign, warning
sign, and red traffic light bulb.

```bash
ctest --test-dir build --output-on-failure
```

On Windows after `scripts\build_vs2022.bat`:

```bat
ctest --test-dir build-vs2022 -C Release --output-on-failure
```

## Project Layout

- `include/RoadSignDetector/DetectionTypes.hpp` shared enum and result structs.
- `include/RoadSignDetector/RoadObjectDetector.hpp` detector API.
- `src/RoadObjectDetector.cpp` contours, area filters, shape analysis, Hough circles, and heuristics.
- `src/HsvMaskSegmenter.cpp` simple HSV mask generator for standalone testing.
- `src/DisplayOnTerminal.cpp` console table output.
- `src/DebugImageWriter.cpp` annotated image and mask output.
- `tests/detector_smoke_test.cpp` no-GUI algorithm smoke tests.

## Detection Pipeline

1. Receive one binary mask per HSV color family.
2. Normalize each mask to foreground/background pixels.
3. Extract external contours with `cv::findContours`.
4. Filter tiny noise using contour area limits.
5. Measure perimeter, bounding box, aspect ratio, circularity, and vertices.
6. Approximate polygons with `cv::approxPolyDP`.
7. Search each candidate region with `cv::HoughCircles`.
8. Apply rules such as red octagon -> stop sign, red triangle -> yield sign,
   yellow square/diamond -> warning sign, and circular colored bulb -> traffic light.
