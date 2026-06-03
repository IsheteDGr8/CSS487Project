# Road Sign and Traffic Light Detection

Classical OpenCV project for detecting road signs and traffic lights without
pretrained machine learning models.

The current `Manish` branch contains contour extraction, area filtering, shape
analysis, Hough circle checks, basic text reading, and rule-based heuristics.

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

## Project Layout

- `include/RoadSignDetector/DetectionTypes.hpp` shared enum and result structs.
- `include/RoadSignDetector/RoadObjectDetector.hpp` detector API.
- `src/RoadObjectDetector.cpp` contours, area filters, shape analysis, Hough circles, and heuristics.
- `src/TextReader.cpp` template based text and speed number reading.
- `src/SignPictureAnalyzer.cpp` icon clues such as red slash, white cross, and dark symbols.
- `src/HsvMaskSegmenter.cpp` simple HSV mask generator for standalone testing.
- `src/DisplayOnTerminal.cpp` console table output.
- `src/DebugImageWriter.cpp` annotated image and mask output.

## Detection Pipeline

1. Receive one binary mask per HSV color family.
2. Normalize each mask to foreground/background pixels.
3. Extract external contours with `cv::findContours`.
4. Filter tiny noise using contour area limits.
5. Measure perimeter, bounding box, aspect ratio, circularity, and vertices.
6. Approximate polygons with `cv::approxPolyDP`.
7. Search each candidate region with `cv::HoughCircles`.
8. Read simple sign text and speed numbers using generated OpenCV font templates.
9. Apply rules such as red octagon -> stop sign, red circle plus digits -> speed
   sign, red circle plus slash and arrow -> turn restriction, blue circle plus
   white arrow -> keep direction, red triangle plus train symbols -> railway
   crossing, and circular colored bulb -> traffic light.

## Current Labels

The detector can currently label:

- stop sign
- speed limit sign with a number
- no U-turn sign
- no left turn sign
- no right turn sign
- keep left sign
- keep right sign
- railway crossing sign
- falling rocks sign
- road narrows sign
- pedestrian crossing sign
- bicycle crossing sign
- ferry sign
- animal crossing sign
- first aid sign
- no horn sign
- no entry sign
- safety first sign
- circular sign
- red, yellow, and green traffic lights
