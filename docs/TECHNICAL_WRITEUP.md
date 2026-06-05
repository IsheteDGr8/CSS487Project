# Technical Write-Up — Road Sign and Traffic Light Detection

**Course:** CSS 487 Computer Vision  
**Team:** Ishaan, Manish Ram  
**Project:** Classical road sign and traffic light detection using OpenCV 4  

---

## 1. Objective

Build a real-time-capable computer vision system that detects common US road signs and
traffic lights from dashcam-style images and video **without** pretrained neural networks.
The system should be explainable, modular, and demonstrable in class.

### Goals

- Classify road signs by **color + shape** (macro categories aligned with MUTCD conventions).
- Detect **red, yellow, and green traffic-light bulbs** in still images and video.
- Merge two team pipelines without duplicate or conflicting bounding boxes.
- Provide a repeatable demo with organized test data and batch scripts.

---

## 2. Approach

### 2.1 Road sign pipeline (Ishaan)

| Step | Method | OpenCV API |
|---|---|---|
| Color isolation | Fixed HSV thresholds per sign color | `cvtColor`, `inRange`, morphology |
| Shape analysis | Contour area, aspect ratio, polygon vertices | `findContours`, `approxPolyDP`, `convexHull` |
| Speed limit OCR | White panel mask + digit template matching | `matchTemplate`, Otsu threshold |
| Output | Bounding box + text label on frame | `rectangle`, `putText` |

**Design choice:** One detector runs per test folder (e.g. regulatory images only call
`detectRegulatorySigns`) to avoid labeling sky/trees as service or guide signs.

### 2.2 Traffic-light pipeline (Manish)

| Step | Method | OpenCV API |
|---|---|---|
| Path A | HSV bulb search + roundness + housing checks | `TrafficLightFinder` |
| Path B | HSV masks → contours → Hough circles | `HoughCircles`, `RoadObjectDetector` |
| Fusion | Merge detections, sort by confidence, show top 2 | `chooseStrongestDetections` |

**Design choice:** Dual-path detection restores accuracy from Manish's original work while
sign logic remains in `ShapeAnalyzer`.

### 2.3 Integration (`main.cpp`)

- `PipelineMode`: `SignsOnly`, `TrafficLightsOnly`, or `Both`.
- `SignCategory`: selects which sign detector runs per folder.
- Dashcam video uses `Both` mode with regulatory + warning + service sign detectors.

---

## 3. Results

> **Action before submission:** Insert screenshots from your demo runs below.

### 3.1 Still-image tests

| Category | Expected | Observed |
|---|---|---|
| Stop sign | Red octagon → `STOP SIGN` | *(add screenshot)* |
| Speed limit | White panel + digits | *(add screenshot)* |
| Warning | Yellow diamond | *(add screenshot)* |
| Service | Blue rectangle | *(add screenshot)* |
| Traffic light | Colored bulb label | *(add screenshot)* |

### 3.2 Dashcam video

- Video: `data/road_sign_data/dashcam.mp4`
- FPS overlay shown in corner during playback
- Signs and traffic lights annotated in combined mode

*(Insert frame capture or short description of best/worst frames.)*

### 3.3 Known limitations

- Macro labels (`WARNING SIGN`) rather than specific MUTCD text (e.g. "DEER CROSSING").
- Traffic-light accuracy depends on lighting; small or distant bulbs may be missed.
- Speed-limit OCR assumes two large digits on a white rectangular panel.
- Stress-test images in `stress test signs/` are not included in the automated demo loop.

---

## 4. Architecture diagram

See `docs/architecture-diagram.svg` in the repository.

```
BGR frame
   ├── ColorSegmenter → ShapeAnalyzer     (road signs)
   └── HsvMaskSegmenter → RoadObjectDetector → TrafficLightFinder   (traffic lights)
```

---

## 5. Lessons learned

1. **Run only the relevant detector per test context.** Running all sign detectors on every
   image caused false positives (blue sky → service sign, green trees → guide sign).
2. **Preserve dual-path traffic-light logic** when merging codebases; removing the mask loop
   reduced detection quality even though `TrafficLightFinder` was unchanged.
3. **Pipeline modes** (`SignsOnly` / `TrafficLightsOnly` / `Both`) allow clean integration
   without ghost bounding boxes.
4. **Fixed HSV thresholds** work well on curated test sets but require tuning for new lighting.
5. **Team integration** benefits from a shared `Detection` struct and clear file ownership.

---

## 6. Team contributions

| Member | Contribution |
|---|---|
| Ishaan | `ColorSegmenter`, `ShapeAnalyzer`, sign demo loop, pipeline integration, test data organization |
| Manish Ram | `TrafficLightFinder`, `HsvMaskSegmenter`, `RoadObjectDetector`, `DetectionSelection`, legacy CLI utilities |

---

## 7. How to reproduce

```bat
scripts\build_vs2022.bat
scripts\run_demo.bat
```

Or F5 in VS Code with configuration **RoadSignDetector (full demo)**.

---

## 8. References

- OpenCV 4 documentation: https://docs.opencv.org/4.x/
- Manual on Uniform Traffic Control Devices (MUTCD) — sign shapes and colors
- Course lecture materials on HSV segmentation, contours, and Hough transforms
