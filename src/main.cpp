/*
 * File: main.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Authors: Ishaan, Manish Ram
 *
 * Purpose:
 *   Demo entry point for the merged pipeline. Runs folder-based sign tests,
 *   traffic-light image tests, and a dashcam video demo with live annotation.
 *
 * Pipeline architecture:
 *   - Road signs: ColorSegmenter (HSV masks) + ShapeAnalyzer (shape rules).
 *   - Traffic lights: HsvMaskSegmenter + RoadObjectDetector + TrafficLightFinder.
 *   - PipelineMode and SignCategory prevent cross-detector false positives.
 *
 * Pipeline modes (see kDefaultStaticMode / kDefaultVideoMode):
 *   SignsOnly          — sign detectors only
 *   TrafficLightsOnly  — traffic-light detectors only
 *   Both               — both detectors on the same frame
 *
 * CLI overrides (optional):
 *   --signs-only   force SignsOnly on every phase
 *   --lights-only  force TrafficLightsOnly on every phase
 *   --both         force Both on every phase
 *
 * Assumptions:
 *   - OpenCV 4 is installed; working directory is the project root.
 *   - Test data lives under data/road_sign_data/, data/, or build/data/.
 */

#include <cstddef>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "RoadSignDetector/ColorSegmenter.h"
#include "RoadSignDetector/DetectionSelection.hpp"
#include "RoadSignDetector/HsvMaskSegmenter.hpp"
#include "RoadSignDetector/RoadObjectDetector.hpp"
#include "RoadSignDetector/ShapeAnalyzer.h"

namespace fs = std::filesystem;

enum class PipelineMode
{
    SignsOnly,
    TrafficLightsOnly,
    Both
};

// Which sign detector(s) to run — matches the old per-folder test behavior so
// e.g. a stop-sign image does not also scan for blue service / green guide blobs.
enum class SignCategory
{
    None,
    Regulatory,
    Construction,
    Guide,
    Service,
    Warning,
    Dashcam
};

// Flip to PipelineMode::Both to re-enable dual detection on static images
// once TrafficLightFinder accuracy is improved.
constexpr PipelineMode kDefaultStaticMode = PipelineMode::SignsOnly;
constexpr PipelineMode kDefaultVideoMode = PipelineMode::Both;

static const char *pipelineModeLabel(const PipelineMode mode)
{
    switch (mode)
    {
    case PipelineMode::SignsOnly:
        return "SignsOnly";
    case PipelineMode::TrafficLightsOnly:
        return "TrafficLightsOnly";
    case PipelineMode::Both:
        return "Both";
    }

    return "Unknown";
}

static void processSignCategory(cv::Mat &frame,
                                const ColorSegmenter &signSegmenter,
                                const ShapeAnalyzer &signAnalyzer,
                                const SignCategory category)
{
    switch (category)
    {
    case SignCategory::Regulatory:
    {
        const cv::Mat redMask = signSegmenter.getStaticRedMask(frame);
        const cv::Mat whiteMask = signSegmenter.getStaticWhiteMask(frame);
        signAnalyzer.detectRegulatorySigns(frame, redMask, whiteMask, frame);
        break;
    }
    case SignCategory::Construction:
    {
        const cv::Mat orangeMask = signSegmenter.getStaticOrangeMask(frame);
        signAnalyzer.detectConstructionSign(orangeMask, frame);
        break;
    }
    case SignCategory::Guide:
    {
        const cv::Mat greenMask = signSegmenter.getStaticGreenMask(frame);
        signAnalyzer.detectGuideSign(greenMask, frame);
        break;
    }
    case SignCategory::Service:
    {
        const cv::Mat blueMask = signSegmenter.getStaticBlueMask(frame);
        signAnalyzer.detectServiceSign(blueMask, frame);
        break;
    }
    case SignCategory::Warning:
    {
        const cv::Mat yellowMask = signSegmenter.getStaticYellowMask(frame);
        signAnalyzer.detectWarningSign(yellowMask, frame);
        break;
    }
    case SignCategory::Dashcam:
    {
        const cv::Mat redMask = signSegmenter.getStaticRedMask(frame);
        const cv::Mat yellowMask = signSegmenter.getStaticYellowMask(frame);
        const cv::Mat blueMask = signSegmenter.getStaticBlueMask(frame);
        const cv::Mat whiteMask = signSegmenter.getStaticWhiteMask(frame);
        signAnalyzer.detectRegulatorySigns(frame, redMask, whiteMask, frame);
        signAnalyzer.detectWarningSign(yellowMask, frame);
        signAnalyzer.detectServiceSign(blueMask, frame);
        break;
    }
    case SignCategory::None:
        break;
    }
}

static bool isTrafficLightType(const rsd::DetectionType type)
{
    return type == rsd::DetectionType::RedTrafficLight ||
           type == rsd::DetectionType::YellowTrafficLight ||
           type == rsd::DetectionType::GreenTrafficLight;
}

static cv::Scalar trafficLightColor(const rsd::DetectionType type)
{
    switch (type)
    {
    case rsd::DetectionType::RedTrafficLight:
        return cv::Scalar(0, 0, 255);
    case rsd::DetectionType::YellowTrafficLight:
        return cv::Scalar(0, 255, 255);
    case rsd::DetectionType::GreenTrafficLight:
        return cv::Scalar(0, 255, 0);
    default:
        return cv::Scalar(255, 0, 255);
    }
}

static void processTrafficLights(cv::Mat &frame,
                                 const rsd::HsvMaskSegmenter &manishSegmenter,
                                 const rsd::RoadObjectDetector &manishDetector)
{
    const std::vector<rsd::MaskInput> masks = manishSegmenter.createMasks(frame);
    const std::vector<rsd::Detection> detections = manishDetector.detect(frame, masks);

    std::vector<rsd::Detection> trafficOnly;
    trafficOnly.reserve(detections.size());
    for (const rsd::Detection &detection : detections)
    {
        if (isTrafficLightType(detection.type))
        {
            trafficOnly.push_back(detection);
        }
    }

    constexpr std::size_t kMaximumTrafficLightDetections = 2U;
    for (const rsd::Detection &det :
         rsd::chooseStrongestDetections(trafficOnly, kMaximumTrafficLightDetections))
    {
        const cv::Scalar color = trafficLightColor(det.type);
        const cv::Rect &box = det.features.boundingBox;

        cv::rectangle(frame, box, color, 3);
        if (!det.contour.empty())
        {
            cv::drawContours(frame, std::vector<std::vector<cv::Point>>{det.contour}, -1,
                             cv::Scalar(255, 0, 255), 2);
        }

        if (det.features.hasHoughCircle)
        {
            const cv::Point center(
                static_cast<int>(std::lround(det.features.houghCenter.x)),
                static_cast<int>(std::lround(det.features.houghCenter.y)));
            cv::circle(frame, center, static_cast<int>(det.features.houghRadius),
                       cv::Scalar(0, 255, 0), 2);
        }

        const std::string label = rsd::toString(det.type);
        cv::putText(frame, label, cv::Point(box.x, std::max(box.y - 10, 22)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
    }
}

static void processFrame(cv::Mat &frame,
                         const ColorSegmenter &signSegmenter,
                         const ShapeAnalyzer &signAnalyzer,
                         const rsd::HsvMaskSegmenter &manishSegmenter,
                         const rsd::RoadObjectDetector &manishDetector,
                         const PipelineMode mode,
                         const SignCategory signCategory)
{
    switch (mode)
    {
    case PipelineMode::SignsOnly:
        processSignCategory(frame, signSegmenter, signAnalyzer, signCategory);
        break;
    case PipelineMode::TrafficLightsOnly:
        processTrafficLights(frame, manishSegmenter, manishDetector);
        break;
    case PipelineMode::Both:
        if (signCategory != SignCategory::None)
        {
            processSignCategory(frame, signSegmenter, signAnalyzer, signCategory);
        }
        processTrafficLights(frame, manishSegmenter, manishDetector);
        break;
    }
}

static bool parseCliModeOverride(const int argc, char *argv[], PipelineMode &modeOut)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--both")
        {
            modeOut = PipelineMode::Both;
            return true;
        }
        if (arg == "--signs-only")
        {
            modeOut = PipelineMode::SignsOnly;
            return true;
        }
        if (arg == "--lights-only")
        {
            modeOut = PipelineMode::TrafficLightsOnly;
            return true;
        }
    }

    return false;
}

static bool hasSignData(const fs::path &root)
{
    if (!fs::exists(root))
    {
        return false;
    }

    const char *folders[] = {"construction signs", "guide signs", "warning signs",
                             "service signs", "regulatory signs", "traffic lights"};
    for (const char *name : folders)
    {
        if (fs::exists(root / name))
        {
            return true;
        }
    }

    return fs::exists(root / "dashcam.mp4");
}

static fs::path dataRoot()
{
    const fs::path candidates[] = {"data/road_sign_data", "data", "build/data"};
    for (const auto &candidate : candidates)
    {
        if (hasSignData(candidate))
        {
            return candidate;
        }
    }

    for (const auto &candidate : candidates)
    {
        if (fs::exists(candidate))
        {
            return candidate;
        }
    }

    return "data/road_sign_data";
}

static bool isImageFile(const fs::path &path)
{
    if (!path.has_extension())
    {
        return false;
    }

    const std::string ext = path.extension().string();
    return ext == ".jpg" || ext == ".jpeg" || ext == ".png";
}

static void resizeStatic(cv::Mat &frame, const int staticWidth)
{
    const int staticHeight =
        static_cast<int>(frame.rows * (static_cast<double>(staticWidth) / frame.cols));
    cv::resize(frame, frame, cv::Size(staticWidth, staticHeight));
}

static bool isTargetRegulatoryImage(const fs::path &path)
{
    const std::string stem = path.stem().string();
    return stem.rfind("stop_", 0) == 0 || stem.rfind("no_entry_", 0) == 0 ||
           stem.rfind("speed_limit_", 0) == 0;
}

static void showStaticFrame(const std::string &windowTitle, const cv::Mat &frame)
{
    constexpr int topPad = 50;
    constexpr int displayWidth = 800;

    cv::Mat padded;
    cv::copyMakeBorder(frame, padded, topPad, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(30, 30, 30));

    const int displayHeight =
        static_cast<int>(padded.rows * (static_cast<double>(displayWidth) / padded.cols));
    cv::Mat display;
    cv::resize(padded, display, cv::Size(displayWidth, displayHeight));

    cv::namedWindow(windowTitle, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowTitle, displayWidth, displayHeight);
    cv::imshow(windowTitle, display);
    cv::waitKey(0);
    cv::destroyWindow(windowTitle);
}

static void runFolderImageTests(const fs::path &folder,
                                const std::string &sectionLabel,
                                const std::string &windowPrefix,
                                ColorSegmenter &signSegmenter,
                                ShapeAnalyzer &signAnalyzer,
                                rsd::HsvMaskSegmenter &manishSegmenter,
                                rsd::RoadObjectDetector &manishDetector,
                                const int staticWidth,
                                const PipelineMode mode,
                                const SignCategory signCategory,
                                const bool filterRegulatory = false)
{
    if (!fs::exists(folder))
    {
        std::cerr << "Skipping " << sectionLabel << " — folder not found: " << folder.string()
                  << std::endl;
        return;
    }

    std::cout << "--- " << sectionLabel << " (press any key for next) ---" << std::endl;
    std::cout << "Mode: " << pipelineModeLabel(mode) << std::endl;

    for (const auto &entry : fs::directory_iterator(folder))
    {
        if (!entry.is_regular_file() || !isImageFile(entry.path()))
        {
            continue;
        }

        if (filterRegulatory && !isTargetRegulatoryImage(entry.path()))
        {
            continue;
        }

        cv::Mat image = cv::imread(entry.path().string());
        if (image.empty())
        {
            std::cerr << "Failed to load: " << entry.path().string() << std::endl;
            continue;
        }

        resizeStatic(image, staticWidth);
        processFrame(image, signSegmenter, signAnalyzer, manishSegmenter, manishDetector, mode,
                     signCategory);

        const std::string windowTitle = windowPrefix + entry.path().filename().string();
        std::cout << windowTitle << std::endl;
        showStaticFrame(windowTitle, image);
    }
}

static void runConstructionFolderTests(ColorSegmenter &signSegmenter,
                                       ShapeAnalyzer &signAnalyzer,
                                       rsd::HsvMaskSegmenter &manishSegmenter,
                                       rsd::RoadObjectDetector &manishDetector,
                                       const fs::path &root,
                                       const int staticWidth,
                                       const PipelineMode mode)
{
    runFolderImageTests(root / "construction signs", "CONSTRUCTION SIGNS", "Construction - ",
                        signSegmenter, signAnalyzer, manishSegmenter, manishDetector, staticWidth,
                        mode, SignCategory::Construction);
}

static void runGuideFolderTests(ColorSegmenter &signSegmenter,
                                ShapeAnalyzer &signAnalyzer,
                                rsd::HsvMaskSegmenter &manishSegmenter,
                                rsd::RoadObjectDetector &manishDetector,
                                const fs::path &root,
                                const int staticWidth,
                                const PipelineMode mode)
{
    runFolderImageTests(root / "guide signs", "GUIDE SIGNS", "Guide - ",
                        signSegmenter, signAnalyzer, manishSegmenter, manishDetector, staticWidth,
                        mode, SignCategory::Guide);
}

static void runServiceFolderTests(ColorSegmenter &signSegmenter,
                                  ShapeAnalyzer &signAnalyzer,
                                  rsd::HsvMaskSegmenter &manishSegmenter,
                                  rsd::RoadObjectDetector &manishDetector,
                                  const fs::path &root,
                                  const int staticWidth,
                                  const PipelineMode mode)
{
    runFolderImageTests(root / "service signs", "SERVICE SIGNS", "Service - ",
                        signSegmenter, signAnalyzer, manishSegmenter, manishDetector, staticWidth,
                        mode, SignCategory::Service);
}

static void runWarningFolderTests(ColorSegmenter &signSegmenter,
                                  ShapeAnalyzer &signAnalyzer,
                                  rsd::HsvMaskSegmenter &manishSegmenter,
                                  rsd::RoadObjectDetector &manishDetector,
                                  const fs::path &root,
                                  const int staticWidth,
                                  const PipelineMode mode)
{
    runFolderImageTests(root / "warning signs", "WARNING SIGNS", "Warning - ",
                        signSegmenter, signAnalyzer, manishSegmenter, manishDetector, staticWidth,
                        mode, SignCategory::Warning);
}

static void runRegulatoryFolderTests(ColorSegmenter &signSegmenter,
                                     ShapeAnalyzer &signAnalyzer,
                                     rsd::HsvMaskSegmenter &manishSegmenter,
                                     rsd::RoadObjectDetector &manishDetector,
                                     const fs::path &root,
                                     const int staticWidth,
                                     const PipelineMode mode)
{
    runFolderImageTests(root / "regulatory signs",
                        "REGULATORY SIGNS: STOP / DO NOT ENTER / SPEED LIMIT",
                        "Regulatory - ", signSegmenter, signAnalyzer, manishSegmenter, manishDetector,
                        staticWidth, mode, SignCategory::Regulatory, true);
}

static void runTrafficLightFolderTests(ColorSegmenter &signSegmenter,
                                       ShapeAnalyzer &signAnalyzer,
                                       rsd::HsvMaskSegmenter &manishSegmenter,
                                       rsd::RoadObjectDetector &manishDetector,
                                       const fs::path &root,
                                       const int staticWidth,
                                       const PipelineMode mode)
{
    runFolderImageTests(root / "traffic lights", "TRAFFIC LIGHTS", "Traffic Light - ",
                        signSegmenter, signAnalyzer, manishSegmenter, manishDetector, staticWidth,
                        mode, SignCategory::None);
}

static bool runDashcamVideo(const fs::path &videoPath,
                            ColorSegmenter &signSegmenter,
                            ShapeAnalyzer &signAnalyzer,
                            rsd::HsvMaskSegmenter &manishSegmenter,
                            rsd::RoadObjectDetector &manishDetector,
                            const int videoWidth,
                            const PipelineMode mode)
{
    cv::VideoCapture cap(videoPath.string());
    if (!cap.isOpened())
    {
        std::cerr << "Video failed to load: " << videoPath.string() << std::endl;
        return false;
    }

    const std::string windowTitle = "Video Demo - " + videoPath.filename().string();
    cv::Mat frame;
    double fps = 0.0;

    while (true)
    {
        const int64 startTick = cv::getTickCount();
        cap >> frame;

        if (frame.empty())
        {
            std::cout << "Finished: " << videoPath.filename().string() << std::endl;
            break;
        }

        cv::resize(frame, frame,
                   cv::Size(videoWidth,
                            static_cast<int>(frame.rows *
                                             (static_cast<double>(videoWidth) / frame.cols))));

        processFrame(frame, signSegmenter, signAnalyzer, manishSegmenter, manishDetector, mode,
                     SignCategory::Dashcam);

        const int64 endTick = cv::getTickCount();
        fps = cv::getTickFrequency() / (endTick - startTick);
        cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(fps)), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        cv::imshow(windowTitle, frame);

        if (cv::waitKey(30) == 27)
        {
            cap.release();
            cv::destroyWindow(windowTitle);
            return true;
        }
    }

    cap.release();
    cv::destroyWindow(windowTitle);
    return false;
}

static void runDashcamVideoTests(ColorSegmenter &signSegmenter,
                                 ShapeAnalyzer &signAnalyzer,
                                 rsd::HsvMaskSegmenter &manishSegmenter,
                                 rsd::RoadObjectDetector &manishDetector,
                                 const fs::path &root,
                                 const int videoWidth,
                                 const PipelineMode mode)
{
    const char *videoNames[] = {"dashcam.mp4"};
    bool foundAny = false;

    std::cout << "--- STARTING VIDEO DEMO ---" << std::endl;
    std::cout << "Mode: " << pipelineModeLabel(mode) << std::endl;
    std::cout << "Press ESC on a video window to skip remaining clips." << std::endl;

    for (const char *name : videoNames)
    {
        const fs::path videoPath = root / name;
        if (!fs::exists(videoPath))
        {
            std::cerr << "Skipping missing video: " << videoPath.string() << std::endl;
            continue;
        }

        foundAny = true;
        std::cout << "Playing: " << name << std::endl;
        if (runDashcamVideo(videoPath, signSegmenter, signAnalyzer, manishSegmenter, manishDetector,
                            videoWidth, mode))
        {
            break;
        }
    }

    if (!foundAny)
    {
        std::cerr << "No dashcam videos found under " << root.string()
                  << "/ (expected dashcam.mp4)" << std::endl;
    }

    cv::destroyAllWindows();
}

/*
 * Purpose: Run all folder demos, then play dashcam.mp4.
 *
 * Preconditions:
 *   - argc/argv may contain optional --signs-only, --lights-only, or --both.
 *   - Test data exists under data/road_sign_data/ (or fallback paths).
 *
 * Postconditions:
 *   - Displays annotated images and video windows; returns 1 if no data found.
 */
int main(const int argc, char *argv[])
{
    ColorSegmenter signSegmenter;
    ShapeAnalyzer signAnalyzer;
    rsd::HsvMaskSegmenter manishSegmenter;
    rsd::RoadObjectDetector manishDetector;

    constexpr int STATIC_WIDTH = 400;
    constexpr int VIDEO_WIDTH = 600;

    PipelineMode cliOverride = PipelineMode::Both;
    const bool hasCliOverride = parseCliModeOverride(argc, argv, cliOverride);

    const PipelineMode staticMode = hasCliOverride ? cliOverride : kDefaultStaticMode;
    const PipelineMode trafficLightMode =
        hasCliOverride ? cliOverride : PipelineMode::TrafficLightsOnly;
    const PipelineMode videoMode = hasCliOverride ? cliOverride : kDefaultVideoMode;

    const fs::path root = dataRoot();
    std::cout << "Using data folder: " << fs::absolute(root).string() << std::endl;
    if (hasCliOverride)
    {
        std::cout << "CLI override active — all phases use mode: "
                  << pipelineModeLabel(cliOverride) << std::endl;
    }

    if (!hasSignData(root))
    {
        std::cerr << "No sign images or videos found. Put test data under data/road_sign_data/, "
                     "data/, or build/data/."
                  << std::endl;
        return 1;
    }

    runConstructionFolderTests(signSegmenter, signAnalyzer, manishSegmenter, manishDetector, root,
                               STATIC_WIDTH, staticMode);
    runGuideFolderTests(signSegmenter, signAnalyzer, manishSegmenter, manishDetector, root,
                        STATIC_WIDTH, staticMode);
    runServiceFolderTests(signSegmenter, signAnalyzer, manishSegmenter, manishDetector, root,
                          STATIC_WIDTH, staticMode);
    runWarningFolderTests(signSegmenter, signAnalyzer, manishSegmenter, manishDetector, root,
                          STATIC_WIDTH, staticMode);
    runRegulatoryFolderTests(signSegmenter, signAnalyzer, manishSegmenter, manishDetector, root,
                             STATIC_WIDTH, staticMode);
    runTrafficLightFolderTests(signSegmenter, signAnalyzer, manishSegmenter, manishDetector, root,
                               STATIC_WIDTH, trafficLightMode);
    runDashcamVideoTests(signSegmenter, signAnalyzer, manishSegmenter, manishDetector, root,
                         VIDEO_WIDTH, videoMode);

    return 0;
}
