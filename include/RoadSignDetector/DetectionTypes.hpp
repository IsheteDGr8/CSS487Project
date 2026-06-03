/*
 * File: DetectionTypes.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares the small value types used by the detection layer. Keeping
 *   these types separate makes the algorithm easy to test with generated masks
 *   or image files without depending on a GUI.
 */

#ifndef ROAD_SIGN_DETECTOR_DETECTION_TYPES_HPP
#define ROAD_SIGN_DETECTOR_DETECTION_TYPES_HPP

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace rsd
{

/*
 * Color family associated with a binary mask produced by HSV segmentation.
 * The HSV segmentation stage generates these masks; the detector only needs to
 * know which color a mask represents so it can apply sign-specific rules.
 */
enum class MaskColor
{
    Red,
    Yellow,
    Green,
    Blue,
    Unknown
};

enum class DetectionType
{
    StopSign,
    SpeedLimitSign,
    NoUTurnSign,
    NoLeftTurnSign,
    NoRightTurnSign,
    KeepLeftSign,
    KeepRightSign,
    RailwayCrossingSign,
    FallingRocksSign,
    RoadNarrowsSign,
    PedestrianCrossingSign,
    BicycleCrossingSign,
    FerrySign,
    AnimalCrossingSign,
    FirstAidSign,
    NoHornSign,
    NoEntrySign,
    SafetyFirstSign,
    CircularSign,
    RedTrafficLight,
    YellowTrafficLight,
    GreenTrafficLight,
    Unknown
};

/*
 * A binary mask and its semantic color label.
 *
 * Preconditions:
 *   - mask should be a single-channel 8-bit image.
 *   - non-zero pixels represent candidate foreground regions.
 *
 * Postconditions:
 *   - The detector treats mask as read-only and never stores references to it.
 */
struct MaskInput
{
    cv::Mat mask;
    MaskColor color = MaskColor::Unknown;
    std::string debugName;
};

/*
 * Geometric measurements from one contour.
 */
struct ShapeFeatures
{
    double area = 0.0;
    double perimeter = 0.0;
    double circularity = 0.0;
    double aspectRatio = 0.0;
    int vertexCount = 0;
    cv::Rect boundingBox;
    cv::Point2f center;
    float enclosingRadius = 0.0F;
    bool hasHoughCircle = false;
    cv::Point2f houghCenter;
    float houghRadius = 0.0F;
};


struct Detection
{
    DetectionType type = DetectionType::Unknown;
    MaskColor color = MaskColor::Unknown;
    double confidence = 0.0;
    std::string text;
    ShapeFeatures features;
    std::vector<cv::Point> contour;
};

inline std::string toString(const MaskColor color)
{
    switch (color)
    {
    case MaskColor::Red:
        return "red";
    case MaskColor::Yellow:
        return "yellow";
    case MaskColor::Green:
        return "green";
    case MaskColor::Blue:
        return "blue";
    case MaskColor::Unknown:
    default:
        return "unknown";
    }
}

inline std::string toString(const DetectionType type)
{
    switch (type)
    {
    case DetectionType::StopSign:
        return "STOP_SIGN";
    case DetectionType::SpeedLimitSign:
        return "SPEED_LIMIT_SIGN";
    case DetectionType::NoUTurnSign:
        return "NO_U_TURN_SIGN";
    case DetectionType::NoLeftTurnSign:
        return "NO_LEFT_TURN_SIGN";
    case DetectionType::NoRightTurnSign:
        return "NO_RIGHT_TURN_SIGN";
    case DetectionType::KeepLeftSign:
        return "KEEP_LEFT_SIGN";
    case DetectionType::KeepRightSign:
        return "KEEP_RIGHT_SIGN";
    case DetectionType::RailwayCrossingSign:
        return "RAILWAY_CROSSING_SIGN";
    case DetectionType::FallingRocksSign:
        return "FALLING_ROCKS_SIGN";
    case DetectionType::RoadNarrowsSign:
        return "ROAD_NARROWS_SIGN";
    case DetectionType::PedestrianCrossingSign:
        return "PEDESTRIAN_CROSSING_SIGN";
    case DetectionType::BicycleCrossingSign:
        return "BICYCLE_CROSSING_SIGN";
    case DetectionType::FerrySign:
        return "FERRY_SIGN";
    case DetectionType::AnimalCrossingSign:
        return "ANIMAL_CROSSING_SIGN";
    case DetectionType::FirstAidSign:
        return "FIRST_AID_SIGN";
    case DetectionType::NoHornSign:
        return "NO_HORN_SIGN";
    case DetectionType::NoEntrySign:
        return "NO_ENTRY_SIGN";
    case DetectionType::SafetyFirstSign:
        return "SAFETY_FIRST_SIGN";
    case DetectionType::CircularSign:
        return "CIRCULAR_SIGN";
    case DetectionType::RedTrafficLight:
        return "RED_TRAFFIC_LIGHT";
    case DetectionType::YellowTrafficLight:
        return "YELLOW_TRAFFIC_LIGHT";
    case DetectionType::GreenTrafficLight:
        return "GREEN_TRAFFIC_LIGHT";
    case DetectionType::Unknown:
    default:
        return "UNKNOWN";
    }
}

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_DETECTION_TYPES_HPP
