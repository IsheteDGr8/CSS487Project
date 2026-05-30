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
    YieldSign,
    WarningSign,
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
    case DetectionType::YieldSign:
        return "YIELD_SIGN";
    case DetectionType::WarningSign:
        return "WARNING_SIGN";
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
