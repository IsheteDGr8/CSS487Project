/*
 * File: RoadObjectDetector.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Implements the classical computer vision pipeline: contour extraction,
 *   area filtering, polygon approximation, Hough circle analysis, and final
 *   rule-based labeling.
 */

#include "RoadSignDetector/RoadObjectDetector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace rsd
{
namespace
{
cv::Mat normalizeMask(const cv::Mat& mask)
{
    cv::Mat grayMask;
    if (mask.channels() == 1)
    {
        grayMask = mask.clone();
    }
    else
    {
        cv::cvtColor(mask, grayMask, cv::COLOR_BGR2GRAY);
    }

    cv::Mat binaryMask;
    cv::threshold(grayMask, binaryMask, 0, 255, cv::THRESH_BINARY);
    return binaryMask;
}

cv::Rect intersectWithImageBounds(const cv::Rect& rect, const cv::Size& imageSize)
{
    const cv::Rect imageBounds(0, 0, imageSize.width, imageSize.height);
    return rect & imageBounds;
}
} // namespace

RoadObjectDetector::RoadObjectDetector(DetectorConfig config)
    : config_(config)
{
}

std::vector<Detection> RoadObjectDetector::detect(
    const cv::Mat& bgrImage,
    const std::vector<MaskInput>& masks) const
{
    std::vector<Detection> detections;

    for (const MaskInput& input : masks)
    {
        const std::vector<std::vector<cv::Point>> contours = extractContours(input.mask);

        for (const std::vector<cv::Point>& contour : contours)
        {
            const ShapeFeatures features = analyzeContour(bgrImage, contour);
            Detection detection = labelDetection(input.color, features, contour);

            if (detection.type != DetectionType::Unknown)
            {
                detections.push_back(detection);
            }
        }
    }

    std::sort(
        detections.begin(),
        detections.end(),
        [](const Detection& left, const Detection& right)
        {
            return left.confidence > right.confidence;
        });

    return detections;
}

std::vector<std::vector<cv::Point>> RoadObjectDetector::extractContours(
    const cv::Mat& mask) const
{
    std::vector<std::vector<cv::Point>> filteredContours;

    if (mask.empty())
    {
        return filteredContours;
    }

    cv::Mat binaryMask = normalizeMask(mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const std::vector<cv::Point>& contour : contours)
    {
        const double area = cv::contourArea(contour);
        if (area >= config_.minContourArea && area <= config_.maxContourArea)
        {
            filteredContours.push_back(contour);
        }
    }

    return filteredContours;
}

ShapeFeatures RoadObjectDetector::analyzeContour(
    const cv::Mat& bgrImage,
    const std::vector<cv::Point>& contour) const
{
    ShapeFeatures features;
    features.area = cv::contourArea(contour);
    features.perimeter = cv::arcLength(contour, true);
    features.boundingBox = cv::boundingRect(contour);

    if (features.boundingBox.height > 0)
    {
        features.aspectRatio =
            static_cast<double>(features.boundingBox.width) /
            static_cast<double>(features.boundingBox.height);
    }

    if (features.perimeter > 0.0)
    {
        features.circularity =
            (4.0 * CV_PI * features.area) /
            (features.perimeter * features.perimeter);
    }

    std::vector<cv::Point> polygon;
    const double epsilon = config_.polygonApproximationRatio * features.perimeter;
    cv::approxPolyDP(contour, polygon, epsilon, true);
    features.vertexCount = static_cast<int>(polygon.size());

    cv::minEnclosingCircle(contour, features.center, features.enclosingRadius);

    ShapeFeatures houghFeatures = features;
    if (tryFindHoughCircle(bgrImage, features.boundingBox, houghFeatures))
    {
        features = houghFeatures;
    }

    return features;
}

Detection RoadObjectDetector::labelDetection(
    const MaskColor color,
    const ShapeFeatures& features,
    const std::vector<cv::Point>& contour) const
{
    Detection detection;
    detection.color = color;
    detection.features = features;
    detection.contour = contour;

    /*
     * Traffic light bulbs are nearly circular colored regions. HoughCircles is
     * used as extra evidence because traffic lights are designed as bright
     * circles even when their outer housing is rectangular.
     */
    if (features.hasHoughCircle && isCircleLike(features))
    {
        if (color == MaskColor::Red)
        {
            detection.type = DetectionType::RedTrafficLight;
            detection.confidence = clampConfidence(0.78 + features.circularity * 0.2);
            return detection;
        }
        if (color == MaskColor::Yellow)
        {
            detection.type = DetectionType::YellowTrafficLight;
            detection.confidence = clampConfidence(0.76 + features.circularity * 0.2);
            return detection;
        }
        if (color == MaskColor::Green)
        {
            detection.type = DetectionType::GreenTrafficLight;
            detection.confidence = clampConfidence(0.76 + features.circularity * 0.2);
            return detection;
        }
    }

    if (color == MaskColor::Red && isOctagon(features))
    {
        detection.type = DetectionType::StopSign;
        detection.confidence = clampConfidence(0.74 + features.circularity * 0.18);
        return detection;
    }

    if (color == MaskColor::Red && isTriangle(features))
    {
        detection.type = DetectionType::YieldSign;
        detection.confidence = 0.82;
        return detection;
    }

    if (color == MaskColor::Yellow && isSquareLike(features))
    {
        detection.type = DetectionType::WarningSign;
        detection.confidence = 0.80;
        return detection;
    }

    if ((color == MaskColor::Red || color == MaskColor::Blue) && isCircleLike(features))
    {
        detection.type = DetectionType::CircularSign;
        detection.confidence = clampConfidence(0.68 + features.circularity * 0.2);
        return detection;
    }

    return detection;
}

bool RoadObjectDetector::tryFindHoughCircle(
    const cv::Mat& bgrImage,
    const cv::Rect& boundingBox,
    ShapeFeatures& features) const
{
    if (bgrImage.empty() || boundingBox.empty())
    {
        return false;
    }

    const cv::Rect roiBox = intersectWithImageBounds(boundingBox, bgrImage.size());
    if (roiBox.width <= 0 || roiBox.height <= 0)
    {
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(bgrImage(roiBox), gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2.0, 2.0);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(
        gray,
        circles,
        cv::HOUGH_GRADIENT,
        config_.houghDp,
        config_.houghMinDistance,
        config_.houghParam1,
        config_.houghParam2,
        config_.houghMinRadius,
        config_.houghMaxRadius);

    if (circles.empty())
    {
        return false;
    }

    const cv::Vec3f& circle = circles.front();
    features.hasHoughCircle = true;
    features.houghCenter = cv::Point2f(circle[0] + static_cast<float>(roiBox.x),
                                       circle[1] + static_cast<float>(roiBox.y));
    features.houghRadius = circle[2];
    return true;
}

bool RoadObjectDetector::isTriangle(const ShapeFeatures& features)
{
    return features.vertexCount == 3;
}

bool RoadObjectDetector::isOctagon(const ShapeFeatures& features)
{
    return features.vertexCount >= 7 && features.vertexCount <= 10;
}

bool RoadObjectDetector::isSquareLike(const ShapeFeatures& features) const
{
    const double lowerBound = 1.0 - config_.squareAspectTolerance;
    const double upperBound = 1.0 + config_.squareAspectTolerance;
    return features.vertexCount == 4 &&
           features.aspectRatio >= lowerBound &&
           features.aspectRatio <= upperBound;
}

bool RoadObjectDetector::isCircleLike(const ShapeFeatures& features) const
{
    return features.circularity >= config_.minimumCircularity;
}

double RoadObjectDetector::clampConfidence(const double value)
{
    return std::max(0.0, std::min(1.0, value));
}

} // namespace rsd
