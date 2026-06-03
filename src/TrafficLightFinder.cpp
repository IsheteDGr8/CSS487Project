/*
 * File: TrafficLightFinder.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Implements traffic light bulb detection using classical OpenCV features.
 */

#include "RoadSignDetector/TrafficLightFinder.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace rsd
{
namespace
{
constexpr double kMinimumRoundness = 0.42;
constexpr double kStrongRoundness = 0.68;

double maskRatio(const cv::Mat& mask)
{
    if (mask.empty())
    {
        return 0.0;
    }

    return static_cast<double>(cv::countNonZero(mask)) /
           static_cast<double>(mask.rows * mask.cols);
}
} // namespace

std::vector<Detection> TrafficLightFinder::find(const cv::Mat& bgrImage) const
{
    std::vector<Detection> detections;
    if (bgrImage.empty())
    {
        return detections;
    }

    cv::Mat hsvImage;
    cv::cvtColor(bgrImage, hsvImage, cv::COLOR_BGR2HSV);

    const LightMasks masks = makeLightMasks(hsvImage);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(masks.combined.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const std::vector<cv::Point>& contour : contours)
    {
        ShapeFeatures features = measureContour(contour);
        if (features.boundingBox.empty())
        {
            continue;
        }

        if (hasHoughCircle(bgrImage, features.boundingBox, features) ||
            looksLikeTrafficBulb(hsvImage, contour, features))
        {
            if (!looksLikeTrafficBulb(hsvImage, contour, features))
            {
                continue;
            }

            MaskColor color = MaskColor::Unknown;
            const DetectionType type =
                chooseLightType(masks, contour, features.boundingBox, color);
            if (type == DetectionType::Unknown)
            {
                continue;
            }

            Detection detection;
            detection.type = type;
            detection.color = color;
            detection.features = features;
            detection.contour = contour;
            detection.confidence = clampConfidence(
                0.84 +
                std::min(features.circularity, 1.0) * 0.10 +
                (features.hasHoughCircle ? 0.06 : 0.0));
            detections.push_back(detection);
        }
    }

    return detections;
}

TrafficLightFinder::LightMasks TrafficLightFinder::makeLightMasks(const cv::Mat& hsvImage)
{
    LightMasks masks;

    cv::Mat redLow;
    cv::Mat redHigh;
    cv::inRange(hsvImage, cv::Scalar(0, 55, 80), cv::Scalar(14, 255, 255), redLow);
    cv::inRange(hsvImage, cv::Scalar(166, 55, 80), cv::Scalar(180, 255, 255), redHigh);
    cv::bitwise_or(redLow, redHigh, masks.red);

    cv::inRange(hsvImage, cv::Scalar(12, 45, 95), cv::Scalar(46, 255, 255), masks.yellow);
    cv::inRange(hsvImage, cv::Scalar(36, 35, 70), cv::Scalar(96, 255, 255), masks.green);

    cv::bitwise_or(masks.red, masks.yellow, masks.combined);
    cv::bitwise_or(masks.combined, masks.green, masks.combined);

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(masks.combined, masks.combined, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(masks.combined, masks.combined, cv::MORPH_OPEN, kernel);
    return masks;
}

ShapeFeatures TrafficLightFinder::measureContour(const std::vector<cv::Point>& contour)
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
    cv::approxPolyDP(contour, polygon, 0.035 * features.perimeter, true);
    features.vertexCount = static_cast<int>(polygon.size());

    cv::minEnclosingCircle(contour, features.center, features.enclosingRadius);
    return features;
}

DetectionType TrafficLightFinder::chooseLightType(
    const LightMasks& masks,
    const std::vector<cv::Point>& contour,
    const cv::Rect& box,
    MaskColor& color)
{
    const cv::Rect safeBox = expandBox(box, masks.combined.size());
    if (safeBox.empty())
    {
        color = MaskColor::Unknown;
        return DetectionType::Unknown;
    }

    cv::Mat candidateMask = cv::Mat::zeros(safeBox.height, safeBox.width, CV_8UC1);
    std::vector<cv::Point> shiftedContour;
    shiftedContour.reserve(contour.size());
    for (const cv::Point& point : contour)
    {
        shiftedContour.push_back(cv::Point(point.x - safeBox.x, point.y - safeBox.y));
    }

    cv::fillPoly(candidateMask, std::vector<std::vector<cv::Point>>{shiftedContour}, cv::Scalar(255));
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    cv::dilate(candidateMask, candidateMask, kernel);

    cv::Mat redPart;
    cv::Mat yellowPart;
    cv::Mat greenPart;
    cv::bitwise_and(masks.red(safeBox), candidateMask, redPart);
    cv::bitwise_and(masks.yellow(safeBox), candidateMask, yellowPart);
    cv::bitwise_and(masks.green(safeBox), candidateMask, greenPart);

    const int redCount = cv::countNonZero(redPart);
    const int yellowCount = cv::countNonZero(yellowPart);
    const int greenCount = cv::countNonZero(greenPart);
    const int minimumColorPixels = std::max(8, candidateMask.rows * candidateMask.cols / 80);

    if (redCount < minimumColorPixels &&
        yellowCount < minimumColorPixels &&
        greenCount < minimumColorPixels)
    {
        color = MaskColor::Unknown;
        return DetectionType::Unknown;
    }

    if (redCount >= yellowCount && redCount >= greenCount)
    {
        color = MaskColor::Red;
        return DetectionType::RedTrafficLight;
    }

    if (yellowCount >= redCount && yellowCount >= greenCount)
    {
        color = MaskColor::Yellow;
        return DetectionType::YellowTrafficLight;
    }

    color = MaskColor::Green;
    return DetectionType::GreenTrafficLight;
}

bool TrafficLightFinder::hasHoughCircle(
    const cv::Mat& bgrImage,
    const cv::Rect& box,
    ShapeFeatures& features)
{
    const cv::Rect safeBox = keepInsideImage(box, bgrImage.size());
    if (safeBox.width < 5 || safeBox.height < 5)
    {
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(bgrImage(safeBox), gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 1.6, 1.6);

    const int maxRadius = std::max(4, std::min(safeBox.width, safeBox.height));
    const int minRadius = std::max(2, maxRadius / 8);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(
        gray,
        circles,
        cv::HOUGH_GRADIENT,
        1.2,
        std::max(6, std::min(safeBox.width, safeBox.height) / 2),
        100.0,
        11.0,
        minRadius,
        maxRadius);

    if (circles.empty())
    {
        return false;
    }

    const cv::Vec3f& circle = circles.front();
    features.hasHoughCircle = true;
    features.houghCenter = cv::Point2f(circle[0] + static_cast<float>(safeBox.x),
                                       circle[1] + static_cast<float>(safeBox.y));
    features.houghRadius = circle[2];
    return true;
}

bool TrafficLightFinder::looksLikeTrafficBulb(
    const cv::Mat& hsvImage,
    const std::vector<cv::Point>& contour,
    const ShapeFeatures& features)
{
    const double imageArea = static_cast<double>(hsvImage.rows * hsvImage.cols);
    const double minimumArea = std::max(12.0, imageArea * 0.000025);
    const double maximumArea = imageArea * 0.045;

    if (features.area < minimumArea || features.area > maximumArea)
    {
        return false;
    }

    const int imageMinSide = std::min(hsvImage.rows, hsvImage.cols);
    const int largestBoxSide = std::max(features.boundingBox.width, features.boundingBox.height);
    if (largestBoxSide > imageMinSide * 0.28)
    {
        return false;
    }

    if (features.aspectRatio < 0.35 || features.aspectRatio > 2.4)
    {
        return false;
    }

    const bool roundEnough = features.circularity >= kMinimumRoundness || features.hasHoughCircle;
    const bool strongRoundness = features.circularity >= kStrongRoundness;
    if (!roundEnough)
    {
        return false;
    }

    const bool hasBrightCore = hasBrightColorCore(hsvImage, contour, features.boundingBox);
    if (!hasBrightCore)
    {
        return false;
    }

    const bool hasHousing = hasDarkHousingNearBulb(hsvImage, features.boundingBox);
    return hasHousing || features.hasHoughCircle || strongRoundness;
}

bool TrafficLightFinder::hasBrightColorCore(
    const cv::Mat& hsvImage,
    const std::vector<cv::Point>& contour,
    const cv::Rect& box)
{
    const cv::Rect safeBox = keepInsideImage(box, hsvImage.size());
    if (safeBox.empty())
    {
        return false;
    }

    cv::Mat contourMask = cv::Mat::zeros(safeBox.height, safeBox.width, CV_8UC1);
    std::vector<cv::Point> shiftedContour;
    shiftedContour.reserve(contour.size());
    for (const cv::Point& point : contour)
    {
        shiftedContour.push_back(cv::Point(point.x - safeBox.x, point.y - safeBox.y));
    }

    cv::fillPoly(contourMask, std::vector<std::vector<cv::Point>>{shiftedContour}, cv::Scalar(255));

    cv::Mat channels[3];
    cv::split(hsvImage(safeBox), channels);

    cv::Mat brightMask;
    cv::threshold(channels[2], brightMask, 115, 255, cv::THRESH_BINARY);
    cv::bitwise_and(brightMask, contourMask, brightMask);

    const double brightRatio = maskRatio(brightMask);
    const cv::Scalar meanColor = cv::mean(hsvImage(safeBox), contourMask);
    const double meanSaturation = meanColor[1];
    const double meanValue = meanColor[2];

    return brightRatio > 0.28 || (meanValue > 115.0 && meanSaturation > 35.0);
}

bool TrafficLightFinder::hasDarkHousingNearBulb(const cv::Mat& hsvImage, const cv::Rect& box)
{
    const cv::Rect expandedBox = expandBox(box, hsvImage.size());
    if (expandedBox.empty())
    {
        return false;
    }

    cv::Mat darkMask;
    cv::inRange(hsvImage(expandedBox), cv::Scalar(0, 0, 0), cv::Scalar(180, 165, 95), darkMask);

    const double darkRatio = maskRatio(darkMask);
    return darkRatio > 0.08;
}

cv::Rect TrafficLightFinder::expandBox(const cv::Rect& box, const cv::Size& imageSize)
{
    const int padX = std::max(4, box.width);
    const int padY = std::max(4, box.height);
    const cv::Rect expanded(
        box.x - padX,
        box.y - padY,
        box.width + padX * 2,
        box.height + padY * 2);
    return keepInsideImage(expanded, imageSize);
}

cv::Rect TrafficLightFinder::keepInsideImage(const cv::Rect& box, const cv::Size& imageSize)
{
    const cv::Rect imageBounds(0, 0, imageSize.width, imageSize.height);
    return box & imageBounds;
}

double TrafficLightFinder::clampConfidence(const double value)
{
    return std::max(0.0, std::min(1.0, value));
}

} // namespace rsd
