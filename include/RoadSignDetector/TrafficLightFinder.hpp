/*
 * File: TrafficLightFinder.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares a traffic light bulb detector based on bright HSV blobs, circular
 *   shape measurements, Hough circle evidence, and nearby dark housing.
 */

#ifndef ROAD_SIGN_DETECTOR_TRAFFIC_LIGHT_FINDER_HPP
#define ROAD_SIGN_DETECTOR_TRAFFIC_LIGHT_FINDER_HPP

#include "RoadSignDetector/DetectionTypes.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace rsd
{

class TrafficLightFinder
{
public:
    [[nodiscard]] std::vector<Detection> find(const cv::Mat& bgrImage) const;

private:
    struct LightMasks
    {
        cv::Mat red;
        cv::Mat yellow;
        cv::Mat green;
        cv::Mat combined;
    };

    [[nodiscard]] static LightMasks makeLightMasks(const cv::Mat& hsvImage);
    [[nodiscard]] static ShapeFeatures measureContour(const std::vector<cv::Point>& contour);
    [[nodiscard]] static DetectionType chooseLightType(
        const LightMasks& masks,
        const std::vector<cv::Point>& contour,
        const cv::Rect& box,
        MaskColor& color);
    [[nodiscard]] static bool hasHoughCircle(
        const cv::Mat& bgrImage,
        const cv::Rect& box,
        ShapeFeatures& features);
    [[nodiscard]] static bool looksLikeTrafficBulb(
        const cv::Mat& hsvImage,
        const std::vector<cv::Point>& contour,
        const ShapeFeatures& features);
    [[nodiscard]] static bool hasBrightColorCore(
        const cv::Mat& hsvImage,
        const std::vector<cv::Point>& contour,
        const cv::Rect& box);
    [[nodiscard]] static bool hasDarkHousingNearBulb(const cv::Mat& hsvImage, const cv::Rect& box);
    [[nodiscard]] static cv::Rect expandBox(const cv::Rect& box, const cv::Size& imageSize);
    [[nodiscard]] static cv::Rect keepInsideImage(const cv::Rect& box, const cv::Size& imageSize);
    [[nodiscard]] static double clampConfidence(double value);
};

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_TRAFFIC_LIGHT_FINDER_HPP
