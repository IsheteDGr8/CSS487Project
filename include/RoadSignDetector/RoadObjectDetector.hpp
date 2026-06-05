/*
 * File: RoadObjectDetector.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Authors: Manish Ram, Ishaan
 *
 * Purpose:
 *   Declares the traffic-light detector facade. Combines TrafficLightFinder
 *   (HSV bulb search) with mask-contour Hough analysis. Road signs are handled
 *   separately by ShapeAnalyzer in main.cpp.
 *
 * Assumptions:
 *   - Input masks come from HsvMaskSegmenter (red, yellow, green).
 *   - Returned detections contain traffic-light types only from the mask loop.
 */

#ifndef ROAD_SIGN_DETECTOR_ROAD_OBJECT_DETECTOR_HPP
#define ROAD_SIGN_DETECTOR_ROAD_OBJECT_DETECTOR_HPP

#include "RoadSignDetector/DetectionTypes.hpp"
#include "RoadSignDetector/TrafficLightFinder.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace rsd
{

/*
 * Tunable thresholds for contour filtering, circularity, and HoughCircles.
 * Defaults remove tiny specks and accept circles large enough to be bulbs.
 */
struct DetectorConfig
{
    double minContourArea = 200.0;
    double maxContourArea = 250000.0;
    double polygonApproximationRatio = 0.035;
    double minimumCircularity = 0.72;
    double houghDp = 1.2;
    double houghMinDistance = 18.0;
    double houghParam1 = 120.0;
    double houghParam2 = 18.0;
    int houghMinRadius = 5;
    int houghMaxRadius = 80;
};

/*
 * Traffic-light detector using dual-path fusion:
 *   1. TrafficLightFinder on the full BGR frame.
 *   2. HSV mask contours + Hough circle labeling.
 */
class RoadObjectDetector
{
public:
    explicit RoadObjectDetector(DetectorConfig config = DetectorConfig());

    /*
     * Runs traffic-light detection on one frame.
     *
     * Preconditions:
     *   - bgrImage is a valid 8-bit BGR image (non-empty for best results).
     *   - masks holds binary HSV masks from HsvMaskSegmenter.
     *
     * Postconditions:
     *   - Returns detections sorted by descending confidence.
     *   - Does not modify bgrImage or masks.
     */
    [[nodiscard]] std::vector<Detection> detect(
        const cv::Mat &bgrImage,
        const std::vector<MaskInput> &masks) const;

    /*
     * Extracts external contours from a binary mask within the area limits.
     *
     * Preconditions:
     *   - mask uses non-zero pixels for foreground.
     *
     * Postconditions:
     *   - Returns contours whose area is within [minContourArea, maxContourArea].
     */
    [[nodiscard]] std::vector<std::vector<cv::Point>> extractContours(
        const cv::Mat &mask) const;

private:
    [[nodiscard]] ShapeFeatures analyzeContour(
        const cv::Mat &bgrImage,
        const std::vector<cv::Point> &contour) const;

    [[nodiscard]] Detection labelTrafficLight(
        MaskColor color,
        const ShapeFeatures &features,
        const std::vector<cv::Point> &contour) const;

    [[nodiscard]] bool tryFindHoughCircle(
        const cv::Mat &bgrImage,
        const cv::Rect &boundingBox,
        ShapeFeatures &features) const;

    [[nodiscard]] bool isCircleLike(const ShapeFeatures &features) const;
    [[nodiscard]] static double clampConfidence(double value);

    DetectorConfig config_;
    TrafficLightFinder trafficLightFinder_;
};

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_ROAD_OBJECT_DETECTOR_HPP
