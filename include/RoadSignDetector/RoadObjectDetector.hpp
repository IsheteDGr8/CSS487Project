/*
 * File: RoadObjectDetector.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares the detector. This class receives color masks, extracts contours,
 *   filters noise, measures shape features, applies Hough circles, and returns
 *   rule-based road sign or traffic light labels.
 */

#ifndef ROAD_SIGN_DETECTOR_ROAD_OBJECT_DETECTOR_HPP
#define ROAD_SIGN_DETECTOR_ROAD_OBJECT_DETECTOR_HPP

#include "RoadSignDetector/DetectionTypes.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace rsd
{

/*
 *
 * The defaults are to remove
 * tiny specks, tolerate imperfect contours, and only accept Hough circles that
 * are large enough to represent a visible traffic light bulb.
 */
struct DetectorConfig
{
    double minContourArea = 200.0;
    double maxContourArea = 250000.0;
    double polygonApproximationRatio = 0.035;
    double minimumCircularity = 0.72;
    double squareAspectTolerance = 0.35;
    double houghDp = 1.2;
    double houghMinDistance = 18.0;
    double houghParam1 = 120.0;
    double houghParam2 = 18.0;
    int houghMinRadius = 5;
    int houghMaxRadius = 80;
};

/*
 * Algorithmic road object detector.
 *
 *:
 *   - The detector owns only its configuration.
 *   - Input images and masks are borrowed during function calls.
 *   - Returned detections contain copied contours and value type features.
 */
class RoadObjectDetector
{
public:
    explicit RoadObjectDetector(DetectorConfig config = DetectorConfig());

    /*
     * Runs the full detection pipeline.
     *
     * Preconditions:
     *   - bgrImage is either empty or a valid 8 bit BGR image. A non empty image
     *     enables Hough circle analysis inside contour regions.
     *   - masks contains one binary mask per HSV color class.
     *
     * Postconditions:
     *   - Returns zero or more detections sorted by descending confidence.
     *   - Does not modify bgrImage or any mask stored in masks.
     */
    [[nodiscard]] std::vector<Detection> detect(
        const cv::Mat& bgrImage,
        const std::vector<MaskInput>& masks) const;

    /*
     * Extracts external contours from a binary mask.
     *
     * Preconditions:
     *   - mask must represent foreground with non-zero pixels.
     *
     * Postconditions:
     *   - Returns only contours whose area falls inside the configured area
     *     range.
     */
    [[nodiscard]] std::vector<std::vector<cv::Point>> extractContours(
        const cv::Mat& mask) const;

private:
    [[nodiscard]] ShapeFeatures analyzeContour(
        const cv::Mat& bgrImage,
        const std::vector<cv::Point>& contour) const;

    [[nodiscard]] Detection labelDetection(
        MaskColor color,
        const ShapeFeatures& features,
        const std::vector<cv::Point>& contour) const;

    [[nodiscard]] bool tryFindHoughCircle(
        const cv::Mat& bgrImage,
        const cv::Rect& boundingBox,
        ShapeFeatures& features) const;

    [[nodiscard]] static bool isTriangle(const ShapeFeatures& features);
    [[nodiscard]] static bool isOctagon(const ShapeFeatures& features);
    [[nodiscard]] bool isSquareLike(const ShapeFeatures& features) const;
    [[nodiscard]] bool isCircleLike(const ShapeFeatures& features) const;
    [[nodiscard]] static double clampConfidence(double value);

    DetectorConfig config_;
};

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_ROAD_OBJECT_DETECTOR_HPP
