/*
 * File: DebugImageWriter.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Implements debug image output for detections and HSV masks.
 */

#include "RoadSignDetector/DebugImageWriter.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace rsd
{

void DebugImageWriter::save(
    const std::filesystem::path& outputDirectory,
    const cv::Mat& bgrImage,
    const std::vector<Detection>& detections,
    const std::vector<MaskInput>& masks) const
{
    std::filesystem::create_directories(outputDirectory);

    const cv::Mat annotatedImage = createAnnotatedImage(bgrImage, detections);
    cv::imwrite((outputDirectory / "annotated.png").string(), annotatedImage);

    for (const MaskInput& mask : masks)
    {
        const std::string fileName = mask.debugName.empty()
            ? toString(mask.color) + "_mask.png"
            : mask.debugName + "_mask.png";
        cv::imwrite((outputDirectory / fileName).string(), mask.mask);
    }
}

cv::Mat DebugImageWriter::createAnnotatedImage(
    const cv::Mat& bgrImage,
    const std::vector<Detection>& detections) const
{
    cv::Mat annotatedImage = bgrImage.clone();

    for (const Detection& detection : detections)
    {
        drawDetection(annotatedImage, detection);
    }

    return annotatedImage;
}

void DebugImageWriter::drawDetection(cv::Mat& image, const Detection& detection)
{
    const cv::Rect box = detection.features.boundingBox;
    cv::rectangle(image, box, cv::Scalar(0, 255, 255), 2);
    cv::drawContours(image, std::vector<std::vector<cv::Point>>{detection.contour}, -1,
                     cv::Scalar(255, 0, 255), 2);

    if (detection.features.hasHoughCircle)
    {
        const cv::Point center(
            static_cast<int>(std::lround(detection.features.houghCenter.x)),
            static_cast<int>(std::lround(detection.features.houghCenter.y)));
        cv::circle(image, center, static_cast<int>(detection.features.houghRadius),
                   cv::Scalar(0, 255, 0), 2);
    }

    const std::string label =
        toString(detection.type) + " " + toString(detection.color);
    const cv::Point textPoint(box.x, std::max(0, box.y - 8));
    cv::putText(image, label, textPoint, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 2);
}

} // namespace rsd
