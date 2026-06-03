/*
 * File: HsvMaskSegmenter.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Implements baseline HSV color segmentation and morphological cleanup for
 *   standalone testing of the detector.
 */

#include "RoadSignDetector/HsvMaskSegmenter.hpp"

#include <opencv2/imgproc.hpp>

namespace rsd
{

std::vector<MaskInput> HsvMaskSegmenter::createMasks(const cv::Mat& bgrImage) const
{
    cv::Mat hsvImage;
    cv::cvtColor(bgrImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat redLow;
    cv::Mat redHigh;
    cv::Mat yellow;
    cv::Mat green;
    cv::Mat blue;

    cv::inRange(hsvImage, cv::Scalar(0, 80, 70), cv::Scalar(10, 255, 255), redLow);
    cv::inRange(hsvImage, cv::Scalar(170, 80, 70), cv::Scalar(180, 255, 255), redHigh);
    cv::inRange(hsvImage, cv::Scalar(18, 80, 80), cv::Scalar(35, 255, 255), yellow);
    cv::inRange(hsvImage, cv::Scalar(36, 60, 60), cv::Scalar(90, 255, 255), green);
    cv::inRange(hsvImage, cv::Scalar(95, 70, 60), cv::Scalar(130, 255, 255), blue);

    cv::Mat red;
    cv::bitwise_or(redLow, redHigh, red);

    return {
        {cleanMask(red), MaskColor::Red, "red"},
        {cleanMask(yellow), MaskColor::Yellow, "yellow"},
        {cleanMask(green), MaskColor::Green, "green"},
        {cleanMask(blue), MaskColor::Blue, "blue"}};
}

cv::Mat HsvMaskSegmenter::cleanMask(const cv::Mat& rawMask)
{
    cv::Mat cleaned;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(rawMask, cleaned, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE, kernel);
    return cleaned;
}

} // namespace rsd
