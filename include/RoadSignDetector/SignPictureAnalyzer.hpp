/*
 * File: SignPictureAnalyzer.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares icon and color clues used by the rule based sign detector.
 */

#ifndef ROAD_SIGN_DETECTOR_SIGN_PICTURE_ANALYZER_HPP
#define ROAD_SIGN_DETECTOR_SIGN_PICTURE_ANALYZER_HPP

#include <opencv2/core.hpp>

namespace rsd
{

struct PictureClues
{
    double darkAreaRatio = 0.0;
    double redAreaRatio = 0.0;
    double yellowAreaRatio = 0.0;
    double blueAreaRatio = 0.0;
    int darkPartCount = 0;
    bool hasRedSlash = false;
    bool hasWhiteCross = false;
    bool hasWhiteHorizontalBar = false;
    bool hasTwoWheelShapes = false;
    bool hasDarkCurvedArrow = false;
    bool hasDarkLeftArrow = false;
    bool hasDarkRightArrow = false;
    bool hasWhiteLeftArrow = false;
    bool hasWhiteRightArrow = false;
    bool hasDarkTrainShape = false;
    bool hasTallDarkShape = false;
    bool hasManySmallDarkParts = false;
    bool hasBottomWaveShape = false;
    bool hasWideAnimalShape = false;
};

class SignPictureAnalyzer
{
public:
    [[nodiscard]] PictureClues inspect(const cv::Mat& bgrImage, const cv::Rect& signBox) const;

private:
    [[nodiscard]] static cv::Rect keepInsideImage(const cv::Rect& box, const cv::Size& imageSize);
    [[nodiscard]] static double maskRatio(const cv::Mat& mask);
    [[nodiscard]] static bool hasRedDiagonal(const cv::Mat& redMask);
    [[nodiscard]] static bool hasWhiteCrossShape(const cv::Mat& whiteMask);
    [[nodiscard]] static bool hasWhiteCenterBar(const cv::Mat& whiteMask);
    [[nodiscard]] static bool hasTwoDarkCircles(const cv::Mat& darkMask);
    [[nodiscard]] static bool hasBottomWave(const cv::Mat& darkMask);
    [[nodiscard]] static bool hasCurvedArrowShape(const cv::Mat& darkMask);
    [[nodiscard]] static int arrowDirection(const cv::Mat& mask);
    [[nodiscard]] static bool hasTrainShape(const cv::Mat& darkMask);
};

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_SIGN_PICTURE_ANALYZER_HPP
