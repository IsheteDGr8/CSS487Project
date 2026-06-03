/*
 * File: SignPictureAnalyzer.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Implements simple icon and color measurements for road sign labels.
 */

#include "RoadSignDetector/SignPictureAnalyzer.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace rsd
{

PictureClues SignPictureAnalyzer::inspect(const cv::Mat& bgrImage, const cv::Rect& signBox) const
{
    PictureClues clues;
    if (bgrImage.empty() || signBox.empty())
    {
        return clues;
    }

    const cv::Rect safeBox = keepInsideImage(signBox, bgrImage.size());
    if (safeBox.empty())
    {
        return clues;
    }

    const cv::Mat roi = bgrImage(safeBox);
    cv::Mat hsvImage;
    cv::cvtColor(roi, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat redLow;
    cv::Mat redHigh;
    cv::Mat redMask;
    cv::Mat yellowMask;
    cv::Mat blueMask;
    cv::Mat whiteMask;
    cv::Mat darkMask;

    cv::inRange(hsvImage, cv::Scalar(0, 70, 60), cv::Scalar(12, 255, 255), redLow);
    cv::inRange(hsvImage, cv::Scalar(168, 70, 60), cv::Scalar(180, 255, 255), redHigh);
    cv::bitwise_or(redLow, redHigh, redMask);
    cv::inRange(hsvImage, cv::Scalar(14, 60, 70), cv::Scalar(42, 255, 255), yellowMask);
    cv::inRange(hsvImage, cv::Scalar(90, 50, 50), cv::Scalar(135, 255, 255), blueMask);
    cv::inRange(hsvImage, cv::Scalar(0, 0, 165), cv::Scalar(180, 75, 255), whiteMask);
    cv::inRange(hsvImage, cv::Scalar(0, 0, 0), cv::Scalar(180, 140, 135), darkMask);

    clues.redAreaRatio = maskRatio(redMask);
    clues.yellowAreaRatio = maskRatio(yellowMask);
    clues.blueAreaRatio = maskRatio(blueMask);
    clues.darkAreaRatio = maskRatio(darkMask);
    clues.hasRedSlash = hasRedDiagonal(redMask);
    clues.hasWhiteCross = hasWhiteCrossShape(whiteMask);
    clues.hasWhiteHorizontalBar = hasWhiteCenterBar(whiteMask);
    clues.hasTwoWheelShapes = hasTwoDarkCircles(darkMask);
    clues.hasBottomWaveShape = hasBottomWave(darkMask);
    clues.hasDarkCurvedArrow = hasCurvedArrowShape(darkMask);
    clues.hasDarkTrainShape = hasTrainShape(darkMask);

    const int darkArrowDirection = arrowDirection(darkMask);
    clues.hasDarkLeftArrow = darkArrowDirection < 0;
    clues.hasDarkRightArrow = darkArrowDirection > 0;

    const int whiteArrowDirection = arrowDirection(whiteMask);
    clues.hasWhiteLeftArrow = whiteArrowDirection < 0;
    clues.hasWhiteRightArrow = whiteArrowDirection > 0;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(darkMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Rect combinedDarkBox;
    int largePartCount = 0;
    int smallPartCount = 0;

    for (const std::vector<cv::Point>& contour : contours)
    {
        const cv::Rect box = cv::boundingRect(contour);
        const int area = box.width * box.height;
        if (area < 25)
        {
            continue;
        }

        combinedDarkBox = combinedDarkBox.empty() ? box : (combinedDarkBox | box);
        if (area > 150)
        {
            ++largePartCount;
        }
        else
        {
            ++smallPartCount;
        }
    }

    clues.darkPartCount = largePartCount + smallPartCount;
    clues.hasManySmallDarkParts = smallPartCount >= 4 || clues.darkPartCount >= 7;

    if (!combinedDarkBox.empty())
    {
        const double shapeRatio =
            static_cast<double>(combinedDarkBox.height) /
            std::max(1, combinedDarkBox.width);
        const double wideRatio =
            static_cast<double>(combinedDarkBox.width) /
            std::max(1, combinedDarkBox.height);
        clues.hasTallDarkShape = shapeRatio > 1.8 && combinedDarkBox.height > safeBox.height / 3;
        clues.hasWideAnimalShape = wideRatio > 1.6 && clues.darkAreaRatio > 0.08;
    }

    return clues;
}

cv::Rect SignPictureAnalyzer::keepInsideImage(const cv::Rect& box, const cv::Size& imageSize)
{
    const cv::Rect imageBounds(0, 0, imageSize.width, imageSize.height);
    return box & imageBounds;
}

double SignPictureAnalyzer::maskRatio(const cv::Mat& mask)
{
    if (mask.empty())
    {
        return 0.0;
    }

    return static_cast<double>(cv::countNonZero(mask)) /
           static_cast<double>(mask.rows * mask.cols);
}

bool SignPictureAnalyzer::hasRedDiagonal(const cv::Mat& redMask)
{
    if (redMask.empty())
    {
        return false;
    }

    int diagonalCount = 0;
    const int band = std::max(3, redMask.cols / 14);

    for (int y = 0; y < redMask.rows; ++y)
    {
        const uchar* row = redMask.ptr<uchar>(y);
        for (int x = 0; x < redMask.cols; ++x)
        {
            if (row[x] == 0)
            {
                continue;
            }

            const int downSlashY = (x * redMask.rows) / std::max(1, redMask.cols);
            const int upSlashY = redMask.rows - 1 - downSlashY;
            if (std::abs(y - downSlashY) <= band || std::abs(y - upSlashY) <= band)
            {
                ++diagonalCount;
            }
        }
    }

    return diagonalCount > redMask.rows * redMask.cols / 45;
}

bool SignPictureAnalyzer::hasWhiteCrossShape(const cv::Mat& whiteMask)
{
    if (whiteMask.empty())
    {
        return false;
    }

    const cv::Rect centerBox(
        whiteMask.cols / 4,
        whiteMask.rows / 4,
        whiteMask.cols / 2,
        whiteMask.rows / 2);
    const cv::Mat center = whiteMask(centerBox);

    cv::Mat verticalBand = center.colRange(center.cols / 3, 2 * center.cols / 3);
    cv::Mat horizontalBand = center.rowRange(center.rows / 3, 2 * center.rows / 3);

    const double verticalRatio = maskRatio(verticalBand);
    const double horizontalRatio = maskRatio(horizontalBand);
    const double centerRatio = maskRatio(center);

    return verticalRatio > 0.38 && horizontalRatio > 0.38 && centerRatio > 0.20;
}

bool SignPictureAnalyzer::hasWhiteCenterBar(const cv::Mat& whiteMask)
{
    if (whiteMask.empty())
    {
        return false;
    }

    const cv::Rect centerBand(
        whiteMask.cols / 5,
        (whiteMask.rows * 2) / 5,
        (whiteMask.cols * 3) / 5,
        whiteMask.rows / 5);

    return maskRatio(whiteMask(centerBand)) > 0.55;
}

bool SignPictureAnalyzer::hasTwoDarkCircles(const cv::Mat& darkMask)
{
    if (darkMask.empty())
    {
        return false;
    }

    cv::Mat blurred;
    cv::GaussianBlur(darkMask, blurred, cv::Size(7, 7), 1.5);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(
        blurred,
        circles,
        cv::HOUGH_GRADIENT,
        1.2,
        std::max(12, darkMask.cols / 5),
        100.0,
        10.0,
        std::max(3, darkMask.cols / 20),
        std::max(8, darkMask.cols / 4));

    return circles.size() >= 2;
}

bool SignPictureAnalyzer::hasBottomWave(const cv::Mat& darkMask)
{
    if (darkMask.empty())
    {
        return false;
    }

    const cv::Rect bottomBand(
        0,
        (darkMask.rows * 2) / 3,
        darkMask.cols,
        darkMask.rows / 3);

    const cv::Rect topBand(0, 0, darkMask.cols, darkMask.rows / 3);
    return maskRatio(darkMask(bottomBand)) > 0.08 && maskRatio(darkMask(topBand)) < 0.12;
}

bool SignPictureAnalyzer::hasCurvedArrowShape(const cv::Mat& darkMask)
{
    if (darkMask.empty())
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(darkMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const std::vector<cv::Point>& contour : contours)
    {
        const cv::Rect box = cv::boundingRect(contour);
        const int area = box.width * box.height;
        if (area < 120)
        {
            continue;
        }

        const cv::Mat part = darkMask(box);
        const cv::Rect topBand(0, 0, part.cols, part.rows / 3);
        const cv::Rect bottomBand(0, (part.rows * 2) / 3, part.cols, part.rows / 3);
        const cv::Rect leftBand(0, 0, part.cols / 3, part.rows);
        const cv::Rect rightBand((part.cols * 2) / 3, 0, part.cols / 3, part.rows);

        const double topRatio = maskRatio(part(topBand));
        const double bottomRatio = maskRatio(part(bottomBand));
        const double leftRatio = maskRatio(part(leftBand));
        const double rightRatio = maskRatio(part(rightBand));
        const double shapeRatio = static_cast<double>(box.height) / std::max(1, box.width);

        if (shapeRatio > 0.75 &&
            shapeRatio < 2.2 &&
            topRatio > 0.08 &&
            bottomRatio > 0.08 &&
            leftRatio > 0.08 &&
            rightRatio > 0.08)
        {
            return true;
        }
    }

    return false;
}

int SignPictureAnalyzer::arrowDirection(const cv::Mat& mask)
{
    if (mask.empty())
    {
        return 0;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Rect largestBox;
    int largestArea = 0;
    for (const std::vector<cv::Point>& contour : contours)
    {
        const cv::Rect box = cv::boundingRect(contour);
        const int area = box.width * box.height;
        if (area > largestArea)
        {
            largestArea = area;
            largestBox = box;
        }
    }

    if (largestBox.empty() || largestArea < 80)
    {
        return 0;
    }

    const cv::Mat part = mask(largestBox);
    const cv::Rect leftThird(0, 0, std::max(1, part.cols / 3), part.rows);
    const cv::Rect rightThird((part.cols * 2) / 3, 0, std::max(1, part.cols / 3), part.rows);

    const double leftRatio = maskRatio(part(leftThird));
    const double rightRatio = maskRatio(part(rightThird));

    if (leftRatio > rightRatio * 1.25)
    {
        return -1;
    }
    if (rightRatio > leftRatio * 1.25)
    {
        return 1;
    }

    return 0;
}

bool SignPictureAnalyzer::hasTrainShape(const cv::Mat& darkMask)
{
    if (darkMask.empty())
    {
        return false;
    }

    cv::Mat cleaned;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(darkMask, cleaned, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cleaned, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int wideBodyCount = 0;
    int smallRoundCount = 0;

    for (const std::vector<cv::Point>& contour : contours)
    {
        const cv::Rect box = cv::boundingRect(contour);
        const int area = box.width * box.height;
        if (area < 30)
        {
            continue;
        }

        const double widthToHeight = static_cast<double>(box.width) / std::max(1, box.height);
        const double heightToWidth = static_cast<double>(box.height) / std::max(1, box.width);
        if (area > 150 && widthToHeight > 1.4)
        {
            ++wideBodyCount;
        }
        if (area >= 30 && area <= 250 && widthToHeight < 1.5 && heightToWidth < 1.5)
        {
            ++smallRoundCount;
        }
    }

    return wideBodyCount >= 1 && smallRoundCount >= 2;
}

} // namespace rsd
