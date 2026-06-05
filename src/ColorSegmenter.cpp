/*
 * ColorSegmenter.cpp
 *
 * Purpose: Implements HSV masking and optional slider tuning windows.
 * Authors: Ishaan, Manish
 *
 * Assumptions:
 *   - Same as ColorSegmenter.h.
 *   - Gaussian blur (5x5) is applied before every mask pass.
 */
#include "../include/RoadSignDetector/ColorSegmenter.h"
#include <algorithm>
#include <iostream>
#include <vector>

namespace
{
    struct TunerContext
    {
        cv::Mat hsvImage;
        int lowerH1 = 0, upperH1 = 10;
        int lowerH2 = 170, upperH2 = 180;
        int lowerS = 120, upperS = 255;
        int lowerV = 70, upperV = 255;
        int morphSize = 3;
    };

    void updateRedMask(int, void *userdata)
    {
        TunerContext *ctx = static_cast<TunerContext *>(userdata);
        cv::Mat mask1, mask2, combinedMask;

        cv::inRange(ctx->hsvImage, cv::Scalar(ctx->lowerH1, ctx->lowerS, ctx->lowerV), cv::Scalar(ctx->upperH1, ctx->upperS, ctx->upperV), mask1);
        cv::inRange(ctx->hsvImage, cv::Scalar(ctx->lowerH2, ctx->lowerS, ctx->lowerV), cv::Scalar(ctx->upperH2, ctx->upperS, ctx->upperV), mask2);
        cv::bitwise_or(mask1, mask2, combinedMask);

        int kSize = ctx->morphSize;
        if (kSize % 2 == 0)
            kSize += 1;
        if (kSize < 1)
            kSize = 1;

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kSize, kSize));
        cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_OPEN, element);
        cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, element);

        cv::imshow("Filtered Mask Result", combinedMask);
    }

    struct SingleColorContext
    {
        cv::Mat hsvImage;
        int lowerH = 0, upperH = 180;
        int lowerS = 100, upperS = 255;
        int lowerV = 100, upperV = 255;
        int morphSize = 3;
    };

    void updateSingleMask(int, void *userdata)
    {
        SingleColorContext *ctx = static_cast<SingleColorContext *>(userdata);
        cv::Mat mask;

        cv::inRange(ctx->hsvImage,
                    cv::Scalar(ctx->lowerH, ctx->lowerS, ctx->lowerV),
                    cv::Scalar(ctx->upperH, ctx->upperS, ctx->upperV),
                    mask);

        int kSize = ctx->morphSize;
        if (kSize % 2 == 0)
            kSize += 1;
        if (kSize < 1)
            kSize = 1;

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kSize, kSize));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);

        cv::imshow("Filtered Mask Result", mask);
    }
}

// Purpose: Interactive sliders to tune red HSV thresholds on one image.
// Pre-condition: imagePath points to a readable image file.
// Post-condition: Opens OpenCV windows; blocks until ESC; windows are closed.
void ColorSegmenter::tuneRedMask(const std::string &imagePath) const
{
    cv::Mat rawImage = cv::imread(imagePath);
    if (rawImage.empty())
        return;

    int targetWidth = 600;
    int targetHeight = (int)(rawImage.rows * ((double)targetWidth / rawImage.cols));
    cv::resize(rawImage, rawImage, cv::Size(targetWidth, targetHeight));

    cv::Mat blurredImage;
    cv::GaussianBlur(rawImage, blurredImage, cv::Size(5, 5), 0);

    static TunerContext context;
    cv::cvtColor(blurredImage, context.hsvImage, cv::COLOR_BGR2HSV);

    cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Red Control Panel", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Filtered Mask Result", cv::WINDOW_AUTOSIZE);

    cv::imshow("Original Image", rawImage);
    cv::createTrackbar("Lower Hue 1", "Red Control Panel", &context.lowerH1, 180, updateRedMask, &context);
    cv::createTrackbar("Upper Hue 1", "Red Control Panel", &context.upperH1, 180, updateRedMask, &context);
    cv::createTrackbar("Lower Hue 2", "Red Control Panel", &context.lowerH2, 180, updateRedMask, &context);
    cv::createTrackbar("Upper Hue 2", "Red Control Panel", &context.upperH2, 180, updateRedMask, &context);
    cv::createTrackbar("Min Sat", "Red Control Panel", &context.lowerS, 255, updateRedMask, &context);
    cv::createTrackbar("Max Sat", "Red Control Panel", &context.upperS, 255, updateRedMask, &context);
    cv::createTrackbar("Min Val", "Red Control Panel", &context.lowerV, 255, updateRedMask, &context);
    cv::createTrackbar("Max Val", "Red Control Panel", &context.upperV, 255, updateRedMask, &context);
    cv::createTrackbar("Morph", "Red Control Panel", &context.morphSize, 15, updateRedMask, &context);

    updateRedMask(0, &context);
    while (cv::waitKey(10) != 27)
    {
    }
    cv::destroyAllWindows();
}

// Purpose: Interactive sliders to tune yellow HSV thresholds on one image.
// Pre-condition: imagePath points to a readable image file.
// Post-condition: Opens OpenCV windows; blocks until ESC; windows are closed.
void ColorSegmenter::tuneYellowMask(const std::string &imagePath) const
{
    cv::Mat rawImage = cv::imread(imagePath);
    if (rawImage.empty())
        return;

    int targetWidth = 600;
    int targetHeight = (int)(rawImage.rows * ((double)targetWidth / rawImage.cols));
    cv::resize(rawImage, rawImage, cv::Size(targetWidth, targetHeight));

    cv::Mat blurredImage;
    cv::GaussianBlur(rawImage, blurredImage, cv::Size(5, 5), 0);

    static SingleColorContext context;
    context.lowerH = 15;
    context.upperH = 35;
    cv::cvtColor(blurredImage, context.hsvImage, cv::COLOR_BGR2HSV);

    cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Yellow Control Panel", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Filtered Mask Result", cv::WINDOW_AUTOSIZE);

    cv::imshow("Original Image", rawImage);
    cv::createTrackbar("Lower Hue", "Yellow Control Panel", &context.lowerH, 180, updateSingleMask, &context);
    cv::createTrackbar("Upper Hue", "Yellow Control Panel", &context.upperH, 180, updateSingleMask, &context);
    cv::createTrackbar("Min Sat", "Yellow Control Panel", &context.lowerS, 255, updateSingleMask, &context);
    cv::createTrackbar("Max Sat", "Yellow Control Panel", &context.upperS, 255, updateSingleMask, &context);
    cv::createTrackbar("Min Val", "Yellow Control Panel", &context.lowerV, 255, updateSingleMask, &context);
    cv::createTrackbar("Max Val", "Yellow Control Panel", &context.upperV, 255, updateSingleMask, &context);
    cv::createTrackbar("Morph", "Yellow Control Panel", &context.morphSize, 15, updateSingleMask, &context);

    updateSingleMask(0, &context);
    while (cv::waitKey(10) != 27)
    {
    }
    cv::destroyAllWindows();
}

// Purpose: Interactive sliders to tune blue HSV thresholds on one image.
// Pre-condition: imagePath points to a readable image file.
// Post-condition: Opens OpenCV windows; blocks until ESC; windows are closed.
void ColorSegmenter::tuneBlueMask(const std::string &imagePath) const
{
    cv::Mat rawImage = cv::imread(imagePath);
    if (rawImage.empty())
        return;

    int targetWidth = 600;
    int targetHeight = (int)(rawImage.rows * ((double)targetWidth / rawImage.cols));
    cv::resize(rawImage, rawImage, cv::Size(targetWidth, targetHeight));

    cv::Mat blurredImage;
    cv::GaussianBlur(rawImage, blurredImage, cv::Size(5, 5), 0);

    static SingleColorContext context;
    context.lowerH = 90;
    context.upperH = 130;
    cv::cvtColor(blurredImage, context.hsvImage, cv::COLOR_BGR2HSV);

    cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Blue Control Panel", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Filtered Mask Result", cv::WINDOW_AUTOSIZE);

    cv::imshow("Original Image", rawImage);
    cv::createTrackbar("Lower Hue", "Blue Control Panel", &context.lowerH, 180, updateSingleMask, &context);
    cv::createTrackbar("Upper Hue", "Blue Control Panel", &context.upperH, 180, updateSingleMask, &context);
    cv::createTrackbar("Min Sat", "Blue Control Panel", &context.lowerS, 255, updateSingleMask, &context);
    cv::createTrackbar("Max Sat", "Blue Control Panel", &context.upperS, 255, updateSingleMask, &context);
    cv::createTrackbar("Min Val", "Blue Control Panel", &context.lowerV, 255, updateSingleMask, &context);
    cv::createTrackbar("Max Val", "Blue Control Panel", &context.upperV, 255, updateSingleMask, &context);
    cv::createTrackbar("Morph", "Blue Control Panel", &context.morphSize, 15, updateSingleMask, &context);

    updateSingleMask(0, &context);
    while (cv::waitKey(10) != 27)
    {
    }
    cv::destroyAllWindows();
}

// Purpose: Red mask for stop / do-not-enter signs (wraps hue at 0 and 180).
// Pre-condition: inputImage is non-empty BGR.
// Post-condition: Returns 8-bit single-channel mask (255 = red pixel).
cv::Mat ColorSegmenter::getStaticRedMask(const cv::Mat &inputImage) const
{
    cv::Mat blurredImage, hsvImage;
    cv::GaussianBlur(inputImage, blurredImage, cv::Size(5, 5), 0);
    cv::cvtColor(blurredImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, combinedMask;
    cv::inRange(hsvImage, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, combinedMask);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_OPEN, element);
    cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, element);

    return combinedMask;
}

// Purpose: Yellow mask for warning signs.
// Pre-condition: inputImage is non-empty BGR.
// Post-condition: Returns 8-bit single-channel mask (255 = yellow pixel).
cv::Mat ColorSegmenter::getStaticYellowMask(const cv::Mat &inputImage) const
{
    cv::Mat blurredImage, hsvImage;
    cv::GaussianBlur(inputImage, blurredImage, cv::Size(5, 5), 0);
    cv::cvtColor(blurredImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsvImage, cv::Scalar(15, 100, 100), cv::Scalar(35, 255, 255), mask);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);

    return mask;
}

// Purpose: Blue mask for service signs.
// Pre-condition: inputImage is non-empty BGR.
// Post-condition: Returns 8-bit single-channel mask (255 = blue pixel).
cv::Mat ColorSegmenter::getStaticBlueMask(const cv::Mat &inputImage) const
{
    cv::Mat blurredImage, hsvImage;
    cv::GaussianBlur(inputImage, blurredImage, cv::Size(5, 5), 0);
    cv::cvtColor(blurredImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    // Bump min saturation so pale sky doesn't count as blue.
    cv::inRange(hsvImage, cv::Scalar(90, 80, 50), cv::Scalar(130, 255, 255), mask);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);

    return mask;
}

// Purpose: Orange mask for construction signs.
// Pre-condition: inputImage is non-empty BGR.
// Post-condition: Returns 8-bit single-channel mask (255 = orange pixel).
cv::Mat ColorSegmenter::getStaticOrangeMask(const cv::Mat &inputImage) const
{
    cv::Mat blurredImage, hsvImage;
    cv::GaussianBlur(inputImage, blurredImage, cv::Size(5, 5), 0);
    cv::cvtColor(blurredImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsvImage, cv::Scalar(3, 100, 80), cv::Scalar(25, 255, 255), mask);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);

    return mask;
}

// Purpose: Green mask for guide signs.
// Pre-condition: inputImage is non-empty BGR.
// Post-condition: Returns 8-bit single-channel mask (255 = green pixel).
cv::Mat ColorSegmenter::getStaticGreenMask(const cv::Mat &inputImage) const
{
    cv::Mat blurredImage, hsvImage;
    cv::GaussianBlur(inputImage, blurredImage, cv::Size(5, 5), 0);
    cv::cvtColor(blurredImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsvImage, cv::Scalar(35, 100, 50), cv::Scalar(85, 255, 255), mask);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);

    return mask;
}

// Purpose: White mask for speed-limit panels (grayscale + red removed).
// Pre-condition: inputImage is non-empty BGR.
// Post-condition: Returns 8-bit single-channel mask (255 = white sign area).
cv::Mat ColorSegmenter::getStaticWhiteMask(const cv::Mat &inputImage) const
{
    cv::Mat blurredImage, grayImage, hsvImage;
    cv::GaussianBlur(inputImage, blurredImage, cv::Size(5, 5), 0);
    cv::cvtColor(blurredImage, grayImage, cv::COLOR_BGR2GRAY);
    cv::cvtColor(blurredImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat whiteMask, redMask1, redMask2, redMask, cleanedMask;
    cv::inRange(grayImage, 175, 255, whiteMask);

    cv::inRange(hsvImage, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), redMask1);
    cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);
    cv::bitwise_and(whiteMask, ~redMask, whiteMask);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(whiteMask, whiteMask, cv::MORPH_OPEN, element);
    cv::morphologyEx(whiteMask, whiteMask, cv::MORPH_CLOSE, element);

    cleanedMask = cv::Mat::zeros(whiteMask.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(whiteMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    struct WhiteCandidate
    {
        double area;
        cv::Rect boundingBox;
        std::vector<cv::Point> contour;
    };

    std::vector<WhiteCandidate> candidates;
    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < 250.0)
            continue;

        candidates.push_back({area, cv::boundingRect(contour), contour});
    }

    if (candidates.empty())
        return cleanedMask;

    std::sort(candidates.begin(), candidates.end(),
              [](const WhiteCandidate &a, const WhiteCandidate &b)
              { return a.area > b.area; });

    double frameArea = inputImage.rows * inputImage.cols;
    double largestArea = candidates.front().area;

    double largestEligibleArea = 0.0;
    for (const auto &candidate : candidates)
    {
        double centerY = candidate.boundingBox.y + candidate.boundingBox.height / 2.0;
        if (centerY > inputImage.rows * 0.72)
            continue;

        if ((double)candidate.boundingBox.width > candidate.boundingBox.height * 2.0)
            continue;

        largestEligibleArea = std::max(largestEligibleArea, candidate.area);
    }

    // One big white panel (close-up speed limit shot).
    if (largestArea > frameArea * 0.20)
    {
        cv::drawContours(cleanedMask, std::vector<std::vector<cv::Point>>{candidates.front().contour},
                         -1, cv::Scalar(255), cv::FILLED);
        return cleanedMask;
    }

    // Street scene: keep sign-sized panels up top, ignore lane stripes below.
    const double referenceArea = std::max(largestEligibleArea, 1.0);
    for (const auto &candidate : candidates)
    {
        if (candidate.area < referenceArea * 0.20)
            continue;

        double centerY = candidate.boundingBox.y + candidate.boundingBox.height / 2.0;
        if (centerY > inputImage.rows * 0.72)
            continue;

        if ((double)candidate.boundingBox.width > candidate.boundingBox.height * 2.0)
            continue;

        cv::drawContours(cleanedMask, std::vector<std::vector<cv::Point>>{candidate.contour},
                         -1, cv::Scalar(255), cv::FILLED);
    }

    return cleanedMask;
}