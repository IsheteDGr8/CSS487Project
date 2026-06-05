/*
 * ColorSegmenter.h
 *
 * Purpose: Builds binary color masks from BGR frames so ShapeAnalyzer can find signs.
 * Authors: Ishaan, Manish
 *
 * Assumptions:
 *   - Input images are 3-channel BGR (OpenCV default).
 *   - HSV thresholds were tuned on our test set and stay fixed at runtime.
 *   - Tuner methods are dev tools only; the demo uses getStatic*Mask().
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class ColorSegmenter
{
public:
    // Purpose: Default constructor; no setup needed.
    // Pre-condition: None.
    // Post-condition: Object is ready to call mask methods.
    ColorSegmenter() = default;

    // Purpose: Interactive sliders to tune red HSV thresholds on one image.
    // Pre-condition: imagePath points to a readable image file.
    // Post-condition: Opens OpenCV windows; blocks until ESC; windows are closed.
    void tuneRedMask(const std::string &imagePath) const;

    // Purpose: Interactive sliders to tune yellow HSV thresholds on one image.
    // Pre-condition: imagePath points to a readable image file.
    // Post-condition: Opens OpenCV windows; blocks until ESC; windows are closed.
    void tuneYellowMask(const std::string &imagePath) const;

    // Purpose: Interactive sliders to tune blue HSV thresholds on one image.
    // Pre-condition: imagePath points to a readable image file.
    // Post-condition: Opens OpenCV windows; blocks until ESC; windows are closed.
    void tuneBlueMask(const std::string &imagePath) const;

    // Purpose: Red mask for stop / do-not-enter signs (wraps hue at 0 and 180).
    // Pre-condition: inputImage is non-empty BGR.
    // Post-condition: Returns 8-bit single-channel mask (255 = red pixel).
    cv::Mat getStaticRedMask(const cv::Mat &inputImage) const;

    // Purpose: Yellow mask for warning signs.
    // Pre-condition: inputImage is non-empty BGR.
    // Post-condition: Returns 8-bit single-channel mask (255 = yellow pixel).
    cv::Mat getStaticYellowMask(const cv::Mat &inputImage) const;

    // Purpose: Blue mask for service signs.
    // Pre-condition: inputImage is non-empty BGR.
    // Post-condition: Returns 8-bit single-channel mask (255 = blue pixel).
    cv::Mat getStaticBlueMask(const cv::Mat &inputImage) const;

    // Purpose: Orange mask for construction signs.
    // Pre-condition: inputImage is non-empty BGR.
    // Post-condition: Returns 8-bit single-channel mask (255 = orange pixel).
    cv::Mat getStaticOrangeMask(const cv::Mat &inputImage) const;

    // Purpose: Green mask for guide signs.
    // Pre-condition: inputImage is non-empty BGR.
    // Post-condition: Returns 8-bit single-channel mask (255 = green pixel).
    cv::Mat getStaticGreenMask(const cv::Mat &inputImage) const;

    // Purpose: White mask for speed-limit panels (grayscale + red removed).
    // Pre-condition: inputImage is non-empty BGR.
    // Post-condition: Returns 8-bit single-channel mask (255 = white sign area).
    cv::Mat getStaticWhiteMask(const cv::Mat &inputImage) const;
};
