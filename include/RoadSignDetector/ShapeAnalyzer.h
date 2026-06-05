/*
 * ShapeAnalyzer.h
 *
 * Purpose: Finds sign-shaped regions in color masks and draws labels on the frame.
 * Authors: Ishaan, Manish
 *
 * Assumptions:
 *   - Masks come from ColorSegmenter and match outputImage size.
 *   - US-style MUTCD shapes and colors; not all regulatory variants are handled.
 *   - Speed-limit OCR expects two large digits on a white rectangle.
 */
#pragma once
#include <opencv2/opencv.hpp>

class ShapeAnalyzer
{
public:
    // Purpose: Default constructor; no setup needed.
    // Pre-condition: None.
    // Post-condition: Object is ready to call detect* methods.
    ShapeAnalyzer() = default;

    // Purpose: Detect stop, do-not-enter, and speed-limit signs.
    // Pre-condition: bgrImage, redMask, whiteMask same size; outputImage is BGR.
    // Post-condition: Draws boxes and labels on outputImage for matches found.
    void detectRegulatorySigns(const cv::Mat &bgrImage, const cv::Mat &redMask,
                               const cv::Mat &whiteMask, cv::Mat &outputImage) const;

    // Purpose: Detect yellow diamond warning signs.
    // Pre-condition: binaryMask is 8-bit single-channel; outputImage is BGR.
    // Post-condition: Draws yellow boxes and "WARNING SIGN" on outputImage.
    void detectWarningSign(const cv::Mat &binaryMask, cv::Mat &outputImage) const;

    // Purpose: Detect orange diamond construction signs.
    // Pre-condition: binaryMask is 8-bit single-channel; outputImage is BGR.
    // Post-condition: Draws orange boxes and "CONSTRUCTION SIGN" on outputImage.
    void detectConstructionSign(const cv::Mat &binaryMask, cv::Mat &outputImage) const;

    // Purpose: Detect green rectangular guide signs.
    // Pre-condition: binaryMask is 8-bit single-channel; outputImage is BGR.
    // Post-condition: Draws green boxes and "GUIDE SIGN" on outputImage.
    void detectGuideSign(const cv::Mat &binaryMask, cv::Mat &outputImage) const;

    // Purpose: Detect blue rectangular service signs.
    // Pre-condition: binaryMask is 8-bit single-channel; outputImage is BGR.
    // Post-condition: Draws blue boxes and "SERVICE SIGN" on outputImage.
    void detectServiceSign(const cv::Mat &binaryMask, cv::Mat &outputImage) const;
};
