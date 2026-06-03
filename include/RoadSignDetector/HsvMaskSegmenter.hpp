/*
 * File: HsvMaskSegmenter.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares a lightweight HSV color segmenter used by the command line demo for my this class 487.
 *   The final segmentation code can replace this class later without changing
 *   the detector.
 */

#ifndef ROAD_SIGN_DETECTOR_HSV_MASK_SEGMENTER_HPP
#define ROAD_SIGN_DETECTOR_HSV_MASK_SEGMENTER_HPP

#include "RoadSignDetector/DetectionTypes.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace rsd
{

/*
 * HSV color segmenter for baseline red/yellow/green/blue masks.
 *
 *  produce clean binary
 * masks so the detection logic can be run and tested before the final
 * segmentation module is ready.
 */
class HsvMaskSegmenter
{
public:
    /*
     * Creates cleaned binary masks for common road sign and traffic light
     * colors.
     *
     * Preconditions:
     *   - bgrImage must be a non empty 8-bit BGR image.
     *
     * Postconditions:
     *   - Returns one singl3 channel mask for each configured color family.
     *   - Does not modify bgrImage.
     */
    [[nodiscard]] std::vector<MaskInput> createMasks(const cv::Mat& bgrImage) const;

private:
    [[nodiscard]] static cv::Mat cleanMask(const cv::Mat& rawMask);
};

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_HSV_MASK_SEGMENTER_HPP
