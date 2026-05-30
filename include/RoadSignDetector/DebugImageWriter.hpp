/*
 * File: DebugImageWriter.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares helper functions for writing visual debugging artifacts. This
 *   keeps image output separate from the detection algorithm.
 */

#ifndef ROAD_SIGN_DETECTOR_DETECTION_DEBUG_WRITER_HPP
#define ROAD_SIGN_DETECTOR_DETECTION_DEBUG_WRITER_HPP

#include "RoadSignDetector/DetectionTypes.hpp"

#include <opencv2/core.hpp>

#include <filesystem>
#include <vector>

namespace rsd
{

/*
 * Writes annotated images and intermediate masks for inspection.
 *   - The writer stores no image data.
 *   - All inputs are read-only; annotated copies are created internally.
 */
class DebugImageWriter
{
public:
    /*
     * Saves annotated.png plus one mask image per color.
     *
     * Preconditions:
     *   - outputDirectory must be a writable path or creatable directory.
     *   - bgrImage must be a valid BGR image.
     *
     * Postconditions:
     *   - Creates outputDirectory if needed.
     *   - Writes debug images to disk using cv::imwrite.
     */
    void save(
        const std::filesystem::path& outputDirectory,
        const cv::Mat& bgrImage,
        const std::vector<Detection>& detections,
        const std::vector<MaskInput>& masks) const;

private:
    [[nodiscard]] cv::Mat createAnnotatedImage(
        const cv::Mat& bgrImage,
        const std::vector<Detection>& detections) const;

    static void drawDetection(cv::Mat& image, const Detection& detection);
};

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_DETECTION_DEBUG_WRITER_HPP
