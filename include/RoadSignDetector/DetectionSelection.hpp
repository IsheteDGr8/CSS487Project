/*
 * File: DetectionSelection.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares helper logic for choosing the strongest detections before they
 *   are shown in the command-line demo or drawn on debug images.
 */

#ifndef ROAD_SIGN_DETECTOR_DETECTION_SELECTION_HPP
#define ROAD_SIGN_DETECTOR_DETECTION_SELECTION_HPP

#include "RoadSignDetector/DetectionTypes.hpp"

#include <cstddef>
#include <vector>

namespace rsd
{

/*
 * Returns the strongest detections by confidence score.
 *
 * Preconditions:
 *   - maximumCount is the largest number of detections the caller wants.
 *
 * Postconditions:
 *   - The returned vector contains at most maximumCount detections.
 *   - Detections are ordered from highest confidence to lowest confidence.
 *   - When scores tie, the larger contour area is preferred.
 */
[[nodiscard]] std::vector<Detection> chooseStrongestDetections(
    const std::vector<Detection>& detections,
    std::size_t maximumCount);

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_DETECTION_SELECTION_HPP
