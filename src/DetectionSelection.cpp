/*
 * File: DetectionSelection.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Implements shared detection selection used by terminal and image output.
 */

#include "RoadSignDetector/DetectionSelection.hpp"

#include <algorithm>

namespace rsd
{

std::vector<Detection> chooseStrongestDetections(
    const std::vector<Detection>& detections,
    const std::size_t maximumCount)
{
    std::vector<Detection> strongestDetections = detections;

    std::stable_sort(
        strongestDetections.begin(),
        strongestDetections.end(),
        [](const Detection& left, const Detection& right)
        {
            if (left.confidence == right.confidence)
            {
                return left.features.area > right.features.area;
            }

            return left.confidence > right.confidence;
        });

    if (strongestDetections.size() > maximumCount)
    {
        strongestDetections.resize(maximumCount);
    }

    return strongestDetections;
}

} // namespace rsd
