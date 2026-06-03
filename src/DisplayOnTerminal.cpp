/*
 * File: DisplayOnTerminal.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Implements console output for detection results.
 */

#include "RoadSignDetector/DisplayOnTerminal.hpp"
#include "RoadSignDetector/DetectionSelection.hpp"

#include <cstddef>
#include <iomanip>
#include <iostream>

namespace rsd
{

namespace
{

constexpr std::size_t kMaximumDisplayedDetections = 2U;

} // namespace

void DisplayOnTerminal::printDetections(
    const std::vector<Detection>& detections,
    std::ostream& output) const
{
    if (detections.empty())
    {
        output << "No detections passed the heuristic filters." << '\n';
        return;
    }

    output << std::left
           << std::setw(28) << "type"
           << std::setw(10) << "color"
           << std::setw(16) << "text"
           << std::setw(12) << "area"
           << std::setw(10) << "verts"
           << std::setw(14) << "circularity"
           << std::setw(10) << "hough"
           << "confidence" << '\n';

    for (const Detection& detection : chooseStrongestDetections(
             detections, kMaximumDisplayedDetections))
    {
        output << std::left
               << std::setw(28) << toString(detection.type)
               << std::setw(10) << toString(detection.color)
               << std::setw(16) << detection.text
               << std::setw(12) << static_cast<int>(detection.features.area)
               << std::setw(10) << detection.features.vertexCount
               << std::setw(14) << std::fixed << std::setprecision(3)
               << detection.features.circularity
               << std::setw(10) << (detection.features.hasHoughCircle ? "yes" : "no")
               << std::setprecision(2) << detection.confidence << '\n';
    }
}

void DisplayOnTerminal::printUsage(
    const std::string& executableName,
    std::ostream& output) const
{
    output << "Usage: " << executableName << " <image_path> [output_directory]" << '\n';
    output << "Example: " << executableName << " samples/intersection.jpg output" << '\n';
}

} // namespace rsd
