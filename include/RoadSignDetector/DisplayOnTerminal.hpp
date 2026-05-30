/*
 * File: DisplayOnTerminal.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares console reporting utilities for numerical detection output.
 */

#ifndef ROAD_SIGN_DETECTOR_DISPLAY_ON_TERMINAL_HPP
#define ROAD_SIGN_DETECTOR_DISPLAY_ON_TERMINAL_HPP

#include "RoadSignDetector/DetectionTypes.hpp"

#include <iosfwd>
#include <string>
#include <vector>

namespace rsd
{

class DisplayOnTerminal
{
public:
    /*
     * Prints one row per detection.
     *
     * Preconditions:
     *   - output must be a writable stream.
     */
    void printDetections(
        const std::vector<Detection>& detections,
        std::ostream& output) const;

    void printUsage(const std::string& executableName, std::ostream& output) const;
};

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_DISPLAY_ON_TERMINAL_HPP
