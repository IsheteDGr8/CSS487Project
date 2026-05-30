/*
 * File: main.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Provides a command-line demo for the detection pipeline. The demo includes
 *   simple HSV masks so this executable can be tested independently.
 */

#include "RoadSignDetector/DebugImageWriter.hpp"
#include "RoadSignDetector/DisplayOnTerminal.hpp"
#include "RoadSignDetector/HsvMaskSegmenter.hpp"
#include "RoadSignDetector/RoadObjectDetector.hpp"

#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <string>
#include <vector>

int main(const int argc, char* argv[])
{
    const rsd::DisplayOnTerminal terminalDisplay;

    if (argc < 2)
    {
        terminalDisplay.printUsage(argv[0], std::cout);
        return 0;
    }

    const std::string imagePath = argv[1];
    const cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (image.empty())
    {
        std::cerr << "Could not read image: " << imagePath << '\n';
        return 1;
    }

    const rsd::HsvMaskSegmenter segmenter;
    const std::vector<rsd::MaskInput> masks = segmenter.createMasks(image);

    const rsd::RoadObjectDetector detector;
    const std::vector<rsd::Detection> detections = detector.detect(image, masks);

    terminalDisplay.printDetections(detections, std::cout);

    if (argc >= 3)
    {
        const rsd::DebugImageWriter debugWriter;
        debugWriter.save(argv[2], image, detections, masks);
        std::cout << "Saved debug images to: " << argv[2] << '\n';
    }

    return 0;
}
