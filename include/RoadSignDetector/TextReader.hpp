/*
 * File: TextReader.hpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Declares a small classical text reader. It uses thresholding, connected
 *   components, and template matching against generated OpenCV font glyphs.
 */

#ifndef ROAD_SIGN_DETECTOR_TEXT_READER_HPP
#define ROAD_SIGN_DETECTOR_TEXT_READER_HPP

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace rsd
{

struct TextMatch
{
    char value = '?';
    double score = 0.0;
};

class TextReader
{
public:
    [[nodiscard]] std::string readText(const cv::Mat& bgrImage, const cv::Rect& signBox) const;
    [[nodiscard]] std::string readSpeedNumber(const cv::Mat& bgrImage, const cv::Rect& signBox) const;

private:
    struct GlyphBox
    {
        cv::Rect box;
        cv::Mat image;
    };

    [[nodiscard]] std::vector<GlyphBox> findGlyphs(const cv::Mat& bgrImage, const cv::Rect& signBox) const;
    [[nodiscard]] TextMatch matchGlyph(const cv::Mat& glyphImage, const std::string& alphabet) const;
    [[nodiscard]] static cv::Mat makeDarkMask(const cv::Mat& bgrImage);
    [[nodiscard]] static cv::Mat makeTemplate(char value);
    [[nodiscard]] static cv::Rect keepInsideImage(const cv::Rect& box, const cv::Size& imageSize);
    [[nodiscard]] static std::string cleanWords(const std::string& rawText);
};

} // namespace rsd

#endif // ROAD_SIGN_DETECTOR_TEXT_READER_HPP
