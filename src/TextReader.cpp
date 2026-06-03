/*
 * File: TextReader.cpp
 * Project: Road Sign and Traffic Light Detection System
 * Author: Manish Ram
 *
 * Purpose:
 *   Implements simple text and speed number reading with classical OpenCV.
 */

#include "RoadSignDetector/TextReader.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <limits>

namespace rsd
{
namespace
{
constexpr int kTemplateWidth = 36;
constexpr int kTemplateHeight = 52;
const std::string kLettersAndDigits = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
const std::string kDigits = "0123456789";
} // namespace

std::string TextReader::readText(const cv::Mat& bgrImage, const cv::Rect& signBox) const
{
    const std::vector<GlyphBox> glyphs = findGlyphs(bgrImage, signBox);
    if (glyphs.empty())
    {
        return "";
    }

    std::string rawText;
    int previousRight = -1;

    for (const GlyphBox& glyph : glyphs)
    {
        if (previousRight >= 0)
        {
            const int gap = glyph.box.x - previousRight;
            if (gap > glyph.box.height / 2)
            {
                rawText.push_back(' ');
            }
        }

        const TextMatch match = matchGlyph(glyph.image, kLettersAndDigits);
        if (match.score >= 0.25)
        {
            rawText.push_back(match.value);
        }

        previousRight = glyph.box.x + glyph.box.width;
    }

    return cleanWords(rawText);
}

std::string TextReader::readSpeedNumber(const cv::Mat& bgrImage, const cv::Rect& signBox) const
{
    const std::vector<GlyphBox> glyphs = findGlyphs(bgrImage, signBox);
    std::string digits;

    for (const GlyphBox& glyph : glyphs)
    {
        const TextMatch match = matchGlyph(glyph.image, kDigits);
        if (match.score >= 0.28)
        {
            digits.push_back(match.value);
        }
    }

    if (digits.size() > 3)
    {
        digits.resize(3);
    }

    return digits;
}

std::vector<TextReader::GlyphBox> TextReader::findGlyphs(
    const cv::Mat& bgrImage,
    const cv::Rect& signBox) const
{
    std::vector<GlyphBox> glyphs;
    if (bgrImage.empty() || signBox.empty())
    {
        return glyphs;
    }

    const cv::Rect safeBox = keepInsideImage(signBox, bgrImage.size());
    if (safeBox.empty())
    {
        return glyphs;
    }

    cv::Mat darkMask = makeDarkMask(bgrImage(safeBox));
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::morphologyEx(darkMask, darkMask, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(darkMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const std::vector<cv::Point>& contour : contours)
    {
        const cv::Rect box = cv::boundingRect(contour);
        const int area = box.width * box.height;
        const double heightRatio = static_cast<double>(box.height) / safeBox.height;
        const double widthRatio = static_cast<double>(box.width) / safeBox.width;

        if (area < 35 || heightRatio < 0.10 || heightRatio > 0.82 || widthRatio > 0.55)
        {
            continue;
        }

        cv::Mat glyphImage = darkMask(box).clone();
        cv::resize(glyphImage, glyphImage, cv::Size(kTemplateWidth, kTemplateHeight));
        glyphs.push_back({box, glyphImage});
    }

    std::sort(
        glyphs.begin(),
        glyphs.end(),
        [](const GlyphBox& left, const GlyphBox& right)
        {
            const int rowTolerance = std::max(left.box.height, right.box.height) / 2;
            if (std::abs(left.box.y - right.box.y) > rowTolerance)
            {
                return left.box.y < right.box.y;
            }
            return left.box.x < right.box.x;
        });

    return glyphs;
}

TextMatch TextReader::matchGlyph(const cv::Mat& glyphImage, const std::string& alphabet) const
{
    TextMatch bestMatch;
    bestMatch.score = -std::numeric_limits<double>::infinity();

    for (const char value : alphabet)
    {
        const cv::Mat templateImage = makeTemplate(value);
        cv::Mat result;
        cv::matchTemplate(glyphImage, templateImage, result, cv::TM_CCOEFF_NORMED);

        double score = 0.0;
        cv::minMaxLoc(result, nullptr, &score);
        if (score > bestMatch.score)
        {
            bestMatch.value = value;
            bestMatch.score = score;
        }
    }

    return bestMatch;
}

cv::Mat TextReader::makeDarkMask(const cv::Mat& bgrImage)
{
    cv::Mat hsvImage;
    cv::cvtColor(bgrImage, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat darkMask;
    cv::inRange(hsvImage, cv::Scalar(0, 0, 0), cv::Scalar(180, 130, 145), darkMask);
    return darkMask;
}

cv::Mat TextReader::makeTemplate(const char value)
{
    cv::Mat image = cv::Mat::zeros(kTemplateHeight, kTemplateWidth, CV_8UC1);
    const std::string text(1, value);
    const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 1.35;
    const int thickness = 3;

    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    const cv::Point origin(
        std::max(0, (kTemplateWidth - textSize.width) / 2),
        std::max(textSize.height, (kTemplateHeight + textSize.height) / 2 - 4));

    cv::putText(image, text, origin, fontFace, fontScale, cv::Scalar(255), thickness);
    return image;
}

cv::Rect TextReader::keepInsideImage(const cv::Rect& box, const cv::Size& imageSize)
{
    const cv::Rect imageBounds(0, 0, imageSize.width, imageSize.height);
    return box & imageBounds;
}

std::string TextReader::cleanWords(const std::string& rawText)
{
    std::string cleaned;
    bool lastWasSpace = true;

    for (const char value : rawText)
    {
        if (std::isalnum(static_cast<unsigned char>(value)))
        {
            cleaned.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(value))));
            lastWasSpace = false;
        }
        else if (!lastWasSpace)
        {
            cleaned.push_back(' ');
            lastWasSpace = true;
        }
    }

    if (!cleaned.empty() && cleaned.back() == ' ')
    {
        cleaned.pop_back();
    }

    return cleaned;
}

} // namespace rsd
