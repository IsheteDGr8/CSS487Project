/*
 * ShapeAnalyzer.cpp
 *
 * Purpose: Contour and shape checks, speed-limit digit reading, and on-frame labels.
 * Authors: Ishaan
 *
 * Assumptions:
 *   - Same as ShapeAnalyzer.h.
 *   - Contour area and aspect-ratio gates reject most background blobs.
 */
#include "RoadSignDetector/ShapeAnalyzer.h"
#include <algorithm>
#include <string>

namespace
{
    bool isNestedInside(const cv::Rect &inner, const cv::Rect &outer)
    {
        return inner.x >= outer.x && inner.y >= outer.y &&
               inner.x + inner.width <= outer.x + outer.width &&
               inner.y + inner.height <= outer.y + outer.height;
    }

    int labelY(const cv::Rect &boundingBox)
    {
        return std::max(boundingBox.y - 8, 22);
    }

    bool isLikelySkyBlob(const cv::Rect &boundingBox, double fillRatio, int imageWidth)
    {
        if (boundingBox.y <= 2 && fillRatio < 0.65)
            return true;

        return boundingBox.y <= 2 &&
               boundingBox.width >= static_cast<int>(imageWidth * 0.92);
    }

    bool overlapsAny(const cv::Rect &box, const std::vector<cv::Rect> &claimed, double minOverlapRatio = 0.25)
    {
        const double boxArea = static_cast<double>(box.area());
        if (boxArea <= 0.0)
            return false;

        for (const auto &other : claimed)
        {
            cv::Rect intersection = box & other;
            if (intersection.area() > boxArea * minOverlapRatio)
                return true;
        }
        return false;
    }

    void drawRegulatoryLabel(cv::Mat &outputImage, const cv::Rect &box, const std::string &label,
                             const cv::Scalar &color)
    {
        cv::rectangle(outputImage, box.tl(), box.br(), color, 3);
        cv::putText(outputImage, label, cv::Point(box.x, labelY(box)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
    }

    double innerWhiteRatio(const cv::Mat &bgrImage, const cv::Rect &boundingBox)
    {
        cv::Rect clipped = boundingBox & cv::Rect(0, 0, bgrImage.cols, bgrImage.rows);
        if (clipped.width < 10 || clipped.height < 10)
            return 0.0;

        cv::Mat gray;
        cv::cvtColor(bgrImage(clipped), gray, cv::COLOR_BGR2GRAY);
        cv::Mat whitePixels;
        cv::threshold(gray, whitePixels, 175, 255, cv::THRESH_BINARY);
        return static_cast<double>(cv::countNonZero(whitePixels)) /
               static_cast<double>(whitePixels.rows * whitePixels.cols);
    }

    bool hasDoNotEnterWhiteBar(const cv::Mat &bgrImage, const cv::Rect &boundingBox)
    {
        cv::Rect clipped = boundingBox & cv::Rect(0, 0, bgrImage.cols, bgrImage.rows);
        if (clipped.width < 20 || clipped.height < 20)
            return false;

        cv::Mat gray;
        cv::cvtColor(bgrImage(clipped), gray, cv::COLOR_BGR2GRAY);
        cv::Mat whitePixels;
        cv::threshold(gray, whitePixels, 175, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(whitePixels, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            if (cv::contourArea(contour) < 40.0)
                continue;

            cv::Rect bar = cv::boundingRect(contour);
            double barAspect = static_cast<double>(bar.width) / std::max(1, bar.height);
            if (barAspect > 2.0 && bar.width > clipped.width * 0.35)
                return true;
        }
        return false;
    }

    std::vector<cv::Mat> buildDigitTemplates()
    {
        std::vector<cv::Mat> templates(10);
        for (int digit = 0; digit < 10; ++digit)
        {
            cv::Mat tmpl = cv::Mat::zeros(64, 48, CV_8UC1);
            std::string text(1, static_cast<char>('0' + digit));
            cv::putText(tmpl, text, cv::Point(6, 52), cv::FONT_HERSHEY_SIMPLEX, 1.6, cv::Scalar(255), 3);
            templates[digit] = tmpl;
        }
        return templates;
    }

    char matchDigitTemplate(const cv::Mat &binaryPatch, const std::vector<cv::Mat> &templates, double &bestScore)
    {
        bestScore = -1.0;
        if (binaryPatch.empty())
            return '?';

        cv::Mat padded;
        cv::copyMakeBorder(binaryPatch, padded, 6, 6, 6, 6, cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::Mat normalized;
        cv::resize(padded, normalized, cv::Size(48, 64));

        char bestDigit = '?';
        for (int digit = 0; digit < 10; ++digit)
        {
            cv::Mat result;
            cv::matchTemplate(normalized, templates[digit], result, cv::TM_CCOEFF_NORMED);
            double score = result.at<float>(0, 0);
            if (score > bestScore)
            {
                bestScore = score;
                bestDigit = static_cast<char>('0' + digit);
            }
        }

        return bestDigit;
    }

    int countDigitHoles(const cv::Mat &binaryPatch)
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binaryPatch, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
        if (hierarchy.empty())
            return 0;

        int holes = 0;
        for (int i = 0; i < static_cast<int>(contours.size()); ++i)
        {
            if (hierarchy[i][3] != -1)
                ++holes;
        }
        return holes;
    }

    char classifyDigitHeuristic(const cv::Mat &binaryPatch, int parentWidth)
    {
        if (binaryPatch.empty() || parentWidth <= 0)
            return '?';

        double aspect = static_cast<double>(binaryPatch.rows) / std::max(1, binaryPatch.cols);
        double widthRatio = static_cast<double>(binaryPatch.cols) / parentWidth;

        if (aspect > 2.0 || widthRatio < 0.20)
            return '1';
        if (countDigitHoles(binaryPatch) >= 1)
            return '0';
        if (widthRatio < 0.36)
            return '3';
        if (widthRatio < 0.55)
            return aspect < 1.1 ? '5' : '0';
        return '0';
    }

    cv::Rect expandSpeedLimitOcrBox(const cv::Rect &visualBox, const cv::Rect &seed, int imageCols)
    {
        const int minWidth = std::max(visualBox.width, seed.width);
        const int centerX = visualBox.x + visualBox.width / 2;
        int x = std::max(0, centerX - minWidth / 2);
        int width = std::min(minWidth, imageCols - x);
        return cv::Rect(x, visualBox.y, width, visualBox.height);
    }

    cv::Rect refineSpeedLimitBox(const cv::Mat &bgrImage, const cv::Rect &seed)
    {
        cv::Rect bounds(0, 0, bgrImage.cols, bgrImage.rows);
        cv::Rect padded = seed & bounds;
        if (padded.width < 10 || padded.height < 10)
            return seed;

        const int pad = 4;
        int x0 = std::max(0, padded.x - pad);
        int x1 = std::min(bgrImage.cols, padded.x + padded.width + pad);
        const int span0 = x1 - x0;

        cv::Mat gray;
        cv::cvtColor(bgrImage, gray, cv::COLOR_BGR2GRAY);

        const int centerX = (x0 + x1) / 2;
        const int narrowW = std::max(static_cast<int>(span0 * 0.55), 20);
        int centerLeft = std::max(0, centerX - narrowW / 2);
        int centerRight = std::min(bgrImage.cols, centerX + narrowW / 2);

        int top = padded.y;
        for (int y = padded.y; y < static_cast<int>(bgrImage.rows * 0.75); ++y)
        {
            cv::Mat row = gray(cv::Rect(centerLeft, y, centerRight - centerLeft, 1));
            cv::Mat whitePixels, blackPixels;
            cv::inRange(row, 185, 255, whitePixels);
            cv::inRange(row, 0, 120, blackPixels);
            const int span = centerRight - centerLeft;
            if (cv::countNonZero(whitePixels) > span * 0.55 &&
                cv::countNonZero(blackPixels) > span * 0.12)
            {
                top = y;
                break;
            }
        }

        int bottom = top;
        for (int y = top; y < static_cast<int>(bgrImage.rows * 0.75); ++y)
        {
            cv::Mat row = gray(cv::Rect(centerLeft, y, centerRight - centerLeft, 1));
            cv::Mat whitePixels, blackPixels;
            cv::inRange(row, 185, 255, whitePixels);
            cv::inRange(row, 0, 120, blackPixels);
            const int span = centerRight - centerLeft;
            if (cv::countNonZero(whitePixels) > span * 0.45 &&
                cv::countNonZero(blackPixels) > span * 0.08)
                bottom = y;
        }

        int height = std::max(bottom - top + 1, 20);
        int lowerStart = top + height / 3;
        cv::Mat lowerPanel = gray(cv::Rect(x0, lowerStart, span0, top + height - lowerStart));

        int left = 0;
        int right = lowerPanel.cols - 1;
        for (int x = 0; x < lowerPanel.cols; ++x)
        {
            cv::Mat col = lowerPanel(cv::Rect(x, 0, 1, lowerPanel.rows));
            cv::Mat whitePixels;
            cv::inRange(col, 190, 255, whitePixels);
            if (cv::countNonZero(whitePixels) > lowerPanel.rows * 0.50)
            {
                left = x;
                break;
            }
        }
        for (int x = lowerPanel.cols - 1; x >= 0; --x)
        {
            cv::Mat col = lowerPanel(cv::Rect(x, 0, 1, lowerPanel.rows));
            cv::Mat whitePixels;
            cv::inRange(col, 190, 255, whitePixels);
            if (cv::countNonZero(whitePixels) > lowerPanel.rows * 0.50)
            {
                right = x;
                break;
            }
        }

        int width = std::max(right - left + 1, 24);
        return cv::Rect(x0 + left, top, width, height);
    }

    bool isRedSymbolOnWhitePanel(const cv::Mat &whiteMask, const cv::Rect &redBox, double redArea)
    {
        if (redArea >= 10000.0)
            return false;

        cv::Point center(redBox.x + redBox.width / 2, redBox.y + redBox.height / 2);
        if (center.x < 0 || center.y < 0 || center.x >= whiteMask.cols || center.y >= whiteMask.rows)
            return false;
        if (whiteMask.at<uchar>(center) == 0)
            return false;

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(whiteMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour : contours)
        {
            double whiteArea = cv::contourArea(contour);
            if (whiteArea < 15000.0 || whiteArea < redArea * 1.8)
                continue;

            if (cv::pointPolygonTest(contour, center, false) >= 0)
                return true;
        }
        return false;
    }

    std::string readSpeedLimitDigits(const cv::Mat &bgrImage, const cv::Rect &visualBox,
                                     const cv::Rect &seedBox)
    {
        cv::Rect ocrBox = expandSpeedLimitOcrBox(visualBox, seedBox, bgrImage.cols);
        cv::Rect clipped = ocrBox & cv::Rect(0, 0, bgrImage.cols, bgrImage.rows);
        if (clipped.width < 20 || clipped.height < 30)
            return "";

        static const std::vector<cv::Mat> digitTemplates = buildDigitTemplates();

        cv::Mat gray;
        cv::cvtColor(bgrImage(clipped), gray, cv::COLOR_BGR2GRAY);
        int numberStart = static_cast<int>(clipped.height * 0.52);
        cv::Mat numberRegion = gray(cv::Rect(0, numberStart, clipped.width, clipped.height - numberStart));

        cv::Mat binary;
        cv::threshold(numberRegion, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, element);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        struct DigitCandidate
        {
            double area;
            int x;
            char digit;
        };

        std::vector<DigitCandidate> digits;
        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < 80.0)
                continue;

            cv::Rect digitBox = cv::boundingRect(contour);
            if (digitBox.height < numberRegion.rows * 0.22)
                continue;
            if (digitBox.width < clipped.width * 0.14 || digitBox.width > clipped.width * 0.75)
                continue;

            cv::Mat patch = binary(digitBox);
            double templateScore = -1.0;
            char digit = matchDigitTemplate(patch, digitTemplates, templateScore);
            if (templateScore < 0.55)
                digit = classifyDigitHeuristic(patch, clipped.width);

            digits.push_back({area, digitBox.x, digit});
        }

        if (digits.empty())
            return "";

        std::sort(digits.begin(), digits.end(),
                  [](const DigitCandidate &a, const DigitCandidate &b)
                  { return a.area > b.area; });
        if (digits.size() > 2)
            digits.resize(2);

        std::sort(digits.begin(), digits.end(),
                  [](const DigitCandidate &a, const DigitCandidate &b)
                  { return a.x < b.x; });

        std::string value;
        for (const auto &digit : digits)
        {
            if (digit.digit != '?')
                value.push_back(digit.digit);
        }
        return value;
    }
}

// Purpose: Detect stop, do-not-enter, and speed-limit signs.
// Pre-condition: bgrImage, redMask, whiteMask same size; outputImage is BGR.
// Post-condition: Draws boxes and labels on outputImage for matches found.
void ShapeAnalyzer::detectRegulatorySigns(const cv::Mat &bgrImage, const cv::Mat &redMask,
                                          const cv::Mat &whiteMask, cv::Mat &outputImage) const
{
    std::vector<cv::Rect> claimedBoxes;
    const double maxSignArea = outputImage.rows * outputImage.cols * 0.95;

    // Speed limits first (white panel + two digits).
    {
        struct SpeedLimitCandidate
        {
            cv::Rect box;
            double area;
            std::string digits;
        };

        std::vector<SpeedLimitCandidate> speedCandidates;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(whiteMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < 1200.0 || area > maxSignArea)
                continue;

            cv::Rect roughBox = cv::boundingRect(contour);
            if (roughBox.y > outputImage.rows * 0.90)
                continue;
            if (roughBox.width > outputImage.cols * 0.35)
                continue;

            double roughAspect = static_cast<double>(roughBox.width) / roughBox.height;
            if (roughAspect > 1.35)
                continue;

            cv::Rect boundingBox = refineSpeedLimitBox(bgrImage, roughBox);
            if (boundingBox.y <= 3 && roughAspect > 0.95)
                continue;
            if (boundingBox.height < 35)
                continue;

            double aspectRatio = static_cast<double>(boundingBox.width) / boundingBox.height;
            if (aspectRatio > 1.35)
                continue;

            std::string digits = readSpeedLimitDigits(bgrImage, boundingBox, roughBox);
            if (digits.size() < 2)
                continue;

            speedCandidates.push_back({boundingBox, area, digits});
        }

        if (!speedCandidates.empty())
        {
            const auto best = std::max_element(speedCandidates.begin(), speedCandidates.end(),
                                               [](const SpeedLimitCandidate &a,
                                                  const SpeedLimitCandidate &b)
                                               {
                                                   return a.area < b.area;
                                               });
            std::string label = "SPEED LIMIT " + best->digits;
            drawRegulatoryLabel(outputImage, best->box, label, cv::Scalar(255, 255, 255));
            claimedBoxes.push_back(best->box);
        }
    }

    // Red signs: octagon = stop, white bar inside = do not enter.
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < 4000.0 || area > maxSignArea)
                continue;

            cv::Rect boundingBox = cv::boundingRect(contour);
            if (boundingBox.y > outputImage.rows * 0.90)
                continue;
            if (overlapsAny(boundingBox, claimedBoxes))
                continue;

            double aspectRatio = static_cast<double>(boundingBox.width) / boundingBox.height;
            if (aspectRatio < 0.70 || aspectRatio > 1.30)
                continue;

            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);
            double hullPerimeter = cv::arcLength(hull, true);
            std::vector<cv::Point> approxPolygon;
            cv::approxPolyDP(hull, approxPolygon, 0.04 * hullPerimeter, true);
            int vertices = static_cast<int>(approxPolygon.size());
            if (vertices < 7 || vertices > 9)
                continue;

            double whiteRatio = innerWhiteRatio(bgrImage, boundingBox);
            if (whiteRatio > 0.35)
                continue;
            if (isRedSymbolOnWhitePanel(whiteMask, boundingBox, area))
                continue;

            const cv::Scalar redColor(0, 0, 255);
            if (hasDoNotEnterWhiteBar(bgrImage, boundingBox))
            {
                drawRegulatoryLabel(outputImage, boundingBox, "DO NOT ENTER", redColor);
                claimedBoxes.push_back(boundingBox);
            }
            else if (vertices == 8)
            {
                drawRegulatoryLabel(outputImage, boundingBox, "STOP SIGN", redColor);
                claimedBoxes.push_back(boundingBox);
            }
        }
    }
}

// Purpose: Detect yellow diamond warning signs.
// Pre-condition: binaryMask is 8-bit single-channel; outputImage is BGR.
// Post-condition: Draws yellow boxes and "WARNING SIGN" on outputImage.
void ShapeAnalyzer::detectWarningSign(const cv::Mat &binaryMask, cv::Mat &outputImage) const
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxSignArea = outputImage.rows * outputImage.cols * 0.99;

    double largestArea = 0.0;
    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > largestArea && area < maxSignArea)
            largestArea = area;
    }

    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < 500.0 || area > maxSignArea || area < (largestArea * 0.30))
            continue;

        cv::Rect boundingBox = cv::boundingRect(contour);
        if (boundingBox.y > outputImage.rows * 0.90)
            continue;

        double aspectRatio = (double)boundingBox.width / boundingBox.height;
        if (aspectRatio < 0.65 || aspectRatio > 1.35)
            continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hullPerimeter = cv::arcLength(hull, true);
        double epsilon = 0.04 * hullPerimeter;
        std::vector<cv::Point> approxPolygon;
        cv::approxPolyDP(hull, approxPolygon, epsilon, true);

        int vertices = (int)approxPolygon.size();

        // Diamond = warning. Sometimes get 5 verts if a small plate merges in.
        if (vertices == 4 || vertices == 5)
        {
            cv::rectangle(outputImage, boundingBox.tl(), boundingBox.br(), cv::Scalar(0, 255, 255), 3);
            cv::putText(outputImage, "WARNING SIGN",
                        cv::Point(boundingBox.x, labelY(boundingBox)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        }
    }
}

// Purpose: Detect orange diamond construction signs.
// Pre-condition: binaryMask is 8-bit single-channel; outputImage is BGR.
// Post-condition: Draws orange boxes and "CONSTRUCTION SIGN" on outputImage.
void ShapeAnalyzer::detectConstructionSign(const cv::Mat &binaryMask, cv::Mat &outputImage) const
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxSignArea = outputImage.rows * outputImage.cols * 0.99;

    double largestArea = 0.0;
    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > largestArea && area < maxSignArea)
            largestArea = area;
    }

    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < 500.0 || area > maxSignArea || area < (largestArea * 0.30))
            continue;

        cv::Rect boundingBox = cv::boundingRect(contour);
        if (boundingBox.y > outputImage.rows * 0.90)
            continue;

        double aspectRatio = (double)boundingBox.width / boundingBox.height;
        if (aspectRatio < 0.75 || aspectRatio > 1.25)
            continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hullPerimeter = cv::arcLength(hull, true);
        double epsilon = 0.04 * hullPerimeter;
        std::vector<cv::Point> approxPolygon;
        cv::approxPolyDP(hull, approxPolygon, epsilon, true);

        int vertices = (int)approxPolygon.size();

        if (vertices == 4)
        {
            cv::rectangle(outputImage, boundingBox.tl(), boundingBox.br(), cv::Scalar(0, 140, 255), 3);
            cv::putText(outputImage, "CONSTRUCTION SIGN",
                        cv::Point(boundingBox.x, labelY(boundingBox)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 140, 255), 2);
        }
    }
}

// Purpose: Detect green rectangular guide signs.
// Pre-condition: binaryMask is 8-bit single-channel; outputImage is BGR.
// Post-condition: Draws green boxes and "GUIDE SIGN" on outputImage.
void ShapeAnalyzer::detectGuideSign(const cv::Mat &binaryMask, cv::Mat &outputImage) const
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxSignArea = outputImage.rows * outputImage.cols * 0.95;
    std::vector<std::pair<double, cv::Rect>> candidates;

    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < 1000.0 || area > maxSignArea)
            continue;

        cv::Rect boundingBox = cv::boundingRect(contour);
        if (boundingBox.y > outputImage.rows * 0.85)
            continue;

        double aspectRatio = (double)boundingBox.width / boundingBox.height;
        if (aspectRatio < 0.2 || aspectRatio > 5.0)
            continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hullPerimeter = cv::arcLength(hull, true);
        double epsilon = 0.04 * hullPerimeter;
        std::vector<cv::Point> approxPolygon;
        cv::approxPolyDP(hull, approxPolygon, epsilon, true);

        int vertices = (int)approxPolygon.size();

        if (vertices >= 4 && vertices <= 6)
            candidates.emplace_back(area, boundingBox);
    }

    for (size_t i = 0; i < candidates.size(); ++i)
    {
        const cv::Rect &box = candidates[i].second;
        bool nestedInLarger = false;

        for (size_t j = 0; j < candidates.size(); ++j)
        {
            if (i == j)
                continue;

            if (candidates[j].first > candidates[i].first &&
                isNestedInside(box, candidates[j].second))
            {
                nestedInLarger = true;
                break;
            }
        }

        if (nestedInLarger)
            continue;

        cv::rectangle(outputImage, box.tl(), box.br(), cv::Scalar(0, 200, 0), 3);
        cv::putText(outputImage, "GUIDE SIGN",
                    cv::Point(box.x, labelY(box)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 200, 0), 2);
    }
}

// Purpose: Detect blue rectangular service signs.
// Pre-condition: binaryMask is 8-bit single-channel; outputImage is BGR.
// Post-condition: Draws blue boxes and "SERVICE SIGN" on outputImage.
void ShapeAnalyzer::detectServiceSign(const cv::Mat &binaryMask, cv::Mat &outputImage) const
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxSignArea = outputImage.rows * outputImage.cols * 0.95;
    std::vector<std::pair<double, cv::Rect>> candidates;

    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < 1000.0 || area > maxSignArea)
            continue;

        cv::Rect boundingBox = cv::boundingRect(contour);
        if (boundingBox.y > outputImage.rows * 0.85)
            continue;

        double aspectRatio = (double)boundingBox.width / boundingBox.height;
        if (aspectRatio < 0.2 || aspectRatio > 5.0)
            continue;

        double fillRatio = area / (boundingBox.width * boundingBox.height);
        if (fillRatio < 0.55)
            continue;

        if (isLikelySkyBlob(boundingBox, fillRatio, outputImage.cols))
            continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hullPerimeter = cv::arcLength(hull, true);
        double epsilon = 0.04 * hullPerimeter;
        std::vector<cv::Point> approxPolygon;
        cv::approxPolyDP(hull, approxPolygon, epsilon, true);

        int vertices = (int)approxPolygon.size();

        if (vertices >= 4 && vertices <= 6)
            candidates.emplace_back(area, boundingBox);
    }

    for (size_t i = 0; i < candidates.size(); ++i)
    {
        const cv::Rect &box = candidates[i].second;
        bool nestedInLarger = false;

        for (size_t j = 0; j < candidates.size(); ++j)
        {
            if (i == j)
                continue;

            if (candidates[j].first > candidates[i].first &&
                isNestedInside(box, candidates[j].second))
            {
                nestedInLarger = true;
                break;
            }
        }

        if (nestedInLarger)
            continue;

        cv::rectangle(outputImage, box.tl(), box.br(), cv::Scalar(255, 100, 0), 3);
        cv::putText(outputImage, "SERVICE SIGN",
                    cv::Point(box.x, labelY(box)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 100, 0), 2);
    }
}
