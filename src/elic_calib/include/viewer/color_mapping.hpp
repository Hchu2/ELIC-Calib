#ifndef COLOR_MAPPING_H
#define COLOR_MAPPING_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cmath>
#include <iostream>

namespace ns_elic {
    using uchar = unsigned char;

    struct Crgb {
    public:
        uchar r; // [0, 255]
        uchar g; // [0, 255]
        uchar b; // [0, 255]

    public:
        Crgb(uchar r, uchar g, uchar b) : r(r), g(g), b(b) {}
    };

    /**
     * @brief override operator '<<' for type 'Crgb'
     */
    static std::ostream &operator<<(std::ostream &os, const Crgb &obj) {
        os << '{';
        os << "'r': " << (int) obj.r << ", 'g': " << (int) obj.g << ", 'b': " << (int) obj.b;
        os << '}';
        return os;
    }

    struct Chsv {
    public:
        float h; // [0.0f, 360.0f]
        float s; // [0.0f, 1.0f]
        float v; // [0.0f, 1.0f]

    public:
        Chsv(float h, float s, float v) : h(h), s(s), v(v) {}
    };

    /**
     * @brief override operator '<<' for type 'Chsv'
     */
    static std::ostream &operator<<(std::ostream &os, const Chsv &obj) {
        os << '{';
        os << "'h': " << obj.h << ", 's': " << obj.s << ", 'v': " << obj.v;
        os << '}';
        return os;
    }

    // translate hsv color to rgb color
    static Crgb hsv2rgb(const Chsv &hsv) {
        float H = hsv.h, S = hsv.s, V = hsv.v;
        float flag = H / 60.0f;
        int temp = int(flag / 2.0f);

        float C = V * S;
        float X = C * (1.0f - std::abs(flag - 2.0f * (float) temp - 1));
        float m = V - C;

        float r, g, b;
        switch (int(flag)) {
            case 0:
                r = C, g = X, b = 0.0f;
                break;
            case 1:
                r = X, g = C, b = 0.0f;
                break;
            case 2:
                r = 0.0f, g = C, b = X;
                break;
            case 3:
                r = 0.0f, g = X, b = C;
                break;
            case 4:
                r = X, g = 0.0f, b = C;
                break;
                // case [5] and [6] are same
            case 5:
            case 6:
                r = C, g = 0.0f, b = X;
                break;
            default:
                r = g = b = 0.0f;
                break;
        }

        return Crgb((r + m) * 255.0f, (g + m) * 255.0f, (b + m) * 255.0f);
    }

    struct HSVMapping {
    protected:
        mutable float mapValue{};

        union {
            // mapValue to hue
            struct {
                float startHue;
                float endHue;
                float sat;
                float val;
            } hueMapping{};
            // mapValue to sat
            struct {
                float hue;
                float startSat;
                float endSat;
                float val;
            } satMapping;
            // mapValue to val
            struct {
                float hue;
                float sat;
                float startVal;
                float endVal;
            } valMapping;
        };

        HSVMapping() = default;

    public:
        const HSVMapping &setMap(float mapValue) const {
            this->mapValue = mapValue;
            return *this;
        }

        virtual float getStdStart() const = 0;

        virtual float getStdEnd() const = 0;

        virtual float getByteStart() const = 0;

        virtual float getByteEnd() const = 0;

        virtual Chsv construct() const = 0;

        virtual cv::Mat construct(const cv::Mat &mapMat) const = 0;
    };

    // mapValue value to hue dime
    struct HueMapping : public HSVMapping {
        HueMapping(float startHue, float endHue, float sat, float val) noexcept: HSVMapping() {
            this->hueMapping.startHue = startHue;
            this->hueMapping.endHue = endHue;
            this->hueMapping.sat = sat;
            this->hueMapping.val = val;
        }

        float getByteStart() const override {
            return this->hueMapping.startHue / 360.0 * 255.0;
        }

        float getByteEnd() const override {
            return this->hueMapping.endHue / 360.0 * 255.0;
        }

        float getStdStart() const override {
            return this->hueMapping.startHue;
        }

        float getStdEnd() const override {
            return this->hueMapping.endHue;
        }

        Chsv construct() const override {
            return {this->mapValue, this->hueMapping.sat, this->hueMapping.val};
        }

        cv::Mat construct(const cv::Mat &mapMat) const override {
            const cv::Mat &hMat = mapMat;
            cv::Mat sMat(hMat.size(), CV_8UC1, cv::Scalar(this->hueMapping.sat * 255.0));
            cv::Mat vMat(hMat.size(), CV_8UC1, cv::Scalar(this->hueMapping.val * 255.0));
            cv::Mat dst;
            cv::merge(std::vector<cv::Mat>{hMat, sMat, vMat}, dst);
            return dst;
        }
    };

    // mapValue value to sat dime
    struct SatMapping : public HSVMapping {
        SatMapping(float startSat, float endSat, float hue, float val) noexcept: HSVMapping() {
            this->satMapping.hue = hue;
            this->satMapping.startSat = startSat;
            this->satMapping.endSat = endSat;
            this->satMapping.val = val;
        }

        float getByteStart() const override {
            return this->satMapping.startSat * 255.0;
        }

        float getByteEnd() const override {
            return this->satMapping.endSat * 255.0;
        }

        float getStdStart() const override {
            return this->satMapping.startSat;
        }

        float getStdEnd() const override {
            return this->satMapping.endSat;
        }

        Chsv construct() const override {
            return {this->satMapping.hue, this->mapValue, this->satMapping.val};
        }

        cv::Mat construct(const cv::Mat &mapMat) const override {
            cv::Mat hMat(mapMat.size(), CV_8UC1, cv::Scalar(this->satMapping.hue / 360.0 * 255.0));
            const cv::Mat &sMat = mapMat;
            cv::Mat vMat(mapMat.size(), CV_8UC1, cv::Scalar(this->satMapping.val * 255.0));
            cv::Mat dst;
            cv::merge(std::vector<cv::Mat>{hMat, sMat, vMat}, dst);
            return dst;
        }
    };

    // mapValue value to val dime
    struct ValMapping : public HSVMapping {
        ValMapping(float startVal, float endVal, float hue, float sat) noexcept: HSVMapping() {
            this->valMapping.hue = hue;
            this->valMapping.sat = sat;
            this->valMapping.startVal = startVal;
            this->valMapping.endVal = endVal;
        }

        float getByteStart() const override {
            return this->valMapping.startVal * 255.0;
        }

        float getByteEnd() const override {
            return this->valMapping.endVal * 255.0;
        }

        float getStdStart() const override {
            return this->valMapping.startVal;
        }

        float getStdEnd() const override {
            return this->valMapping.endVal;
        }

        Chsv construct() const override {
            return {this->valMapping.hue, this->valMapping.sat, this->mapValue};
        }

        cv::Mat construct(const cv::Mat &mapMat) const override {
            cv::Mat hMat(mapMat.size(), CV_8UC1, cv::Scalar(this->valMapping.hue / 360.0 * 255.0));
            cv::Mat sMat(mapMat.size(), CV_8UC1, cv::Scalar(this->valMapping.sat * 255.0));
            const cv::Mat &vMat = mapMat;
            cv::Mat dst;
            cv::merge(std::vector<cv::Mat>{hMat, sMat, vMat}, dst);
            return dst;
        }
    };

    namespace style {
        [[maybe_unused]] const static HueMapping red_yellow{0.0, 60.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping yellow_green{45.0, 130.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping green_cyan{100.0, 190.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping cyan_blue{180.0, 240.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping blue_purple{220.0, 300.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping purple_red{290.0, 360.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping panchromatic{0.0, 360.0, 1.0, 1.0};

        [[maybe_unused]] const static SatMapping red{0.0, 1.0, 360.0, 1.0};
        [[maybe_unused]] const static SatMapping pink{0.0, 1.0, 340.0, 1.0};
        [[maybe_unused]] const static SatMapping purple{0.0, 1.0, 310.0, 1.0};
        [[maybe_unused]] const static SatMapping blue{0.0, 1.0, 240.0, 1.0};
        [[maybe_unused]] const static SatMapping cyan{0.0, 1.0, 190.0, 1.0};
        [[maybe_unused]] const static SatMapping green{0.0, 1.0, 120.0, 1.0};
        [[maybe_unused]] const static SatMapping yellow{0.0, 1.0, 60.0, 1.0};
        [[maybe_unused]] const static SatMapping orange{0.0, 1.0, 20.0, 1.0};

        [[maybe_unused]] const static HueMapping red_yellow_green{0.0, 150.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping yellow_green_cyan{50.0, 180.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping green_cyan_blue{50.0, 250.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping cyan_blue_purple{180.0, 300.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping blue_purple_red{240.0, 360.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping cold{180.0, 360.0, 1.0, 1.0};
        [[maybe_unused]] const static HueMapping worm{0.0, 180.0, 1.0, 1.0};

        [[maybe_unused]] const static ValMapping gray{0.0, 1.0, 0.0, 0.0};
        [[maybe_unused]] const static ValMapping black_red{0.0, 1.0, 0.0, 1.0};
        [[maybe_unused]] const static ValMapping black_green{0.0, 1.0, 120.0, 1.0};
        [[maybe_unused]] const static ValMapping black_blue{0.0, 1.0, 240.0, 1.0};

    } // namespace ns_style

    // Continuous mapping
    static Crgb mapping(float value, float srcMin, float srcMax,
                        const HSVMapping &map = style::panchromatic, bool reversal = false) {
        // check range
        value < srcMin ? value = srcMin : float{};
        value > srcMax ? value = srcMax : float{};
        // linner mapping
        if (reversal) {
            value = srcMax - (value - srcMin);
        }
        float start = map.getStdStart(), end = map.getStdEnd();
        float mapVal = (value - srcMin) / (srcMax - srcMin) * (end - start) + start;

        // color mapping
        return hsv2rgb(map.setMap(mapVal).construct());
    }

    // Discrete mapping
    static Crgb mapping(float value, float srcMin, float srcMax, ushort classes,
                        const HSVMapping &map = style::panchromatic, bool reversal = false) {
        // check range
        value < srcMin ? value = srcMin : float{};
        value > srcMax ? value = srcMax : float{};
        // liner mapping
        if (reversal) {
            value = srcMax - (value - srcMin);
        }
        float start = map.getStdStart(), end = map.getStdEnd();
        float mapVal = (value - srcMin) / (srcMax - srcMin) * (end - start);
        if (classes == 0) {
            classes = 1;
        }
        float pieceSize = (end - start) / (float) classes;
        mapVal = int(mapVal / pieceSize) * pieceSize + 0.5f * pieceSize + start;
        if (mapVal > end) {
            mapVal = end - 0.5f * pieceSize;
        }
        // color mapping
        return hsv2rgb(map.setMap(mapVal).construct());
    }

    static cv::Mat mapping(const cv::Mat &srcImg, float srcMin, float srcMax,
                           const HSVMapping &map = style::panchromatic, bool reversal = false) {
        cv::Mat refineMat = cv::max(srcMin, cv::min(srcMax, srcImg));

        float startVal = map.getByteStart(), endVal = map.getByteEnd();

        if (reversal) {
            float temp = startVal;
            startVal = endVal;
            endVal = temp;
        }

        // [srcMin, srcMax] to [startVal, endVal]
        // (pixel - srcMin)/(srcMax - srcMin)*(endVal - startVal) + startVal

        cv::Mat gray;
        float factor = (endVal - startVal) / (srcMax - srcMin);
        refineMat.convertTo(gray, CV_8UC1, factor, -srcMin * factor + startVal);

        cv::Mat hsv = map.construct(gray), bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR_FULL);

        return bgr;
    }
} // namespace ns_elic

#endif
