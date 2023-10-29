//
// Created by csl on 10/7/22.
//

#ifndef SLAM_SCENE_VIEWER_COLOUR_H
#define SLAM_SCENE_VIEWER_COLOUR_H

#include "ostream"
#include "numeric"
#include "stdexcept"
#include "cmath"

namespace ns_viewer {

    struct Colour {
        inline static Colour White() {
            return Colour(1.0f, 1.0f, 1.0f, 1.0f);
        }

        inline static Colour Black() {
            return Colour(0.0f, 0.0f, 0.0f, 1.0f);
        }

        inline static Colour Red() {
            return Colour(1.0f, 0.0f, 0.0f, 1.0f);
        }

        inline static Colour Green() {
            return Colour(0.0f, 1.0f, 0.0f, 1.0f);
        }

        inline static Colour Blue() {
            return Colour(0.0f, 0.0f, 1.0f, 1.0f);
        }

        inline static Colour Unspecified() {
            return Colour(
                    std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()
            );
        }

        /// Default constructs white.
        inline Colour()
                : red(1.0f), green(1.0f), blue(1.0f), alpha(1.0f) {
        }

        /// Construct from component values
        inline Colour(const float red, const float green, const float blue, const float alpha = 1.0f)
                : red(red), green(green), blue(blue), alpha(alpha) {
        }

        /// Construct from rgba array.
        inline Colour(const float rgba[4]) {
            r = rgba[0];
            g = rgba[1];
            b = rgba[2];
            a = rgba[3];
        }

        /// Return pointer to OpenGL compatible RGBA array.
        inline float *Get() {
            return c;
        }

        /// Return this colour with alpha adjusted.
        inline Colour WithAlpha(const float alpha) {
            return Colour(r, g, b, alpha);
        }

        /// Construct from HSV Colour
        /// @param hue Colour hue in range [0,1]
        /// @param sat Saturation in range [0,1]
        /// @param val Value / Brightness in range [0,1].
        static inline Colour
        Hsv(const float hue, const float sat = 1.0f, const float val = 1.0f, const float alpha = 1.0f) {
            const float h = 6.0f * hue;
            const int i = (int) floor(h);
            const float f = (i % 2 == 0) ? 1 - (h - i) : h - i;
            const float m = val * (1 - sat);
            const float n = val * (1 - sat * f);

            switch (i) {
                case 0:
                    return Colour(val, n, m, alpha);
                case 1:
                    return Colour(n, val, m, alpha);
                case 2:
                    return Colour(m, val, n, alpha);
                case 3:
                    return Colour(m, n, val, alpha);
                case 4:
                    return Colour(n, m, val, alpha);
                case 5:
                    return Colour(val, m, n, alpha);
                default:
                    throw std::runtime_error("Found extra colour in rainbow.");
            }
        }

        union {
            struct {
                float red;
                float green;
                float blue;
                float alpha;
            };
            struct {
                float r;
                float g;
                float b;
                float a;
            };
            float c[4];
        };


        friend std::ostream &operator<<(std::ostream &os, const Colour &colour) {
            os << "r: " << colour.r << " g: " << colour.g << " b: " << colour.b << " a: " << colour.a;
            return os;
        }
    };

    class ColourWheel {
    public:
        /// Construct ColourWheel with Saturation, Value and Alpha constant.
        inline ColourWheel(float saturation = 0.5f, float value = 1.0f, float alpha = 1.0f)
                : unique_colours(0), sat(saturation), val(value), alpha(alpha) {

        }

        /// Use Golden ratio (/angle) to pick well spaced colours.
        inline Colour GetColourBin(int i) const {
            float hue = i * 0.5f * (3.0f - sqrt(5.0f));
            hue -= (int) hue;
            return Colour::Hsv(hue, sat, val, alpha);
        }

        /// Return next unique colour from ColourWheel.
        inline Colour GetUniqueColour() {
            return GetColourBin(unique_colours++);
        }

        /// Reset colour wheel counter to initial state
        inline void Reset() {
            unique_colours = 0;
        }

        friend std::ostream &operator<<(std::ostream &os, const ColourWheel &wheel) {
            os << "unique_colours: " << wheel.unique_colours << " sat: " << wheel.sat << " val: " << wheel.val
               << " alpha: " << wheel.alpha;
            return os;
        }

    protected:
        int unique_colours;
        float sat;
        float val;
        float alpha;
    };

}


#endif //SLAM_SCENE_VIEWER_COLOUR_H