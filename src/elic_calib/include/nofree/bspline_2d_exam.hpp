//
// Created by csl on 1/10/23.
//

#ifndef LIC_CALIB_BSPLINE_2D_EXAM_HPP
#define LIC_CALIB_BSPLINE_2D_EXAM_HPP

#include "Eigen/Dense"
#include "util/type_define.hpp"
#include "thirdparty/logger/src/include/logger.h"

namespace ns_elic {
    struct Bezier {
    public:
        static ns_elic::aligned_vector<Eigen::Vector2d>
        solve(const ns_elic::aligned_vector<Eigen::Vector2d> &controlPoints, std::size_t num) {
            double t = 0.0, delta = 1.0 / static_cast<double>(num - 1);
            ns_elic::aligned_vector<Eigen::Vector2d> result(num);

            for (int i = 0; i < num; ++i) {
                result[i] = bezier(t, controlPoints, 0, controlPoints.size());
                t += delta;
            }
            return result;
        }

    protected:
        static Eigen::Vector2d bezier(double t, const ns_elic::aligned_vector<Eigen::Vector2d> &controlPoints,
                                      std::size_t beg, std::size_t end) {
            if (end - beg == 1) {
                return controlPoints[beg];
            } else {
                auto p1 = bezier(t, controlPoints, beg, end - 1);
                p1(0) *= 1 - t;
                p1(1) *= 1 - t;
                auto p2 = bezier(t, controlPoints, beg + 1, end);
                p2(0) *= t;
                p2(1) *= t;
                return {p1(0) + p2(0), p1(1) + p2(1)};
            }
        }
    };

    struct UniformBSpline {
        static ns_elic::aligned_vector<Eigen::Vector2d>
        solve(ns_elic::aligned_vector<Eigen::Vector2d> controlPoints, std::size_t k, std::size_t num) {
            std::size_t n = controlPoints.size() - 1;
            std::size_t m = n + k + 1;
            double t = 0.0;

            std::vector<double> tAry(m + 1);
            for (int i = 0; i < m + 1; ++i) {
                if (i >= k && i <= m - 1 - k) {
                    tAry[i] = t;
                    t += 1.0;
                } else {
                    tAry[i] = t;
                }
            }
            LOG_VAR(tAry)

            ns_elic::aligned_vector<Eigen::Vector2d> result(num);
            double delta = static_cast<double>(tAry[m - k + 1] - tAry[k - 1]) / static_cast<double>(num - 1);
            t = tAry[k - 1];

            for (int j = 0; j < num; ++j) {
                result[j](0) = 0.0f, result[j](1) = 0.0f;
                for (int i = 0; i <= n; ++i) {
                    double val = deBoor(i, k, t, tAry);
                    result[j](0) += controlPoints[i](0) * val;
                    result[j](1) += controlPoints[i](1) * val;
                }
                t += delta;
            }

            return result;
        }

    protected:
        static double deBoor(std::size_t i, std::size_t k, double t, const std::vector<double> &tAry) {
            if (k == 0) {
                if (t >= tAry[i] && t <= tAry[i + 1]) {
                    return 1.0;
                } else {
                    return 0.0;
                }
            }
            double v1 = 0.0, v2 = 0.0;
            if (tAry[i + k] != tAry[i]) {
                v1 = (t - tAry[i]) / (tAry[i + k] - tAry[i]) * deBoor(i, k - 1, t, tAry);
            }
            if (tAry[i + k + 1] != tAry[i + 1]) {
                v2 = (tAry[i + k + 1] - t) / (tAry[i + k + 1] - tAry[i + 1]) * deBoor(i + 1, k - 1, t, tAry);
            }
            return v1 + v2;
        }
    };

    void GenBsplineExample(const std::string &dir) {
        const ns_elic::aligned_vector<Eigen::Vector2d> controlsPoints{
                {0, 1.5},
                {1, 0},
                {2, 2},
                {3, 0},
                {4, 1.5}
        };
        auto writeToFile = [](const ns_elic::aligned_vector<Eigen::Vector2d> &ctrlPts,
                              const ns_elic::aligned_vector<Eigen::Vector2d> &trajPts,
                              const std::string &filename) {
            std::ofstream file(filename, std::ios::out);
            file << "# controls points" << std::endl;
            for (const auto &pt: ctrlPts) {
                file << pt(0) << ',' << pt(1) << std::endl;
            }
            file << "# trajectory points" << std::endl;
            for (const auto &pt: trajPts) {
                file << pt(0) << ',' << pt(1) << std::endl;
            }
            file.close();
        };
        {
            // bezier
            auto result = Bezier::solve(controlsPoints, 100);
            writeToFile(controlsPoints, result, dir + "/bezier.txt");
        }

        // bspline
        for (int i = 0; i < 3; ++i) {
            const std::size_t degree = i + 1;
            auto result = UniformBSpline::solve(controlsPoints, degree, 100);
            writeToFile(controlsPoints, result, dir + "/bspline_" + std::to_string(degree) + ".txt");
        }
    }

}

#endif //LIC_CALIB_BSPLINE_2D_EXAM_HPP
