//
// Created by csl on 10/13/22.
//

#ifndef LIC_CALIB_TRANS_PLANE_HPP
#define LIC_CALIB_TRANS_PLANE_HPP

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/se3.hpp"
#include "thirdparty/logger/src/include/logger.h"

namespace ns_elic {
    Eigen::Vector4d ComputePlaneCoeff(const std::vector<Eigen::Vector3d> &points) {
        // ax + by + cz + d = 0
        Eigen::MatrixXd mat(points.size(), 4);
        for (int i = 0; i < points.size(); ++i) {
            const auto &p = points.at(i);
            double x = p(0), y = p(1), z = p(2);
            mat(i, 0) = x, mat(i, 1) = y, mat(i, 2) = z, mat(i, 3) = 1.0;
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector4d x = svd.matrixV().col(3);
        return x;
    }

    void TransPlaneTest() {
        std::vector<Eigen::Vector3d> points = {
                {0, 0, 0},
                {1, 0, 0},
                {1, 1, 1}
        };
        auto coeff = ComputePlaneCoeff(points);
        LOG_VAR(coeff.transpose())
        Eigen::AngleAxisd angleAxis(M_PI_2, Eigen::Vector3d(0.0, 0.0, 1.0));
        auto se3 = Sophus::SE3d(angleAxis.toRotationMatrix(), Eigen::Vector3d(1, 2, 3));
        for (const auto &item: points) {
            LOG_VAR(item.transpose());
            LOG_VAR((se3 * item).transpose());
        }
        Eigen::Vector4d newCoeff = se3.matrix().transpose().inverse() * coeff;
        LOG_VAR(newCoeff.transpose())
    }
}

#endif //LIC_CALIB_TRANS_PLANE_HPP
