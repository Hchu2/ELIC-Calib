//
// Created by csl on 12/20/22.
//

#ifndef LIC_CALIB_PLANE_OBSERVABILITY_HPP
#define LIC_CALIB_PLANE_OBSERVABILITY_HPP

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/se3.hpp"
#include "thirdparty/logger/src/include/logger.h"
#include "slam-scene-viewer/scene_viewer.h"
#include "util/utils.hpp"

namespace ns_elic {
    void LearnPlaneObservability() {

        // cf1 = ax + by + cz + d
        // cf2 = a^2 + b^2 + c^2 - 1
        /**
         * x  y  z  1
         * 2a 2b 2c 0
         */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pts(new pcl::PointCloud<pcl::PointXYZRGBA>);
        std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> u(1.0f, 2.0f);
        for (int i = 0; i < 100; ++i) {
            pcl::PointXYZRGBA p;
            p.x = u(engine), p.y = u(engine), p.z = 0.0f;
            // p.x = u(engine), p.y = 1.0f, p.z = 0.0f;
            // p.x = 1.0f, p.y = 1.0f, p.z = 0.0f;
            auto color = ns_viewer::SceneViewer::GetUniqueColour();
            p.r = color.r * 255.0f, p.g = color.g * 255.0f, p.b = color.b * 255.0f, p.a = color.a * 255.0f;
            pts->push_back(p);
        }
        Eigen::Vector4d x;
        x << 0.5, 0.5, 0.5, 0.5;
        for (int i = 0; i < 10; ++i) {
            Eigen::Matrix4d HMat = Eigen::Matrix4d::Zero();
            Eigen::Vector4d bVec = Eigen::Vector4d::Zero();
            const auto &a = x(0), b = x(1), c = x(2), d = x(3);
            for (const auto &p: pts->points) {
                Eigen::Vector2d r;
                r(0) = a * p.x + b * p.y + c * p.z + d;
                r(1) = a * a + b * b + c * c - 1;
                Eigen::Matrix<double, 2, 4> j;
                j(0, 0) = p.x, j(0, 1) = p.y, j(0, 2) = p.z, j(0, 3) = 1;
                j(1, 0) = 2 * a, j(1, 1) = 2 * b, j(1, 2) = 2 * c, j(1, 3) = 0;
                HMat += j.transpose() * j;
                bVec -= j.transpose() * r;
            }
            Eigen::Matrix<double, 4, 5> HBarMat;
            HBarMat.topLeftCorner<4, 4>() = HMat;
            HBarMat.topRightCorner<4, 1>() = bVec;
            LOG_VAR(HBarMat)
            LOG_VAR(ReducedRowEchelonForm(HBarMat))
            {
                Eigen::JacobiSVD<Eigen::Matrix4d> svd(HMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
                LOG_VAR(svd.rank())
                Eigen::FullPivLU<Eigen::Matrix4d> lu(HMat);
                LOG_VAR(lu.kernel().transpose())
            }
            Eigen::Vector4d update = HMat.ldlt().solve(bVec);
            LOG_VAR(update.transpose())
            LOG_ENDL()

            x += update;
        }
        LOG_VAR(x.transpose())

        ns_viewer::SceneViewer viewer("/home/csl/ros_ws/LIC-Calib/src/lic_calib/scene");
        viewer.AddFeatures(pts);
        viewer.RunSingleThread();

    }

}

#endif //LIC_CALIB_PLANE_OBSERVABILITY_HPP
