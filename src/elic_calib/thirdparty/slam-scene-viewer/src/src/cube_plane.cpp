//
// Created by csl on 10/22/22.
//

#include "slam-scene-viewer/cube_plane.h"

namespace ns_viewer {
    ColourWheel CubePlane::COLOUR_WHEEL = ColourWheel(1.0f);
    std::default_random_engine CubePlane::RANDOM_ENGINE = {};

    CubePlane::CubePlane(float roll, float pitch, float yaw, float dx, float dy, float dz, float width,
                         float height, float thickness, const ns_viewer::Colour &color)
            : xSpan(width), ySpan(thickness), zSpan(height), color(color) {

        // compute ths plane pose
        auto y = Eigen::AngleAxisf(DEG_TO_RAD * yaw, Vector3f(0.0, 0.0, 1.0));
        auto p = Eigen::AngleAxisf(DEG_TO_RAD * pitch, Vector3f(1.0, 0.0, 0.0));
        auto r = Eigen::AngleAxisf(DEG_TO_RAD * roll, Vector3f(0.0, 1.0, 0.0));
        auto rotWtoL = r * p * y;
        LtoW = Posef(rotWtoL.matrix().inverse(), Vector3f(dx, dy, dz));
        WtoL = LtoW.inverse();
    }

    pcl::PointCloud<pcl::PointXY>::Ptr
    CubePlane::GenerateFeatures(int size, float xMin, float xMax, float yMin, float yMax) {
        // generate random feature
        RANDOM_ENGINE.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> ux(xMin, xMax);
        std::uniform_real_distribution<float> uy(yMin, yMax);

        pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
        cloud->resize(size);

        for (int i = 0; i < size; ++i) {
            auto &pt = cloud->at(i);
            pt.x = ux(RANDOM_ENGINE);
            pt.y = uy(RANDOM_ENGINE);
        }
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    CubePlane::GenerateFeatures(int size, int faceOption, float opacity) const {
        // feature cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr feature = GenerateFeatures(size, Colour::Black(), faceOption);
        for (auto &p: feature->points) {
            const auto &uniqueColour = COLOUR_WHEEL.GetUniqueColour();
            p.r = static_cast<std::uint8_t>(uniqueColour.r * 255.0f);
            p.g = static_cast<std::uint8_t>(uniqueColour.g * 255.0f);
            p.b = static_cast<std::uint8_t>(uniqueColour.b * 255.0f);
            p.a = static_cast<std::uint8_t>(opacity * 255.0f);
        }
        return feature;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    CubePlane::GenerateFeatures(int size, const Colour &colour, int faceOption) const {
        // feature cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr feature(new pcl::PointCloud<pcl::PointXYZRGBA>);

        auto trans = [this](pcl::PointXYZRGBA &p) {
            Eigen::Vector3f pt(p.x, p.y, p.z);
            Eigen::Vector3f result = LtoW.trans(pt);
            p.x = result(0), p.y = result(1), p.z = result(2);
        };

        if (IsFaceWith(FRONT, faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * xSpan, 0.5f * xSpan, -0.5f * zSpan, 0.5f * zSpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = pt2d.x;
                pt3d.y = 0.5f * ySpan;
                pt3d.z = pt2d.y;
                pt3d.r = static_cast<std::uint8_t>(colour.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(colour.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(colour.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(colour.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWith(BACK, faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * xSpan, 0.5f * xSpan, -0.5f * zSpan, 0.5f * zSpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = pt2d.x;
                pt3d.y = -0.5f * ySpan;
                pt3d.z = pt2d.y;
                pt3d.r = static_cast<std::uint8_t>(colour.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(colour.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(colour.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(colour.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWith(LEFT, faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * ySpan, 0.5f * ySpan, -0.5f * zSpan, 0.5f * zSpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = -0.5f * xSpan;
                pt3d.y = pt2d.x;
                pt3d.z = pt2d.y;
                pt3d.r = static_cast<std::uint8_t>(colour.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(colour.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(colour.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(colour.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWith(RIGHT, faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * ySpan, 0.5f * ySpan, -0.5f * zSpan, 0.5f * zSpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = 0.5f * xSpan;
                pt3d.y = pt2d.x;
                pt3d.z = pt2d.y;
                pt3d.r = static_cast<std::uint8_t>(colour.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(colour.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(colour.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(colour.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWith(TOP, faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * xSpan, 0.5f * xSpan, -0.5f * ySpan, 0.5f * ySpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = pt2d.x;
                pt3d.y = pt2d.y;
                pt3d.z = 0.5f * zSpan;
                pt3d.r = static_cast<std::uint8_t>(colour.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(colour.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(colour.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(colour.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        if (IsFaceWith(BOTTOM, faceOption)) {
            auto cloud2d = GenerateFeatures(size, -0.5f * xSpan, 0.5f * xSpan, -0.5f * ySpan, 0.5f * ySpan);
            for (int i = 0; i < size; ++i) {
                const auto &pt2d = cloud2d->at(i);
                pcl::PointXYZRGBA pt3d;
                pt3d.x = pt2d.x;
                pt3d.y = pt2d.y;
                pt3d.z = -0.5f * zSpan;
                pt3d.r = static_cast<std::uint8_t>(colour.r * 255.0f);
                pt3d.g = static_cast<std::uint8_t>(colour.g * 255.0f);
                pt3d.b = static_cast<std::uint8_t>(colour.b * 255.0f);
                pt3d.a = static_cast<std::uint8_t>(colour.a * 255.0f);
                trans(pt3d);
                feature->push_back(pt3d);
            }
        }

        return feature;
    }

    CubePlane::CubePlane(float roll, float pitch, float yaw,
                         float dx, float dy, float dz,
                         float width, const Colour &color)
            : CubePlane(roll, pitch, yaw, dx, dy, dz, width, 2.0f, 0.1f, color) {}
}
