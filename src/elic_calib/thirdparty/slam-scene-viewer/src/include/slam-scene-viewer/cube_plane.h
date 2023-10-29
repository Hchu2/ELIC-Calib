//
// Created by csl on 10/22/22.
//

#ifndef SLAM_SCENE_VIEWER_CUBE_PLANE_H
#define SLAM_SCENE_VIEWER_CUBE_PLANE_H

#include "slam-scene-viewer/colour.hpp"
#include "slam-scene-viewer/pose.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "random"
#include "chrono"

namespace ns_viewer {
    struct CubePlane {
    public:
        enum Face : int {
            /**
             * @brief options
             */
            NONE = 1 << 0,
            FRONT = 1 << 1,
            BACK = 1 << 2,
            LEFT = 1 << 3,
            RIGHT = 1 << 4,
            TOP = 1 << 5,
            BOTTOM = 1 << 6,
            ALL = FRONT | BACK | LEFT | RIGHT | TOP | BOTTOM
        };

        static bool IsFaceWith(int desired, int curFace) {
            return (desired == (desired & curFace));
        }

        /**
         * @brief override operator '<<' for type 'Face'
         */
        friend std::ostream &operator<<(std::ostream &os, const Face &curFace) {
            std::stringstream stream;
            int count = 0;
            if (IsFaceWith(FRONT, curFace)) {
                stream << "FRONT";
                ++count;
            }
            if (IsFaceWith(BACK, curFace)) {
                stream << " | BACK";
                ++count;
            }
            if (IsFaceWith(LEFT, curFace)) {
                stream << " | LEFT";
                ++count;
            }
            if (IsFaceWith(RIGHT, curFace)) {
                stream << " | RIGHT";
                ++count;
            }
            if (IsFaceWith(TOP, curFace)) {
                stream << " | TOP";
                ++count;
            }
            if (IsFaceWith(BOTTOM, curFace)) {
                stream << " | BOTTOM";
                ++count;
            }
            if (count == 0) {
                os << "NONE";
            } else if (count == 6) {
                os << "ALL";
            } else {
                std::string str = stream.str();
                if (str.at(1) == '|') {
                    str = str.substr(3, str.size() - 3);
                }
                os << str;

            }
            return os;
        };


    protected:
        static ColourWheel COLOUR_WHEEL;
        static std::default_random_engine RANDOM_ENGINE;

    public:
        Posef WtoL;
        Posef LtoW;
        float xSpan;
        float ySpan;
        float zSpan;
        Colour color;

        CubePlane(float roll, float pitch, float yaw,
                  float dx, float dy, float dz,
                  float width = 1.0f, float height = 2.0f, float thickness = 0.1f,
                  const Colour &color = CubePlane::COLOUR_WHEEL.GetUniqueColour());

        CubePlane(float roll, float pitch, float yaw,
                  float dx, float dy, float dz, float width,
                  const Colour &color);

    public:
        [[nodiscard]] pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        GenerateFeatures(int size, const Colour &colour, int faceOption = Face::ALL) const;

        [[nodiscard]] pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        GenerateFeatures(int size, int faceOption = Face::ALL, float opacity = 1.0f) const;

    protected:
        [[nodiscard]] static pcl::PointCloud<pcl::PointXY>::Ptr
        GenerateFeatures(int size, float xMin, float xMax, float yMin, float yMax);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //SLAM_SCENE_VIEWER_CUBE_PLANE_H
