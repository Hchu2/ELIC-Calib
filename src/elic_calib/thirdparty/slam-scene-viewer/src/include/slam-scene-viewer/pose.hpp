//
// Created by csl on 10/3/22.
//

#ifndef SLAM_SCENE_VIEWER_POSE_H
#define SLAM_SCENE_VIEWER_POSE_H

#include "slam-scene-viewer/utils.hpp"
#include "Eigen/Geometry"

namespace ns_viewer {
    template<class ScalarType>
    struct Pose {
    public:
        using Scale = ScalarType;
        using Rotation = Matrix<Scale, 3, 3>;
        using Translation = Vector3<Scale>;
        using Transform = Matrix<Scale, 4, 4>;

    public:
        double timeStamp;
        Rotation rotation;
        Translation translation;

        explicit Pose(const Transform &transform,
                      double timeStamp = INVALID_TIME_STAMP)
                : timeStamp(timeStamp),
                  rotation(transform.topLeftCorner(3, 3)),
                  translation(transform.topRightCorner(3, 1)) {}

        explicit Pose(const Rotation &rotation = Rotation::Identity(),
                      const Translation &translation = Translation::Zero(),
                      double timeStamp = INVALID_TIME_STAMP)
                : timeStamp(timeStamp), rotation(rotation), translation(translation) {}

        Transform matrix44() const {
            Transform transform = Transform::Identity();
            transform.topLeftCorner(3, 3) = rotation;
            transform.topRightCorner(3, 1) = translation;
            return transform;
        }

        Matrix34<Scale> matrix34() const {
            ns_viewer::Matrix34<Scale> transform = ns_viewer::Matrix34<Scale>::Identity();
            transform.topLeftCorner(3, 3) = rotation;
            transform.topRightCorner(3, 1) = translation;
            return transform;
        }

        Vector3<Scale> trans(const Vector3<Scale> &p) const {
            return rotation * p + translation;
        }

        Eigen::Quaternion<Scale> quaternion() const {
            return Eigen::Quaternion<Scale>(rotation);
        }

        Pose inverse() const {
            return Pose(rotation.inverse(), -rotation.inverse() * translation, timeStamp);
        }

        template<class TarScale>
        Pose<TarScale> cast() const {
            return Pose<TarScale>(rotation.template cast<TarScale>(), translation.template cast<TarScale>());
        }

    };

    using Posed = Pose<double>;
    using Posef = Pose<float>;
}

#endif //SLAM_SCENE_VIEWER_POSE_H
