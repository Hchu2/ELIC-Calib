//
// Created by csl on 10/3/22.
//

#ifndef LIC_CALIB_ODOMETER_POSE_H
#define LIC_CALIB_ODOMETER_POSE_H

#include "Eigen/Dense"
#include "sophus/se3.hpp"
#include "util/macros.hpp"
#include "util/utils.hpp"

namespace ns_elic {
    template<class ScalarType>
    struct OdomPose {
        double timeStamp;
        Eigen::Matrix<ScalarType, 4, 4> pose;

        explicit OdomPose(double timeStamp = INVALID_TIME_STAMP,
                          const Eigen::Matrix<ScalarType, 4, 4> &pose = Eigen::Matrix<ScalarType, 4, 4>::Identity())
                : timeStamp(timeStamp), pose(pose) {
        }

        Eigen::Matrix<ScalarType, 3, 3> Rotation() const {
            return pose.template block<3, 3>(0, 0);
        }

        Eigen::Matrix<ScalarType, 3, 1> Translation() const {
            return pose.template block<3, 1>(0, 3);
        }
    };

    using OdomPosed = OdomPose<double>;
    using OdomPosef = OdomPose<float>;

    template<class ScalarType>
    struct Pose {
    public:

        using Scale = ScalarType;
        using Rotation = Sophus::SO3<Scale>;
        using Translation = Sophus::Vector3<Scale>;

        Rotation so3;
        Translation t;
        double timeStamp;

        Pose(const Rotation &so3, const Translation &t, double timeStamp = INVALID_TIME_STAMP)
                : so3(so3), t(t), timeStamp(timeStamp) {}

        explicit Pose(double timeStamp = INVALID_TIME_STAMP)
                : so3(), t(Translation::Zero()), timeStamp(timeStamp) {}

        Eigen::Quaternion<ScalarType> q() const {
            return so3.unit_quaternion();
        }

        Sophus::Matrix3<ScalarType> R() const {
            return q().toRotationMatrix();
        }

        Sophus::SE3<ScalarType> se3() const {
            return Sophus::SE3<ScalarType>(so3, t);
        }

        Sophus::Matrix4<ScalarType> T() const {
            Sophus::Matrix4<ScalarType> T = Sophus::Matrix4<ScalarType>::Identity();
            T.template block<3, 3>(0, 0) = R();
            T.template block<3, 1>(0, 3) = t;
            return T;
        }

        static Pose
        FromT(const Sophus::Matrix4<ScalarType> &T, double timeStamp = INVALID_TIME_STAMP) {
            Sophus::Matrix3<ScalarType> rotMat = T.template block<3, 3>(0, 0);
            rotMat = AdjustRotationMatrix(rotMat);

            Pose pose(timeStamp);
            pose.so3 = Rotation(rotMat);
            pose.t = T.template block<3, 1>(0, 3);
            return pose;
        }

        static Pose
        FromRt(const Sophus::Matrix3<ScalarType> &R,
               const Sophus::Vector3<ScalarType> &t,
               double timeStamp = INVALID_TIME_STAMP) {
            Sophus::Matrix3<ScalarType> rotMat = AdjustRotationMatrix(R);

            Pose pose(timeStamp);
            pose.so3 = Rotation(rotMat);
            pose.t = t;
            return pose;
        }

        static Pose
        FromSE3(const Sophus::SE3<ScalarType> &se3, double timeStamp = INVALID_TIME_STAMP) {
            Pose pose(timeStamp);
            pose.so3 = se3.so3();
            pose.t = se3.translation();
            return pose;
        }

    public:
        // Serialization
        template<class Archive>
        void save(Archive &archive) const {
            archive(
                    cereal::make_nvp("q", EigenQuaternionToVector(so3.unit_quaternion())),
                    cereal::make_nvp("t", EigenVecToVector(t)),
                    cereal::make_nvp("timestamp", timeStamp)
            );
        }

        template<class Archive>
        void load(Archive &archive) {
            std::vector<ScalarType> quater, trans;
            archive(
                    cereal::make_nvp("q", quater),
                    cereal::make_nvp("t", trans),
                    cereal::make_nvp("timestamp", timeStamp)
            );
            so3 = Sophus::SO3<ScalarType>(VectorToEigenQuaternion(quater));
            t = VectorToEigenVec<ScalarType, 3>(trans);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using Posed = Pose<double>;
    using Posef = Pose<float>;
}

#endif //LIC_CALIB_ODOMETER_POSE_H
