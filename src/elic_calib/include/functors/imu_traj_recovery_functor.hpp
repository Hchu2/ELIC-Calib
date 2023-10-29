//
// Created by csl on 1/18/23.
//

#ifndef LIC_CALIB_IMU_TRAJ_RECOVERY_FUNCTOR_HPP
#define LIC_CALIB_IMU_TRAJ_RECOVERY_FUNCTOR_HPP

#include <utility>

#include "functors/functor_typedef.hpp"
#include "sensor/imu.h"

namespace ns_elic {
    struct IMUTrajRecoveryFunctor {
    private:
        SplineMeta _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;

        Eigen::Vector3d _gravityInRef;
        double _gyroWeight;
        double _acceWeight;
    public:
        explicit IMUTrajRecoveryFunctor(SplineMeta splineMeta, IMUFrame::Ptr imuFrame,
                                        Eigen::Vector3d gravityInRef, double gyroWeight, double acceWeight)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _gravityInRef(std::move(gravityInRef)), _dtInv(1.0 / _splineMeta.segments.front().dt),
                  _gyroWeight(gyroWeight), _acceWeight(acceWeight) {}

        static auto
        Create(const SplineMeta &splineMeta, const IMUFrame::Ptr &imuFrame, const Eigen::Vector3d &gravityInRef,
               double gyroWeight, double acceWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUTrajRecoveryFunctor>(
                    new IMUTrajRecoveryFunctor(splineMeta, imuFrame, gravityInRef, gyroWeight, acceWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUTrajRecoveryFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector6<T>> residuals(sResiduals);

            std::size_t R_offset;
            std::size_t P_offset;
            double u;
            _splineMeta.ComputeSplineIndex(_imuFrame->GetTimestamp(), R_offset, u);
            P_offset = R_offset + _splineMeta.NumParameters();

            Sophus::SO3<T> so3;
            SO3Tangent<T> so3Vel;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
                    sKnots + R_offset, u, _dtInv, &so3, &so3Vel
            );

            Vector3<T> posAcce;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate<T, 3, 2>(
                    sKnots + P_offset, u, _dtInv, &posAcce
            );

            Vector3<T> gyroPred = so3Vel;
            Vector3<T> accePred = so3.inverse() * (posAcce - _gravityInRef);

            Vector3<T> gyroResiduals = gyroPred - _imuFrame->GetGyro().template cast<T>();
            Vector3<T> acceResiduals = accePred - _imuFrame->GetAcce().template cast<T>();

            residuals.template block<3, 1>(0, 0) = T(_gyroWeight) * gyroResiduals;
            residuals.template block<3, 1>(3, 0) = T(_acceWeight) * acceResiduals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //LIC_CALIB_IMU_TRAJ_RECOVERY_FUNCTOR_HPP
