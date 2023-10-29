//
// Created by csl on 12/9/22.
//

#ifndef LIC_CALIB_IMU_RECOVERY_FUNCTOR_HPP
#define LIC_CALIB_IMU_RECOVERY_FUNCTOR_HPP

#include <utility>

#include "functors/functor_typedef.hpp"

namespace ns_elic {
    struct IMURecoveryFunctor {
    private:
        SplineMeta _splineMeta;
        double _timestamp;

        double _dtInv;
        double _gyroWeight;
        double _acceWeight;

        const Eigen::Vector3d _gravity;

    public:
        explicit IMURecoveryFunctor(SplineMeta splineMeta, double timestamp, Eigen::Vector3d gravity,
                                    double gyroWeight, double acceWeight)
                : _splineMeta(std::move(splineMeta)), _timestamp(timestamp), _gravity(std::move(gravity)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _gyroWeight(gyroWeight), _acceWeight(acceWeight) {}

        static auto
        Create(const SplineMeta &splineMeta, double timestamp, const Eigen::Vector3d &gravity,
               double gyroWeight, double acceWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMURecoveryFunctor>(
                    new IMURecoveryFunctor(splineMeta, timestamp, gravity, gyroWeight, acceWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMURecoveryFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS | GYRO | ACCE ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector6<T>> residuals(sResiduals);

            std::size_t R_offset;
            std::size_t P_offset;
            double u;
            _splineMeta.ComputeSplineIndex(_timestamp, R_offset, u);
            P_offset = R_offset + _splineMeta.NumParameters();

            std::size_t GYRO_OFFSET = 2 * _splineMeta.NumParameters();
            std::size_t ACCE_OFFSET = GYRO_OFFSET + 1;

            Eigen::Map<const Vector3<T>> gyro(sKnots[GYRO_OFFSET]);
            Eigen::Map<const Vector3<T>> acce(sKnots[ACCE_OFFSET]);

            Sophus::SO3<T> so3;
            SO3Tangent<T> so3Vel;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
                    sKnots + R_offset, u, _dtInv, &so3, &so3Vel);

            Vector3<T> posAccel;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate<T, 3, 2>(
                    sKnots + P_offset, u, _dtInv, &posAccel);

            Vector3<T> gyroPred = so3Vel.eval();
            Vector3<T> accePred = (so3.inverse() * (posAccel - _gravity)).eval();

            Vector3<T> gyroResiduals = gyroPred - gyro;
            Vector3<T> acceResiduals = accePred - acce;

            residuals.template block<3, 1>(0, 0) = T(_gyroWeight) * gyroResiduals;
            residuals.template block<3, 1>(3, 0) = T(_acceWeight) * acceResiduals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //LIC_CALIB_IMU_RECOVERY_FUNCTOR_HPP
