//
// Created by csl on 10/2/22.
//

#ifndef LIC_CALIB_IMU_GYRO_FUNCTOR_HPP
#define LIC_CALIB_IMU_GYRO_FUNCTOR_HPP

#include "functors/functor_typedef.hpp"

namespace ns_elic {
    struct IMUGyroFunctor {
    private:
        SplineMeta _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;
        double _gyroWeight;

    public:
        explicit IMUGyroFunctor(SplineMeta splineMeta, IMUFrame::Ptr imuFrame, double gyroWeight)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _gyroWeight(gyroWeight) {}

        static auto Create(const SplineMeta &splineMeta, const IMUFrame::Ptr &imuFrame, double gyroWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUGyroFunctor>(
                    new IMUGyroFunctor(splineMeta, imuFrame, gyroWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUGyroFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | GYRO_BIAS | GYRO_MAP_COEFF ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_OFFSET;
            double u;
            _splineMeta.template ComputeSplineIndex(_imuFrame->GetTimestamp(), SO3_OFFSET, u);

            std::size_t GYRO_BIAS_OFFSET = _splineMeta.NumParameters();
            std::size_t GYRO_MAP_COEFF_OFFSET = GYRO_BIAS_OFFSET + 1;

            SO3Tangent<T> gyroVel;
            CeresSplineHelper::evaluate_lie<T, Sophus::SO3>(
                    sKnots + SO3_OFFSET, u, _dtInv, nullptr, &gyroVel
            );

            Eigen::Map<SO3Tangent<T> const> gyroBias(sKnots[GYRO_BIAS_OFFSET]);
            Eigen::Map<Matrix3<T> const> gyroMapMat(sKnots[GYRO_MAP_COEFF_OFFSET]);

            Eigen::Map<Vector3<T>> residuals(sResiduals);
            residuals = gyroMapMat * gyroVel + gyroBias - _imuFrame->GetGyro().template cast<T>();
            residuals = T(_gyroWeight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct IMUGyroIdealFunctor {
    private:
        SplineMeta _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;
        double _gyroWeight;

    public:
        explicit IMUGyroIdealFunctor(SplineMeta splineMeta, IMUFrame::Ptr imuFrame, double gyroWeight)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _gyroWeight(gyroWeight) {}

        static auto Create(const SplineMeta &splineMeta, const IMUFrame::Ptr &imuFrame, double gyroWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUGyroIdealFunctor>(
                    new IMUGyroIdealFunctor(splineMeta, imuFrame, gyroWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUGyroIdealFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | SO3_IjToIr ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_OFFSET;
            double u;
            _splineMeta.template ComputeSplineIndex(_imuFrame->GetTimestamp(), SO3_OFFSET, u);

            Sophus::SO3<T> so3_IrToIr0;
            SO3Tangent<T> so3Vel_IrToIr0InBody;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
                    sKnots + SO3_OFFSET, u, _dtInv, &so3_IrToIr0, &so3Vel_IrToIr0InBody
            );

            std::size_t SO3_IjToIr_OFFSET = _splineMeta.NumParameters();
            Eigen::Map<Sophus::SO3<T> const> const SO3_IjToIr(sKnots[SO3_IjToIr_OFFSET]);

            Sophus::SO3<T> so3_IjToIr0 = so3_IrToIr0 * SO3_IjToIr;
            SO3Tangent<T> so3Vel_IrToIr0InIr0 = so3_IrToIr0 * so3Vel_IrToIr0InBody;
            SO3Tangent<T> so3Vel_IjToIr0InIr0 = so3Vel_IrToIr0InIr0;

            Vector3<T> gyroPred = so3_IjToIr0.inverse() * so3Vel_IjToIr0InIr0;

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            residuals = T(_gyroWeight) * (gyroPred - _imuFrame->GetGyro().template cast<T>());

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //LIC_CALIB_IMU_GYRO_FUNCTOR_HPP
