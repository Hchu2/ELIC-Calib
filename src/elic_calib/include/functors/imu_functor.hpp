//
// Created by csl on 10/3/22.
//

#ifndef LIC_CALIB_IMU_FUNCTOR_HPP
#define LIC_CALIB_IMU_FUNCTOR_HPP

#include "functors/functor_typedef.hpp"

namespace ns_elic {
    struct IMUFunctor {
    private:
        SplineMeta _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;
        double _gyroWeight;
        double _acceWeight;
    public:
        explicit IMUFunctor(SplineMeta splineMeta, IMUFrame::Ptr imuFrame, double gyroWeight, double acceWeight)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _gyroWeight(gyroWeight), _acceWeight(acceWeight) {}

        static auto
        Create(const SplineMeta &splineMeta, const IMUFrame::Ptr &imuFrame, double gyroWeight, double acceWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUFunctor>(
                    new IMUFunctor(splineMeta, imuFrame, gyroWeight, acceWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUFunctor).hash_code();
        }

    public:
        /**
        * param blocks:
        * [ SO3 | ... | SO3 | POS | ... | POS | GYRO_BIAS | ACCE_BIAS | G_REFINE | GYRO_MAP_COEFF | ACCE_MAP_COEFF | SO3_AtoG ]
        * [ SO3_IjToIr | POS_IjInIr | TIME_OFFSET_IjToIr ]
        */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector6<T>> residuals(sResiduals);

            std::size_t GYRO_BIAS_OFFSET = 2 * _splineMeta.NumParameters();
            std::size_t ACCE_BIAS_OFFSET = GYRO_BIAS_OFFSET + 1;
            std::size_t G_REFINE_OFFSET = ACCE_BIAS_OFFSET + 1;
            std::size_t GYRO_MAP_COEFF_OFFSET = G_REFINE_OFFSET + 1;
            std::size_t ACCE_MAP_COEFF_OFFSET = GYRO_MAP_COEFF_OFFSET + 1;
            std::size_t SO3_AtoG_OFFSET = ACCE_MAP_COEFF_OFFSET + 1;
            std::size_t SO3_IjToIr_OFFSET = SO3_AtoG_OFFSET + 1;
            std::size_t POS_IjInIr_OFFSET = SO3_IjToIr_OFFSET + 1;
            std::size_t TIME_OFFSET_IjToIr_OFFSET = POS_IjInIr_OFFSET + 1;

            T TIME_OFFSET_IjToIr = sKnots[TIME_OFFSET_IjToIr_OFFSET][0];
            auto refIMUTime = _imuFrame->GetTimestamp() + TIME_OFFSET_IjToIr;
            std::size_t R_offset;
            T u;
            _splineMeta.template ComputeSplineIndex(refIMUTime, R_offset, u);
            std::size_t P_offset = R_offset + _splineMeta.NumParameters();

            Sophus::SO3<T> so3_IrToIr0;
            SO3Tangent<T> so3Vel_IrToIr0InBody, so3Acce_IrToIr0InBody;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate_lie(
                    sKnots + R_offset, u, _dtInv, &so3_IrToIr0, &so3Vel_IrToIr0InBody, &so3Acce_IrToIr0InBody
            );

            Vector3<T> posAcce_IrInIr0;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate<3, 2>(
                    sKnots + P_offset, u, _dtInv, &posAcce_IrInIr0
            );

            Eigen::Map<const Vector3<T>> gyroBias(sKnots[GYRO_BIAS_OFFSET]);
            Eigen::Map<const Vector3<T>> acceBias(sKnots[ACCE_BIAS_OFFSET]);
            Eigen::Map<const Vector3<T>> gravity(sKnots[G_REFINE_OFFSET]);

            auto gyroCoeff = sKnots[GYRO_MAP_COEFF_OFFSET];
            auto acceCoeff = sKnots[ACCE_MAP_COEFF_OFFSET];

            Matrix3<T> gyroMapMat = Matrix3<T>::Zero();
            Matrix3<T> acceMapMat = Matrix3<T>::Zero();

            gyroMapMat.diagonal() = Eigen::Map<const Vector3<T>>(gyroCoeff, 3);
            gyroMapMat(0, 1) = *(gyroCoeff + 3);
            gyroMapMat(0, 2) = *(gyroCoeff + 4);
            gyroMapMat(1, 2) = *(gyroCoeff + 5);

            acceMapMat.diagonal() = Eigen::Map<const Vector3<T>>(acceCoeff, 3);
            acceMapMat(0, 1) = *(acceCoeff + 3);
            acceMapMat(0, 2) = *(acceCoeff + 4);
            acceMapMat(1, 2) = *(acceCoeff + 5);

            Eigen::Map<Sophus::SO3<T> const> const SO3_AtoG(sKnots[SO3_AtoG_OFFSET]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_IjToIr(sKnots[SO3_IjToIr_OFFSET]);
            Eigen::Map<const Vector3<T>> POS_IjInIr(sKnots[POS_IjInIr_OFFSET]);

            SO3Tangent<T> so3Vel_IrToIr0InIr0 = so3_IrToIr0 * so3Vel_IrToIr0InBody;
            SO3Tangent<T> so3Vel_IjToIr0InIr0 = so3Vel_IrToIr0InIr0;
            Sophus::SO3<T> so3_IjToIr0 = so3_IrToIr0 * SO3_IjToIr;
            SO3Tangent<T> so3Acce_IrToIr0InIr0 = so3_IrToIr0 * so3Acce_IrToIr0InBody;

            Matrix3<T> hatCoffMat = Sophus::SO3<T>::hat(so3_IrToIr0 * POS_IjInIr);
            SO3Tangent<T> posAcce_IjInIr0 =
                    -hatCoffMat * so3Acce_IrToIr0InIr0 + posAcce_IrInIr0
                    - Sophus::SO3<T>::hat(so3Vel_IrToIr0InIr0) * hatCoffMat * so3Vel_IrToIr0InIr0;

            Vector3<T> gyroPred =
                    (gyroMapMat * (so3_IjToIr0.inverse() * SO3_AtoG * so3Vel_IjToIr0InIr0)).eval() + gyroBias;
            Vector3<T> accePred =
                    (acceMapMat * (so3_IjToIr0.inverse() * (posAcce_IjInIr0 - gravity))).eval() + acceBias;

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
#endif //LIC_CALIB_IMU_FUNCTOR_HPP
