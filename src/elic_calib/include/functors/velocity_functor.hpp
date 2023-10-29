//
// Created by csl on 3/16/23.
//

#ifndef RIC_CALIB_VELOCITY_FUNCTOR_HPP
#define RIC_CALIB_VELOCITY_FUNCTOR_HPP

#include <utility>

#include "functors/functor_typedef.hpp"

namespace ns_elic {
    struct VelocityFunctor {
    private:
        SplineMeta _splineMeta;

        Eigen::Vector3d _velCurCtoW;
        double _timestamp;
    public:
        VelocityFunctor(SplineMeta splineMeta, Eigen::Vector3d velCurCtoW, double timestamp, double weight)
                : _splineMeta(std::move(splineMeta)), _velCurCtoW(std::move(velCurCtoW)), _timestamp(timestamp),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _weight(weight) {}

    private:
        double _dtInv;
        double _weight;

    public:

        static auto
        Create(const SplineMeta &splineMeta, const Eigen::Vector3d &velCurCtoW, double timestamp, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<VelocityFunctor>(
                    new VelocityFunctor(splineMeta, velCurCtoW, timestamp, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(VelocityFunctor).hash_code();
        }

    public:

        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS | POS_CinI | SCALE ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            std::size_t R_offset;
            std::size_t P_offset;
            double u;
            _splineMeta.ComputeSplineIndex(_timestamp, R_offset, u);
            P_offset = R_offset + _splineMeta.NumParameters();

            std::size_t POS_CinI_OFFSET = 2 * _splineMeta.NumParameters();
            std::size_t SCALE_OFFSET = POS_CinI_OFFSET + 1;
            Eigen::Map<const Vector3<T>> POS_CinI(sKnots[POS_CinI_OFFSET]);
            T SCALE = sKnots[SCALE_OFFSET][0];

            Sophus::SO3<T> SO3_ItoW;
            SO3Tangent<T> VEL_SO3_ItoWinBody;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
                    sKnots + R_offset, u, _dtInv, &SO3_ItoW, &VEL_SO3_ItoWinBody);

            Vector3<T> VEL_POS_IinWinW;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate<T, 3, 1>(
                    sKnots + P_offset, u, _dtInv, &VEL_POS_IinWinW);

            SO3Tangent<T> VEL_SO3_ItoWinW = SO3_ItoW * VEL_SO3_ItoWinBody;
            Vector3<T> VEL_CinWinW_PRED = -Sophus::SO3<T>::hat(SO3_ItoW * POS_CinI) * VEL_SO3_ItoWinW + VEL_POS_IinWinW;

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            residuals = T(_weight) * (VEL_CinWinW_PRED - SCALE * _velCurCtoW.template cast<T>());

            return true;
        }
    };
}

#endif //RIC_CALIB_VELOCITY_FUNCTOR_HPP
