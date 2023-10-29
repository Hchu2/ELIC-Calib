//
// Created by csl on 3/6/23.
//

#ifndef ELIC_CALIB_RELATIVE_SE3_FUNCTOR_HPP
#define ELIC_CALIB_RELATIVE_SE3_FUNCTOR_HPP

#include "functors/functor_typedef.hpp"

namespace ns_elic {
    struct RelativeSE3Functor {
    private:
        SplineMeta _splineMeta;

        Sophus::SE3d SE3_CjToCi;
        double _ti;
        double _tj;

        double _dtInv;
        double _so3Weight;
        double _posWeight;

    public:
        RelativeSE3Functor(SplineMeta splineMeta, const Posed &CiToW, const Posed &CjToW, double so3Weight,
                           double posWeight)
                : _splineMeta(std::move(splineMeta)), SE3_CjToCi(CiToW.se3().inverse() * CjToW.se3()),
                  _ti(CiToW.timeStamp), _tj(CjToW.timeStamp),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _so3Weight(so3Weight), _posWeight(posWeight) {}

        static auto Create(const SplineMeta &splineMeta, const Posed &CiToW, const Posed &CjToW, double so3Weight,
                           double posWeight) {
            return new ceres::DynamicAutoDiffCostFunction<RelativeSE3Functor>(
                    new RelativeSE3Functor(splineMeta, CiToW, CjToW, so3Weight, posWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RelativeSE3Functor).hash_code();
        }

    public:

        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS | SCALE | SO3_CtoI | POS_CinI]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_I_OFFSET, POS_I_OFFSET;
            std::size_t SO3_J_OFFSET, POS_J_OFFSET;
            std::size_t SCALE_OFFSET = 2 * _splineMeta.NumParameters();
            std::size_t SO3_CtoI_OFFSET = SCALE_OFFSET + 1;
            std::size_t POS_CinI_OFFSET = SO3_CtoI_OFFSET + 1;

            // calculate the so3 and pos offset
            std::pair<std::size_t, T> piIU;
            _splineMeta.template ComputeSplineIndex(T(_ti), piIU.first, piIU.second);
            SO3_I_OFFSET = piIU.first;
            POS_I_OFFSET = SO3_I_OFFSET + _splineMeta.NumParameters();
            // query
            Sophus::SO3<T> SO3_IiToRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate_lie(
                    sKnots + SO3_I_OFFSET, piIU.second, _dtInv, &SO3_IiToRef
            );
            Vector3<T> POS_IiInRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate<3, 0>(
                    sKnots + POS_I_OFFSET, piIU.second, _dtInv, &POS_IiInRef
            );

            // calculate the so3 and pos offset
            std::pair<std::size_t, T> pjIU;
            _splineMeta.template ComputeSplineIndex(T(_tj), pjIU.first, pjIU.second);
            SO3_J_OFFSET = pjIU.first;
            POS_J_OFFSET = SO3_J_OFFSET + _splineMeta.NumParameters();
            // query
            Sophus::SO3<T> SO3_IjToRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate_lie(
                    sKnots + SO3_J_OFFSET, pjIU.second, _dtInv, &SO3_IjToRef
            );
            Vector3<T> POS_IjInRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate<3, 0>(
                    sKnots + POS_J_OFFSET, pjIU.second, _dtInv, &POS_IjInRef
            );

            Eigen::Map<const Sophus::SO3<T>> SO3_CtoI(sKnots[SO3_CtoI_OFFSET]);
            Eigen::Map<const Vector3<T>> POS_CinI(sKnots[POS_CinI_OFFSET]);
            Sophus::SE3<T> SE3_CtoI(SO3_CtoI, POS_CinI);

            Sophus::SE3<T> SE3_IiToRef(SO3_IiToRef, POS_IiInRef);
            Sophus::SE3<T> SE3_IjToRef(SO3_IjToRef, POS_IjInRef);
            Sophus::SE3<T> SE3_IjToIi = SE3_IiToRef.inverse() * SE3_IjToRef;
            T SCALE = sKnots[SCALE_OFFSET][0];
            Sophus::SE3<T> SE3_CjToCi_Scaled(
                    SE3_CjToCi.so3().template cast<T>(), SE3_CjToCi.translation().template cast<T>() * SCALE
            );

            Eigen::Map<Vector3<T>> so3Residuals(sResiduals);
            Eigen::Map<Vector3<T>> poseResiduals(sResiduals + 3);

            Sophus::SE3<T> error = (SE3_IjToIi * SE3_CtoI).inverse() * (SE3_CtoI * SE3_CjToCi_Scaled);

            so3Residuals = error.so3().log();
            poseResiduals = error.translation();

            so3Residuals = T(_so3Weight) * so3Residuals;
            poseResiduals = T(_posWeight) * poseResiduals;

            return true;
        }
    };
}

#endif //ELIC_CALIB_RELATIVE_SE3_FUNCTOR_HPP
