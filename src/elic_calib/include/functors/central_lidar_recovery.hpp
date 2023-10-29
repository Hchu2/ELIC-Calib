//
// Created by csl on 4/26/23.
//

#ifndef ELIC_CALIB_CENTRAL_LIDAR_RECOVERY_HPP
#define ELIC_CALIB_CENTRAL_LIDAR_RECOVERY_HPP

#include "core/pose.hpp"
#include "functors/functor_typedef.hpp"

namespace ns_elic {
    struct CentralLiDARRecoveryFunctor {
    private:
        SplineMeta _splineMeta;
        double _dtInv;

        Sophus::SE3d SE3_CurLmToMm;
        double _tMap;
        double _tCur;

        double _so3Weight;
        double _posWeight;

    public:
        explicit CentralLiDARRecoveryFunctor(SplineMeta splineMeta, const Posed &CurLmToMm, double curMapTime,
                                             double so3Weight, double posWeight)
                : _splineMeta(std::move(splineMeta)), _dtInv(1.0 / _splineMeta.segments.front().dt),
                  SE3_CurLmToMm(CurLmToMm.se3()), _tCur(CurLmToMm.timeStamp), _tMap(curMapTime),
                  _so3Weight(so3Weight), _posWeight(posWeight) {}

        static auto Create(const SplineMeta &splineMeta, const Posed &CurLmToMm,
                           double mapTime, double so3Weight, double posWeigh) {
            return new ceres::DynamicAutoDiffCostFunction<CentralLiDARRecoveryFunctor>(
                    new CentralLiDARRecoveryFunctor(splineMeta, CurLmToMm, mapTime, so3Weight, posWeigh)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(CentralLiDARRecoveryFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS |SO3_LmToLr | POS_LmInLr ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            // array offset
            std::size_t SO3_MAP_OFFSET, POS_MAP_OFFSET;
            std::size_t SO3_CUR_OFFSET, POS_CUR_OFFSET;
            std::size_t SO3_LmToLr_OFFSET = 2 * _splineMeta.NumParameters();
            std::size_t POS_LmInLr_OFFSET = SO3_LmToLr_OFFSET + 1;

            // calculate the so3 and pos offset at map time
            std::pair<std::size_t, T> pMapIU;
            _splineMeta.template ComputeSplineIndex(T(_tMap), pMapIU.first, pMapIU.second);
            SO3_MAP_OFFSET = pMapIU.first;
            POS_MAP_OFFSET = SO3_MAP_OFFSET + _splineMeta.NumParameters();
            // query
            Sophus::SO3<T> SO3_MapToRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate_lie(
                    sKnots + SO3_MAP_OFFSET, pMapIU.second, _dtInv, &SO3_MapToRef
            );
            Vector3<T> POS_MapInRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate<3, 0>(
                    sKnots + POS_MAP_OFFSET, pMapIU.second, _dtInv, &POS_MapInRef
            );

            // calculate the so3 and pos offset at current time
            std::pair<std::size_t, T> pCurIU;
            _splineMeta.template ComputeSplineIndex(T(_tCur), pCurIU.first, pCurIU.second);
            SO3_CUR_OFFSET = pCurIU.first;
            POS_CUR_OFFSET = SO3_CUR_OFFSET + _splineMeta.NumParameters();
            // query
            Sophus::SO3<T> SO3_CurToRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate_lie(
                    sKnots + SO3_CUR_OFFSET, pCurIU.second, _dtInv, &SO3_CurToRef
            );
            Vector3<T> POS_CurInRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate<3, 0>(
                    sKnots + POS_CUR_OFFSET, pCurIU.second, _dtInv, &POS_CurInRef
            );

            Sophus::SE3<T> SE3_MapToRef(SO3_MapToRef, POS_MapInRef);
            Sophus::SE3<T> SE3_CurToRef(SO3_CurToRef, POS_CurInRef);
            Sophus::SE3<T> SE3_CurToMap = SE3_MapToRef.inverse() * SE3_CurToRef;

            Eigen::Map<const Sophus::SO3<T>> SO3_LmToLr(sKnots[SO3_LmToLr_OFFSET]);
            Eigen::Map<const Vector3<T>> POS_LmInLr(sKnots[POS_LmInLr_OFFSET]);
            Sophus::SE3<T> SE3_LmToLr(SO3_LmToLr, POS_LmInLr);

            Eigen::Map<Vector3<T>> so3Residuals(sResiduals);
            Eigen::Map<Vector3<T>> posResiduals(sResiduals + 3);

            Sophus::SE3<T> error = (SE3_CurToMap * SE3_LmToLr).inverse() * (SE3_LmToLr * SE3_CurLmToMm.cast<T>());

            so3Residuals = T(_so3Weight) * error.so3().log();
            posResiduals = T(_posWeight) * error.translation();

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //ELIC_CALIB_CENTRAL_LIDAR_RECOVERY_HPP
