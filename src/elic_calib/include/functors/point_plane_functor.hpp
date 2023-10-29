//
// Created by csl on 10/5/22.
//

#ifndef LIC_CALIB_POINT_PLANE_FUNCTOR_HPP
#define LIC_CALIB_POINT_PLANE_FUNCTOR_HPP


#include "functors/functor_typedef.hpp"
#include "core/surfel_association.h"

namespace ns_elic {
    struct PointPlaneFunctor {
    private:
        SplineMeta _splineMeta;
        PointPlaneCorrespondence _corr;

        double _dtInv;
        double _pointPlaneWeight;

    public:
        explicit PointPlaneFunctor(SplineMeta splineMeta, PointPlaneCorrespondence corr, double pointPlaneWeight)
                : _splineMeta(std::move(splineMeta)), _corr(std::move(corr)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _pointPlaneWeight(pointPlaneWeight) {}

        static auto
        Create(const SplineMeta &splineMeta, const PointPlaneCorrespondence &corr, double pointPlaneWeight) {
            return new ceres::DynamicAutoDiffCostFunction<PointPlaneFunctor>(
                    new PointPlaneFunctor(splineMeta, corr, pointPlaneWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(PointPlaneFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS | SO3_LmToLr | POS_LmInLr | SO3_LrToIr | POS_LrInIr | TIME_OFFSET_LmToLr | TIME_OFFSET_LrToIr ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_OFFSET;
            std::size_t POS_OFFSET;
            std::size_t SO3_LmToLr_OFFSET = 2 * _splineMeta.NumParameters();
            std::size_t POS_LmInLr_OFFSET = SO3_LmToLr_OFFSET + 1;
            std::size_t SO3_LrToIr_OFFSET = POS_LmInLr_OFFSET + 1;
            std::size_t POS_LrInIr_OFFSET = SO3_LrToIr_OFFSET + 1;
            std::size_t TIME_OFFSET_LmToLr_OFFSET = POS_LrInIr_OFFSET + 1;
            std::size_t TIME_OFFSET_LrToIr_OFFSET = TIME_OFFSET_LmToLr_OFFSET + 1;

            // get value
            Eigen::Map<const Sophus::SO3<T>> SO3_LmToLr(sKnots[SO3_LmToLr_OFFSET]);
            Eigen::Map<const Sophus::SO3<T>> SO3_LrToIr(sKnots[SO3_LrToIr_OFFSET]);
            Eigen::Map<const Vector3<T>> POS_LmInLr(sKnots[POS_LmInLr_OFFSET]);
            Eigen::Map<const Vector3<T>> POS_LrInIr(sKnots[POS_LrInIr_OFFSET]);

            T TIME_OFFSET_LmToLr = sKnots[TIME_OFFSET_LmToLr_OFFSET][0];
            T TIME_OFFSET_LrToIr = sKnots[TIME_OFFSET_LrToIr_OFFSET][0];

            auto pointIMUTime = _corr.pointTimeStamp + TIME_OFFSET_LmToLr + TIME_OFFSET_LrToIr;

            // calculate the so3 and pos offset
            std::pair<std::size_t, T> pointIU;
            _splineMeta.template ComputeSplineIndex(pointIMUTime, pointIU.first, pointIU.second);

            SO3_OFFSET = pointIU.first;
            POS_OFFSET = SO3_OFFSET + _splineMeta.NumParameters();

            // query
            Sophus::SO3<T> point_SO3_ItoRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate_lie(
                    sKnots + SO3_OFFSET, pointIU.second, _dtInv, &point_SO3_ItoRef
            );
            Vector3<T> point_POS_IinRef;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate<3, 0>(
                    sKnots + POS_OFFSET, pointIU.second, _dtInv, &point_POS_IinRef
            );

            // construct the residuals
            Vector3<T> pointInLr = SO3_LmToLr * _corr.point.template cast<T>() + POS_LmInLr;
            Vector3<T> pointInIr = SO3_LrToIr * pointInLr + POS_LrInIr;
            Vector3<T> pointInRef = point_SO3_ItoRef * pointInIr + point_POS_IinRef;

            Vector3<T> planeNorm = _corr.planeCoeff.head(3).template cast<T>();
            T distance = pointInRef.template dot(planeNorm) + T(_corr.planeCoeff(3));

            Eigen::Map<Matrix1<T>> residuals(sResiduals);
            residuals.template block<1, 1>(0, 0) = T(_pointPlaneWeight) * Matrix1<T>(distance);

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //LIC_CALIB_POINT_PLANE_FUNCTOR_HPP
