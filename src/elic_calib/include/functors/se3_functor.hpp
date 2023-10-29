//
// Created by csl on 10/4/22.
//

#ifndef LIC_CALIB_SE3_FUNCTOR_HPP
#define LIC_CALIB_SE3_FUNCTOR_HPP

#include "functors/functor_typedef.hpp"
#include "core/pose.hpp"

namespace ns_elic {
    struct SE3Functor {
    private:
        SplineMeta _splineMeta;
        Posed _ItoG{};

        double _dtInv;
        double _so3Weight;
        double _posWeight;

    public:
        explicit SE3Functor(SplineMeta splineMeta, Posed ItoG, double so3Weight, double posWeight)
                : _splineMeta(std::move(splineMeta)), _ItoG(std::move(ItoG)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _so3Weight(so3Weight), _posWeight(posWeight) {}

        static auto Create(const SplineMeta &splineMeta, const Posed &ItoG, double so3Weight, double posWeight) {
            return new ceres::DynamicAutoDiffCostFunction<SE3Functor>(
                    new SE3Functor(splineMeta, ItoG, so3Weight, posWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(SE3Functor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_OFFSET;
            double u;
            // generate the param: 'SO3_OFFSET', 'u'
            _splineMeta.template ComputeSplineIndex(_ItoG.timeStamp, SO3_OFFSET, u);
            std::size_t POS_OFFSET = SO3_OFFSET + _splineMeta.NumParameters();

            Sophus::SO3<T> predSO3ItoG;
            CeresSplineHelper::evaluate_lie<T, Sophus::SO3>(
                    sKnots + SO3_OFFSET, u, _dtInv, &predSO3ItoG
            );
            Vector3<T> predPOSTtoG;
            // if 'DERIV = 0' returns value of the spline, otherwise corresponding derivative.
            CeresSplineHelper::evaluate<T, 3, 0>(
                    sKnots + POS_OFFSET, u, _dtInv, &predPOSTtoG
            );

            Eigen::Map<Vector3<T>> so3Residuals(sResiduals);
            Eigen::Map<Vector3<T>> poseResiduals(sResiduals + 3);

            so3Residuals = (_ItoG.so3.template cast<T>() * predSO3ItoG.inverse()).log();
            poseResiduals = predPOSTtoG - _ItoG.t.template cast<T>();

            so3Residuals = T(_so3Weight) * so3Residuals;
            poseResiduals = T(_posWeight) * poseResiduals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct SO3Functor {
    private:
        SplineMeta _splineMeta;
        Posed _ItoG{};

        double _dtInv;
        double _so3Weight;

    public:
        explicit SO3Functor(SplineMeta splineMeta, Posed ItoG, double so3Weight)
                : _splineMeta(std::move(splineMeta)),
                  _ItoG(std::move(ItoG)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt),
                  _so3Weight(so3Weight) {}

        static auto Create(const SplineMeta &splineMeta, const Posed &ItoG, double so3Weight) {
            return new ceres::DynamicAutoDiffCostFunction<SO3Functor>(
                    new SO3Functor(splineMeta, ItoG, so3Weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(SO3Functor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_OFFSET;
            double u;
            // generate the param: 'SO3_OFFSET', 'u'
            _splineMeta.template ComputeSplineIndex(_ItoG.timeStamp, SO3_OFFSET, u);

            Sophus::SO3<T> predSO3ItoG;
            CeresSplineHelper::evaluate_lie<T, Sophus::SO3>(
                    sKnots + SO3_OFFSET, u, _dtInv, &predSO3ItoG
            );

            Eigen::Map<Vector3<T>> so3Residuals(sResiduals);

            so3Residuals = (predSO3ItoG * _ItoG.so3.inverse().template cast<T>()).log();

            so3Residuals = T(_so3Weight) * so3Residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct PO3Functor {
    private:
        SplineMeta _splineMeta;
        Posed _ItoG{};

        double _dtInv;
        double _posWeight;

    public:
        explicit PO3Functor(SplineMeta splineMeta, Posed ItoG, double posWeight)
                : _splineMeta(std::move(splineMeta)),
                  _ItoG(std::move(ItoG)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt),
                  _posWeight(posWeight) {}

        static auto Create(const SplineMeta &splineMeta, const Posed &ItoG, double posWeight) {
            return new ceres::DynamicAutoDiffCostFunction<PO3Functor>(
                    new PO3Functor(splineMeta, ItoG, posWeight)
            );
        }


        static std::size_t TypeHashCode() {
            return typeid(PO3Functor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ POS | ... | POS ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t POS_OFFSET;
            double u;
            _splineMeta.template ComputeSplineIndex(_ItoG.timeStamp, POS_OFFSET, u);

            Vector3<T> predPOSTtoG;
            // if 'DERIV = 0' returns value of the spline, otherwise corresponding derivative.
            CeresSplineHelper::evaluate<T, 3, 0>(
                    sKnots + POS_OFFSET, u, _dtInv, &predPOSTtoG
            );

            Eigen::Map<Vector3<T>> poseResiduals(sResiduals + 0);

            poseResiduals = predPOSTtoG - _ItoG.t.template cast<T>();

            poseResiduals = T(_posWeight) * poseResiduals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //LIC_CALIB_SE3_FUNCTOR_HPP
