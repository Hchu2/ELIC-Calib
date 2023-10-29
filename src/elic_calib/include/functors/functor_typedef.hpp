//
// Created by csl on 10/3/22.
//

#ifndef LIC_CALIB_FUNCTOR_TYPEDEF_HPP
#define LIC_CALIB_FUNCTOR_TYPEDEF_HPP

#include "basalt/spline/ceres_local_param.hpp"
#include "basalt/spline/ceres_spline_helper.h"
#include "basalt/spline/ceres_spline_helper_jet.h"
#include "basalt/spline/spline_segment.h"
#include "ceres/ceres.h"
#include "config/calib_config.h"

namespace ns_elic {
    using SplineMeta = basalt::SplineMeta<CalibConfig::BSpline::SplineOrder>;
    using CeresSplineHelper = basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>;

    template<typename T>
    using SO3Tangent = typename Sophus::SO3<T>::Tangent;

    template<typename T>
    using Vector1 = Eigen::Matrix<T, 1, 1, 0>;

    template<typename T>
    using Vector2 = Sophus::Vector2<T>;

    template<typename T>
    using Vector3 = Sophus::Vector3<T>;

    template<typename T>
    using Vector6 = Sophus::Vector6<T>;

    template<typename T>
    using Vector9 = Sophus::Vector<T, 9>;

    template<typename T>
    using Matrix1 = Sophus::Matrix<T, 1, 1>;

    template<typename T>
    using Matrix2 = Sophus::Matrix2<T>;

    template<typename T>
    using Matrix3 = Sophus::Matrix3<T>;

    template<typename T>
    Vector3<T> GravityVec(Eigen::Map<Vector2<T> const> &gRefine) {
        auto GRAVITY_NORM = CalibConfig::BSpline::GRefineNorm;
        T cr = ceres::cos(gRefine[0]), sr = ceres::sin(gRefine[0]);
        T cp = ceres::cos(gRefine[1]), sp = ceres::sin(gRefine[1]);
        return Eigen::Matrix<T, 3, 1>(
                -sp * cr * T(GRAVITY_NORM),
                sr * T(GRAVITY_NORM),
                -cr * cp * T(GRAVITY_NORM)
        );
    }

    template<typename T>
    Vector3<T> GravityVec(const Vector2<T> &gRefine) {
        auto GRAVITY_NORM = CalibConfig::BSpline::GRefineNorm;
        T cr = ceres::cos(gRefine[0]), sr = ceres::sin(gRefine[0]);
        T cp = ceres::cos(gRefine[1]), sp = ceres::sin(gRefine[1]);
        return Eigen::Matrix<T, 3, 1>(
                -sp * cr * T(GRAVITY_NORM),
                sr * T(GRAVITY_NORM),
                -cr * cp * T(GRAVITY_NORM)
        );
    }
}

#endif //LIC_CALIB_FUNCTOR_TYPEDEF_HPP
