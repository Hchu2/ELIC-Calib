//
// Created by csl on 10/22/22.
//

#ifndef SLAM_SCENE_VIEWER_UTILS_HPP
#define SLAM_SCENE_VIEWER_UTILS_HPP

#include "Eigen/Dense"

namespace ns_viewer {

    template<class Scalar, int M, int Options = 0>
    using Vector = Eigen::Matrix<Scalar, M, 1, Options>;

    template<class Scalar, int Options = 0>
    using Vector2 = Vector<Scalar, 2, Options>;
    using Vector2f = Vector2<float>;
    using Vector2d = Vector2<double>;

    template<class Scalar, int Options = 0>
    using Vector3 = Vector<Scalar, 3, Options>;
    using Vector3f = Vector3<float>;
    using Vector3d = Vector3<double>;

    template<class Scalar>
    using Vector4 = Vector<Scalar, 4>;
    using Vector4f = Vector4<float>;
    using Vector4d = Vector4<double>;

    template<class Scalar>
    using Vector6 = Vector<Scalar, 6>;
    using Vector6f = Vector6<float>;
    using Vector6d = Vector6<double>;

    template<class Scalar>
    using Vector7 = Vector<Scalar, 7>;
    using Vector7f = Vector7<float>;
    using Vector7d = Vector7<double>;

    template<class Scalar, int M, int N>
    using Matrix = Eigen::Matrix<Scalar, M, N>;

    template<class Scalar>
    using Matrix2 = Matrix<Scalar, 2, 2>;
    using Matrix2f = Matrix2<float>;
    using Matrix2d = Matrix2<double>;

    template<class Scalar>
    using Matrix3 = Matrix<Scalar, 3, 3>;
    using Matrix3f = Matrix3<float>;
    using Matrix3d = Matrix3<double>;

    template<class Scalar>
    using Matrix4 = Matrix<Scalar, 4, 4>;
    using Matrix4f = Matrix4<float>;
    using Matrix4d = Matrix4<double>;

    template<class Scalar>
    using Matrix6 = Matrix<Scalar, 6, 6>;
    using Matrix6f = Matrix6<float>;
    using Matrix6d = Matrix6<double>;

    template<class Scalar>
    using Matrix7 = Matrix<Scalar, 7, 7>;
    using Matrix7f = Matrix7<float>;
    using Matrix7d = Matrix7<double>;

    template<class Scalar>
    using Matrix34 = Matrix<Scalar, 3, 4>;
    using Matrix34f = Matrix34<float>;
    using Matrix34d = Matrix34<double>;

#define INVALID_TIME_STAMP (-1.0)

    static const float DEG_TO_RAD = M_PI / 180.0;

    static const float RAD_TO_DEG = 180.0 / M_PI;

}

#endif //SLAM_SCENE_VIEWER_UTILS_HPP
