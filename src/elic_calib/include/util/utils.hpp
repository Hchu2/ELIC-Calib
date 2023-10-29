
#ifndef LIC_CALIB_UTILS_HPP
#define LIC_CALIB_UTILS_HPP

#include "Eigen/Dense"
#include "cmath"
#include "fmt/color.h"
#include "sophus/se3.hpp"
#include "util/type_define.hpp"
#include "vector"
#include "filesystem"
#include "cereal/cereal.hpp"

namespace ns_elic {
    template<typename Scale, int Rows, int Cols>
    static Eigen::Matrix<Scale, Rows, Cols>
    TrapIntegrationOnce(const std::vector<std::pair<Scale, Eigen::Matrix<Scale, Rows, Cols>>> &data) {
        Eigen::Matrix<Scale, Rows, Cols> sum = Eigen::Matrix<Scale, Rows, Cols>::Zero();
        for (int i = 0; i < data.size() - 1; ++i) {
            int j = i + 1;
            const auto &di = data.at(i);
            const auto &dj = data.at(j);
            sum += (di.second + dj.second) * (dj.first - di.first) * Scale(0.5);
        }
        return sum;
    }

    template<typename Scale, int Rows, int Cols>
    static Eigen::Matrix<Scale, Rows, Cols>
    TrapIntegrationTwice(const std::vector<std::pair<Scale, Eigen::Matrix<Scale, Rows, Cols>>> &data) {
        std::vector<std::pair<Scale, Eigen::Matrix<Scale, Rows, Cols>>> dataOnce;
        Eigen::Matrix<Scale, Rows, Cols> sum = Eigen::Matrix<Scale, Rows, Cols>::Zero();
        for (int i = 0; i < data.size() - 1; ++i) {
            int j = i + 1;
            const auto &di = data.at(i);
            const auto &dj = data.at(j);
            sum += (di.second + dj.second) * (dj.first - di.first) * Scale(0.5);
            dataOnce.push_back({(dj.first + di.first) * Scale(0.5), sum});
        }
        return TrapIntegrationOnce(dataOnce);
    }

    /** sorts vectors from large to small
     * vec: vector to be sorted
     * sortedVec: sorted results
     * ind: the position of each element in the sort result in the original vector
     */
    inline void SortEigenVec3d(const Eigen::Vector3d &vec, Eigen::Vector3d &sortedVec, Eigen::Vector3i &ind) {
        ind = Eigen::Vector3i::LinSpaced(vec.size(), 0, vec.size() - 1);  //[0 1 2]
        auto rule = [vec](int i, int j) -> bool {
            return vec(i) > vec(j);
        };  // regular expression, as a predicate of sort

        std::sort(ind.data(), ind.data() + ind.size(), rule);

        // The data member function returns a pointer to the first element of
        // VectorXd, similar to begin()
        for (int i = 0; i < vec.size(); i++) {
            sortedVec(i) = vec(ind(i));
        }
    }

    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(
            const Eigen::MatrixBase<Derived> &v3d) {
        /**
         *  0 -z  y
         *  z  0 -x
         * -y  x  0
         */
        Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
        m << typename Derived::Scalar(0), -v3d.z(), v3d.y(), v3d.z(),
                typename Derived::Scalar(0), -v3d.x(), -v3d.y(), v3d.x(),
                typename Derived::Scalar(0);
        return m;
    }

    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(
            const Eigen::QuaternionBase<Derived> &q) {
        /**
         * p * q = left_mat(p) * q
         *
         *   1 -qz  qy  qx
         *  qz   1 -qx  qy
         * -qy  qx   1  qz
         * -qx -qy -qz  qw
         */
        Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
        Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
        typename Derived::Scalar q4 = q.w();
        m.block(0, 0, 3, 3) << q4 * Eigen::Matrix3d::Identity() + SkewSymmetric(vq);
        m.block(3, 0, 1, 3) << -vq.transpose();
        m.block(0, 3, 3, 1) << vq;
        m(3, 3) = q4;
        return m;
    }

    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(
            const Eigen::QuaternionBase<Derived> &p) {
        /**
         * p * q = right_mat(q) * p
         *
         *   1  qz -qy  qx
         * -qz   1  qx  qy
         *  qy -qx   1  qz
         * -qx -qy -qz  qw
         */
        Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
        Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
        typename Derived::Scalar p4 = p.w();
        m.block(0, 0, 3, 3) << p4 * Eigen::Matrix3d::Identity() - SkewSymmetric(vp);
        m.block(3, 0, 1, 3) << -vp.transpose();
        m.block(0, 3, 3, 1) << vp;
        m(3, 3) = p4;
        return m;
    }

    template<typename EigenVectorType>
    inline auto EigenVecXToVector(const EigenVectorType &eigenVec) {
        std::vector<typename EigenVectorType::Scalar> vec(eigenVec.rows());
        for (int i = 0; i < vec.size(); ++i) {
            vec.at(i) = eigenVec(i);
        }
        return vec;
    }

    template<typename EigenMatrixType>
    inline auto EigenMatXToVector(const EigenMatrixType &mat) {
        std::vector<std::vector<typename EigenMatrixType::Scalar>> vec(
                mat.rows(), std::vector<typename EigenMatrixType::Scalar>(mat.cols())
        );
        for (int i = 0; i < mat.rows(); ++i) {
            for (int j = 0; j < mat.cols(); ++j) {
                vec.at(i).at(j) = mat(i, j);
            }
        }
        return vec;
    }

    template<typename ScaleType, int M>
    inline auto EigenVecToVector(const Eigen::Matrix<ScaleType, M, 1> &eigenVec) {
        std::vector<ScaleType> vec(M);
        for (int i = 0; i < vec.size(); ++i) {
            vec.at(i) = eigenVec(i);
        }
        return vec;
    }

    template<typename ScaleType, int M>
    inline auto VectorToEigenVec(const std::vector<ScaleType> &vec) {
        Eigen::Matrix<ScaleType, M, 1> eigenVec;
        for (int i = 0; i < vec.size(); ++i) {
            eigenVec(i) = vec.at(i);
        }
        return eigenVec;
    }

    template<typename ScaleType, int M, int N>
    inline auto EigenMatToVector(const Eigen::Matrix<ScaleType, M, N> &mat) {
        std::vector<std::vector<ScaleType>> vec(M, std::vector<ScaleType>(N));
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < N; ++j) {
                vec.at(i).at(j) = mat(i, j);
            }
        }
        return vec;
    }

    template<typename ScaleType, int M, int N>
    inline auto VectorToEigenMat(const std::vector<std::vector<ScaleType>> &vec) {
        Eigen::Matrix<ScaleType, M, N> mat;
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < N; ++j) {
                mat(i, j) = vec.at(i).at(j);
            }
        }
        return mat;
    }

    template<typename ScaleType>
    inline auto EigenQuaternionToVector(const Eigen::Quaternion<ScaleType> &quaternion) {
        std::vector<ScaleType> vec(4);
        vec.at(0) = quaternion.x();
        vec.at(1) = quaternion.y();
        vec.at(2) = quaternion.z();
        vec.at(3) = quaternion.w();
        return vec;
    }

    template<typename ScaleType>
    inline auto VectorToEigenQuaternion(const std::vector<ScaleType> &vec) {
        Eigen::Quaternion<ScaleType> quaternion;
        quaternion.x() = vec.at(0);
        quaternion.y() = vec.at(1);
        quaternion.z() = vec.at(2);
        quaternion.w() = vec.at(3);
        return quaternion;
    }

    inline Eigen::Vector3d RotMatToYPR(const Eigen::Matrix3d &R) {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));

        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;
        return ypr / M_PI * 180.0;
    }

    inline double NormalizeAngle(double ang_degree) {
        if (ang_degree > 180.0) ang_degree -= 360.0;
        if (ang_degree < -180.0) ang_degree += 360.0;
        return ang_degree;
    }

    template<class ScalarType>
    inline Sophus::Matrix3 <ScalarType> AdjustRotationMatrix(const Sophus::Matrix3 <ScalarType> &rotMat) {
        // adjust
        Eigen::JacobiSVD<Sophus::Matrix3<ScalarType>> svd(rotMat, Eigen::ComputeFullV | Eigen::ComputeFullU);
        const Sophus::Matrix3<ScalarType> &vMatrix = svd.matrixV();
        const Sophus::Matrix3<ScalarType> &uMatrix = svd.matrixU();
        Sophus::Matrix3<ScalarType> adjustedRotMat = uMatrix * vMatrix.transpose();
        return adjustedRotMat;
    }

    /**
     * @brief a function to split a string to some string elements according the splitor
     * @param str the string to be split
     * @param splitor the splitor char
     * @param ignoreEmpty whether ignoring the empty string element or not
     * @return the split string vector
     */
    inline std::vector<std::string> SplitString(const std::string &str, char splitor, bool ignoreEmpty = true) {
        std::vector<std::string> vec;
        auto iter = str.cbegin();
        while (true) {
            auto pos = std::find(iter, str.cend(), splitor);
            auto elem = std::string(iter, pos);
            if (!(elem.empty() && ignoreEmpty)) {
                vec.push_back(elem);
            }
            if (pos == str.cend()) {
                break;
            }
            iter = ++pos;
        }
        return vec;
    }

    /**
     * @brief a function to get all the filenames in the directory
     * @param directory the directory
     * @return the filenames in the directory
     */
    inline std::vector<std::string> FilesInDir(const std::string &directory) {
        std::vector<std::string> files;
        for (const auto &elem: std::filesystem::directory_iterator(directory))
            if (elem.status().type() != std::filesystem::file_type::directory)
                files.emplace_back(std::filesystem::canonical(elem.path()).c_str());
        std::sort(files.begin(), files.end());
        return files;
    }

    template<typename ScaleType>
    inline std::string FormatValueVector(const std::vector<const char *> &descVec,
                                         const std::vector<ScaleType> &valVec,
                                         const char *scaleFormatStr = "{:+011.6f}") {
        std::string str;
        const std::size_t M = descVec.size();
        for (int i = 0; i < (M - 1); ++i) {
            str += '\'' + std::string(descVec.at(i)) + "': " +
                   fmt::format(scaleFormatStr, valVec.at(i)) + ", ";
        }
        str += '\'' + std::string(descVec.at(M - 1)) + "': " +
               fmt::format(scaleFormatStr, valVec.at(M - 1));
        return str;
    }

    template<typename ScaleType>
    inline Sophus::Vector2 <ScaleType>
    DistoFunction(const aligned_vector<ScaleType> &distoParams, const Sophus::Vector2 <ScaleType> &p) {
        const ScaleType k1 = distoParams[0], k2 = distoParams[1], k3 = distoParams[2], t1 = distoParams[3], t2 = distoParams[4];
        const ScaleType r2 = p(0) * p(0) + p(1) * p(1);
        const ScaleType r4 = r2 * r2;
        const ScaleType r6 = r4 * r2;
        const ScaleType k_diff = (k1 * r2 + k2 * r4 + k3 * r6);
        const ScaleType t_x = t2 * (r2 + ScaleType(2) * p(0) * p(0)) + ScaleType(2) * t1 * p(0) * p(1);
        const ScaleType t_y = t1 * (r2 + ScaleType(2) * p(1) * p(1)) + ScaleType(2) * t2 * p(0) * p(1);
        return {p(0) * k_diff + t_x, p(1) * k_diff + t_y};
    }

    template<typename ScaleType>
    inline Sophus::Vector2 <ScaleType>
    AddDisto(const aligned_vector<ScaleType> &distoParams, const Sophus::Vector2 <ScaleType> &p) {
        return (p + DistoFunction(distoParams, p));
    }

    template<typename ScaleType>
    Sophus::Vector2 <ScaleType>
    RemoveDisto(const aligned_vector<ScaleType> &distoParams, const Sophus::Vector2 <ScaleType> &p) {
        //criteria to stop the iteration
        const ScaleType epsilon = ScaleType(1e-10);
        Sophus::Vector2<ScaleType> p_u = p;

        Sophus::Vector2<ScaleType> d = DistoFunction(distoParams, p_u);
        //manhattan distance between the two points
        while ((p_u + d - p).template lpNorm<1>() > epsilon) {
            p_u = p - d;
            d = DistoFunction(distoParams, p_u);
        }

        return p_u;
    }

    template<typename ScaleType>
    Sophus::Vector2 <ScaleType>
    ImageToCamera(const Sophus::Vector2 <ScaleType> &focalLength,
                  const Sophus::Vector2 <ScaleType> &focal,
                  const Sophus::Vector2 <ScaleType> &p) {
        Sophus::Vector2<ScaleType> tp = p - focal;
        return {tp(0) / focalLength(0), tp(1) / focalLength(1)};
    }

    template<typename ScaleType>
    Sophus::Vector2 <ScaleType>
    CameraToImage(const Sophus::Vector2 <ScaleType> &focalLength,
                  const Sophus::Vector2 <ScaleType> &focal,
                  const Sophus::Vector2 <ScaleType> &p) {
        Sophus::Vector2<ScaleType> tp(p(0) * focalLength(0), p(1) * focalLength(1));
        return tp + focal;
    }


    template<typename ScaleType, int M, int N>
    Eigen::Matrix<ScaleType, M, N> ReducedRowEchelonForm(const Eigen::Matrix<ScaleType, M, N> &mat) {
        Eigen::Matrix<ScaleType, M, N> rMat = mat;

        std::vector<std::pair<int, int>> indexVec;
        int r = 0, c = 0;
        for (; r < rMat.rows() && c < rMat.cols(); ++c) {
            if (std::abs(rMat(r, c)) < 1E-8) {
                int i = r + 1;
                for (; i < rMat.rows(); ++i) {
                    if (std::abs(rMat(i, c)) > 1E-8) {
                        auto row = rMat.row(r);
                        rMat.row(r) = rMat.row(i);
                        rMat.row(i) = row;
                        break;
                    }
                }
                if (i == rMat.rows()) {
                    continue;
                }
            }
            indexVec.emplace_back(r, c);
            for (int i = r + 1; i < rMat.rows(); ++i) {
                rMat.row(i) -= rMat(i, c) * rMat.row(r) / rMat(r, c);
            }
            r += 1;
        }
        for (auto iter = indexVec.rbegin(); iter != indexVec.rend(); ++iter) {
            auto [row, col] = *iter;
            for (int i = row - 1; i >= 0; --i) {
                rMat.row(i) -= rMat(i, col) / rMat(row, col) * rMat.row(row);
            }
        }
        return rMat;
    }

}

namespace Eigen {
    template<class Archive, typename ScaleType, int Rows, int Cols>
    void serialize(Archive &archive, Eigen::Matrix<ScaleType, Rows, Cols> &m) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                archive(m(i, j));
            }
        }
    }

    template<class Archive, typename ScaleType>
    void serialize(Archive &archive, Eigen::Quaternion<ScaleType> &q) {
        archive(q.coeffs());
    }
}

namespace Sophus {
    template<class Archive, typename ScaleTypes>
    void serialize(Archive &archive, Sophus::SO3<ScaleTypes> &m) {
        archive(m.data()[0], m.data()[1], m.data()[2], m.data()[3]);
    }

    template<class Archive, typename ScaleTypes>
    void serialize(Archive &archive, Sophus::SE3<ScaleTypes> &m) {
        archive(
                // so3
                m.data()[0], m.data()[1], m.data()[2], m.data()[3],
                // trans
                m.data()[4], m.data()[5], m.data()[6]
        );
    }
}

template<class Archive>
void serialize(Archive &ar, PointXYZT &p) {
    ar(
            cereal::make_nvp("x", p.x),
            cereal::make_nvp("y", p.y),
            cereal::make_nvp("z", p.z),
            cereal::make_nvp("timestamp", p.timestamp)
    );
}

namespace pcl {
    template<class Archive>
    void serialize(Archive &ar, PCLHeader &h) {
        ar(
                cereal::make_nvp("stamp", h.stamp),
                cereal::make_nvp("frame_id", h.frame_id),
                cereal::make_nvp("seq", h.seq)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZI &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z),
                cereal::make_nvp("intensity", p.intensity)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZRGB &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z),
                cereal::make_nvp("r", p.r),
                cereal::make_nvp("g", p.g),
                cereal::make_nvp("b", p.b)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZRGBA &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z),
                cereal::make_nvp("r", p.r),
                cereal::make_nvp("g", p.g),
                cereal::make_nvp("b", p.b),
                cereal::make_nvp("a", p.a)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZ &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z)
        );
    }

    template<class Archive, typename PointType>
    void serialize(Archive &ar, pcl::PointCloud<PointType> &cloud) {
        ar(
                cereal::make_nvp("header", cloud.header),
                cereal::make_nvp("points", cloud.points),
                cereal::make_nvp("width", cloud.width),
                cereal::make_nvp("Height", cloud.height),
                cereal::make_nvp("is_dense", cloud.is_dense),
                cereal::make_nvp("sensor_orientation_", cloud.sensor_orientation_),
                cereal::make_nvp("sensor_origin_", cloud.sensor_origin_)
        );
    }
}

#endif // LIC_CALIB_UTILS_HPP