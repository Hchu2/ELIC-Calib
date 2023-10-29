//
// Created by csl on 2/3/23.
//

#ifndef ELIC_CALIB_CENTRALIZATION_FUNCTOR_HPP
#define ELIC_CALIB_CENTRALIZATION_FUNCTOR_HPP

#include "functors/functor_typedef.hpp"

namespace ns_elic {
    struct SO3CentralizationFunctor {
    private:
        std::size_t _blockSize;
        double _weight;

    public:
        explicit SO3CentralizationFunctor(std::size_t blockSize, double weight)
                : _blockSize(blockSize), _weight(weight) {}

        static auto Create(std::size_t blockSize, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<SO3CentralizationFunctor>(
                    new SO3CentralizationFunctor(blockSize, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(SO3CentralizationFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3_I0ToIr | ... | SO3_IjToIr | ... ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            Vector3<T> normSO3 = Vector3<T>::Zero();
            for (int SO3_IjToIr_OFFSET = 0; SO3_IjToIr_OFFSET < _blockSize; ++SO3_IjToIr_OFFSET) {
                Eigen::Map<Sophus::SO3<T> const> const SO3_IjToIr(sKnots[SO3_IjToIr_OFFSET]);
                normSO3 += SO3_IjToIr.log();
            }

            residuals.template block<3, 1>(0, 0) = T(_weight) * normSO3;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct POSCentralizationFunctor {
    private:
        std::size_t _blockSize;
        double _weight;

    public:
        explicit POSCentralizationFunctor(std::size_t blockSize, double weight)
                : _blockSize(blockSize), _weight(weight) {}

        static auto Create(std::size_t blockSize, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<POSCentralizationFunctor>(
                    new POSCentralizationFunctor(blockSize, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(POSCentralizationFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ POS_I0InIr | ... | POS_IjInIr | ... ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            Vector3<T> normPOS = Vector3<T>::Zero();
            for (int POS_IjInIr_OFFSET = 0; POS_IjInIr_OFFSET < _blockSize; ++POS_IjInIr_OFFSET) {
                Eigen::Map<Vector3<T> const> const POS_IjIoIr(sKnots[POS_IjInIr_OFFSET]);
                normPOS += POS_IjIoIr;
            }

            residuals.template block<3, 1>(0, 0) = T(_weight) * normPOS;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct TimeOffsetCentralizationFunctor {
    private:
        std::size_t _blockSize;
        double _weight;

    public:
        explicit TimeOffsetCentralizationFunctor(std::size_t blockSize, double weight)
                : _blockSize(blockSize), _weight(weight) {}

        static auto Create(std::size_t blockSize, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<TimeOffsetCentralizationFunctor>(
                    new TimeOffsetCentralizationFunctor(blockSize, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(TimeOffsetCentralizationFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ TIME_OFFSET_I0ToIr | ... | TIME_OFFSET_IjToIr | ... ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector1<T>> residuals(sResiduals);

            Vector1<T> normTimeOffset = Vector1<T>::Zero();
            for (int TIME_OFFSET_IjToIr_OFFSET = 0;
                 TIME_OFFSET_IjToIr_OFFSET < _blockSize; ++TIME_OFFSET_IjToIr_OFFSET) {
                Eigen::Map<Vector1<T> const> const TIME_OFFSET_IjToIr(sKnots[TIME_OFFSET_IjToIr_OFFSET]);
                normTimeOffset += TIME_OFFSET_IjToIr;
            }

            residuals.template block<1, 1>(0, 0) = T(_weight) * normTimeOffset;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct MultiLiDARRotCentralizationFunctor {
    private:
        const Sophus::SO3d &_SO3_LmToIr;
        double _weight;

    public:
        explicit MultiLiDARRotCentralizationFunctor(const Sophus::SO3d &SO3_LmToIr, double weight)
                : _SO3_LmToIr(SO3_LmToIr), _weight(weight) {}

        static auto Create(const Sophus::SO3d &SO3_LmToIr, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<MultiLiDARRotCentralizationFunctor>(
                    new MultiLiDARRotCentralizationFunctor(SO3_LmToIr, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(MultiLiDARRotCentralizationFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3_LmToLr | SO3_LrToIr ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Eigen::Map<Vector3<T>> so3Residuals(sResiduals);

            std::size_t SO3_LmToLr_OFFSET = 0;
            std::size_t SO3_LrToIr_OFFSET = 1;

            Eigen::Map<Sophus::SO3<T> const> const SO3_LmToLr(sKnots[SO3_LmToLr_OFFSET]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_LrToIr(sKnots[SO3_LrToIr_OFFSET]);

            so3Residuals = T(_weight) * (_SO3_LmToIr * (SO3_LrToIr * SO3_LmToLr).inverse()).log();

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //ELIC_CALIB_CENTRALIZATION_FUNCTOR_HPP
