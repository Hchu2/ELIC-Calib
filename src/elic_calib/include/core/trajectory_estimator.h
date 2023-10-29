//
// Created by csl on 10/2/22.
//

#ifndef LIC_CALIB_TRAJECTORY_ESTIMATOR_H
#define LIC_CALIB_TRAJECTORY_ESTIMATOR_H

#include <utility>

#include "calib/calib_param_manager.h"
#include "ceres/local_parameterization.h"
#include "sensor/imu.h"
#include "sensor/lidar.h"
#include "core/pose.hpp"
#include "core/se3_spline.h"
#include "core/sfm_association.h"
#include "core/surfel_association.h"
#include "core/system_observability.h"
#include "ceres/manifold.h"

namespace ns_elic {

    struct OptimizationOption {
    public:
        // myenumGenor Option OPT_SO3 OPT_POS OPT_SO3_IjToIr OPT_SO3_LmToLr OPT_SO3_LrToIr OPT_SO3_CkToIr OPT_POS_IjInIr OPT_POS_LmInLr OPT_POS_LrInIr OPT_POS_CkInIr OPT_TIME_OFFSET_IjToIr OPT_TIME_OFFSET_LmToLr OPT_TIME_OFFSET_LrToIr OPT_TIME_OFFSET_CkToIr OPT_TIME_READOUT OPT_GYRO_BIAS OPT_GYRO_MAP_COEFF OPT_ACCE_BIAS OPT_ACCE_MAP_COEFF OPT_SO3_AtoG OPT_GRAVITY_REFINE OPT_INV_DEPTH OPT_STRUCTURE_SCALE OPT_FOCAL_LENGTH OPT_FOCAL OPT_DISTO_PARAMS
        enum Option : std::uint32_t {
            /**
             * @brief options
             */
            NONE = 1 << 0,

            OPT_SO3 = 1 << 1,
            OPT_POS = 1 << 2,

            OPT_SO3_IjToIr = 1 << 3,
            OPT_SO3_LmToLr = 1 << 4,
            OPT_SO3_LrToIr = 1 << 5,
            OPT_SO3_CkToIr = 1 << 6,

            OPT_POS_IjInIr = 1 << 7,
            OPT_POS_LmInLr = 1 << 8,
            OPT_POS_LrInIr = 1 << 9,
            OPT_POS_CkInIr = 1 << 10,

            OPT_TIME_OFFSET_IjToIr = 1 << 11,
            OPT_TIME_OFFSET_LmToLr = 1 << 12,
            OPT_TIME_OFFSET_LrToIr = 1 << 13,
            OPT_TIME_OFFSET_CkToIr = 1 << 14,
            OPT_TIME_READOUT = 1 << 15,

            OPT_GYRO_BIAS = 1 << 16,
            OPT_GYRO_MAP_COEFF = 1 << 17,

            OPT_ACCE_BIAS = 1 << 18,
            OPT_ACCE_MAP_COEFF = 1 << 19,

            OPT_SO3_AtoG = 1 << 20,

            OPT_GRAVITY_REFINE = 1 << 21,

            OPT_INV_DEPTH = 1 << 22,
            OPT_STRUCTURE_SCALE = 1 << 23,

            OPT_FOCAL_LENGTH = 1 << 24,
            OPT_FOCAL = 1 << 25,
            OPT_DISTO_PARAMS = 1 << 26,

            ALL = OPT_SO3 | OPT_POS | OPT_SO3_IjToIr | OPT_SO3_LmToLr | OPT_SO3_LrToIr |
                  OPT_SO3_CkToIr | OPT_POS_IjInIr | OPT_POS_LmInLr | OPT_POS_LrInIr |
                  OPT_POS_CkInIr | OPT_TIME_OFFSET_IjToIr | OPT_TIME_OFFSET_LmToLr |
                  OPT_TIME_OFFSET_LrToIr | OPT_TIME_OFFSET_CkToIr | OPT_TIME_READOUT |
                  OPT_GYRO_BIAS | OPT_GYRO_MAP_COEFF | OPT_ACCE_BIAS |
                  OPT_ACCE_MAP_COEFF | OPT_SO3_AtoG | OPT_GRAVITY_REFINE | OPT_INV_DEPTH |
                  OPT_STRUCTURE_SCALE | OPT_FOCAL_LENGTH | OPT_FOCAL | OPT_DISTO_PARAMS
        };

        static bool IsOptionWith(std::uint32_t desired, std::uint32_t curOption) {
            return (desired == (desired & curOption));
        }

        /**
         * @brief override operator '<<' for type 'Option'
         */
        friend std::ostream &operator<<(std::ostream &os, const Option &curOption) {
            std::stringstream stream;
            int count = 0;
            if (IsOptionWith(OPT_SO3, curOption)) {
                stream << "OPT_SO3";
                ++count;
            }
            if (IsOptionWith(OPT_POS, curOption)) {
                stream << " | OPT_POS";
                ++count;
            }
            if (IsOptionWith(OPT_SO3_IjToIr, curOption)) {
                stream << " | OPT_SO3_IjToIr";
                ++count;
            }
            if (IsOptionWith(OPT_SO3_LmToLr, curOption)) {
                stream << " | OPT_SO3_LmToLr";
                ++count;
            }
            if (IsOptionWith(OPT_SO3_LrToIr, curOption)) {
                stream << " | OPT_SO3_LrToIr";
                ++count;
            }
            if (IsOptionWith(OPT_SO3_CkToIr, curOption)) {
                stream << " | OPT_SO3_CkToIr";
                ++count;
            }
            if (IsOptionWith(OPT_POS_IjInIr, curOption)) {
                stream << " | OPT_POS_IjInIr";
                ++count;
            }
            if (IsOptionWith(OPT_POS_LmInLr, curOption)) {
                stream << " | OPT_POS_LmInLr";
                ++count;
            }
            if (IsOptionWith(OPT_POS_LrInIr, curOption)) {
                stream << " | OPT_POS_LrInIr";
                ++count;
            }
            if (IsOptionWith(OPT_POS_CkInIr, curOption)) {
                stream << " | OPT_POS_CkInIr";
                ++count;
            }
            if (IsOptionWith(OPT_TIME_OFFSET_IjToIr, curOption)) {
                stream << " | OPT_TIME_OFFSET_IjToIr";
                ++count;
            }
            if (IsOptionWith(OPT_TIME_OFFSET_LmToLr, curOption)) {
                stream << " | OPT_TIME_OFFSET_LmToLr";
                ++count;
            }
            if (IsOptionWith(OPT_TIME_OFFSET_LrToIr, curOption)) {
                stream << " | OPT_TIME_OFFSET_LrToIr";
                ++count;
            }
            if (IsOptionWith(OPT_TIME_OFFSET_CkToIr, curOption)) {
                stream << " | OPT_TIME_OFFSET_CkToIr";
                ++count;
            }
            if (IsOptionWith(OPT_TIME_READOUT, curOption)) {
                stream << " | OPT_TIME_READOUT";
                ++count;
            }
            if (IsOptionWith(OPT_GYRO_BIAS, curOption)) {
                stream << " | OPT_GYRO_BIAS";
                ++count;
            }
            if (IsOptionWith(OPT_GYRO_MAP_COEFF, curOption)) {
                stream << " | OPT_GYRO_MAP_COEFF";
                ++count;
            }
            if (IsOptionWith(OPT_ACCE_BIAS, curOption)) {
                stream << " | OPT_ACCE_BIAS";
                ++count;
            }
            if (IsOptionWith(OPT_ACCE_MAP_COEFF, curOption)) {
                stream << " | OPT_ACCE_MAP_COEFF";
                ++count;
            }
            if (IsOptionWith(OPT_SO3_AtoG, curOption)) {
                stream << " | OPT_SO3_AtoG";
                ++count;
            }
            if (IsOptionWith(OPT_GRAVITY_REFINE, curOption)) {
                stream << " | OPT_GRAVITY_REFINE";
                ++count;
            }
            if (IsOptionWith(OPT_INV_DEPTH, curOption)) {
                stream << " | OPT_INV_DEPTH";
                ++count;
            }
            if (IsOptionWith(OPT_STRUCTURE_SCALE, curOption)) {
                stream << " | OPT_STRUCTURE_SCALE";
                ++count;
            }
            if (IsOptionWith(OPT_FOCAL_LENGTH, curOption)) {
                stream << " | OPT_FOCAL_LENGTH";
                ++count;
            }
            if (IsOptionWith(OPT_FOCAL, curOption)) {
                stream << " | OPT_FOCAL";
                ++count;
            }
            if (IsOptionWith(OPT_DISTO_PARAMS, curOption)) {
                stream << " | OPT_DISTO_PARAMS";
                ++count;
            }
            if (count == 0) {
                os << "NONE";
            } else if (count == 26) {
                os << "ALL";
            } else {
                std::string str = stream.str();
                if (str.at(1) == '|') {
                    str = str.substr(3, str.size() - 3);
                }
                os << str;
            }
            return os;
        };

    };

    class TrajectoryEstimator : protected ceres::Problem {
    public:
        using Ptr = std::shared_ptr<TrajectoryEstimator>;
        using SplineMeta = basalt::SplineMeta<CalibConfig::BSpline::SplineOrder>;

    protected:
        Trajectory::Ptr _trajectory;
        CalibParamManager::Ptr _calibParamManager;
        ceres::EigenQuaternionManifold *QUATER_MANIFOLD;
        ceres::SphereManifold<3> *S2_MANIFOLD;

        std::vector<CeresFactor> _factors;

    public:
        // using default problem options to create a 'TrajectoryEstimator'
        explicit TrajectoryEstimator(
                Trajectory::Ptr trajectory,
                CalibParamManager::Ptr calibParamManager,
                const ceres::Problem::Options &options = TrajectoryEstimator::DefaultProblemOptions()
        );

        static TrajectoryEstimator::Ptr Create(
                const Trajectory::Ptr &trajectory,
                const CalibParamManager::Ptr &calibParamManager,
                const ceres::Problem::Options &options = TrajectoryEstimator::DefaultProblemOptions()
        );

        ceres::Solver::Summary
        Solve(const ceres::Solver::Options &options = TrajectoryEstimator::DefaultSolverOptions(false));

        static ceres::Problem::Options DefaultProblemOptions();

        static ceres::Solver::Options DefaultSolverOptions(bool useCUDA = false);

        LMEquation Evaluate(const std::map<const double *, std::string> &targetParamsInfoMap);

        LMEquation Evaluate(const std::initializer_list<std::map<const double *, std::string>> &targetParamsInfoMaps);

    public:
        /**
         * different type optimize targets
         */

        void AddIMUGyroIdealMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &imuTopic,
                                        std::uint32_t option, double gyroWeight);

        void AddSO3Centralization(const std::vector<Sophus::SO3d *> &SO3_StoRef, double weight, std::uint32_t option,
                                  bool imuTrueLidarFalse = true);

        void AddPOSCentralization(const std::vector<Eigen::Vector3d *> &POS_SinRef, double weight,
                                  std::uint32_t option, bool imuTrueLidarFalse = true);

        void AddMultiLiDARRotCentralization(const Sophus::SO3d &SO3_LmToIr, const std::string &topic, double weight,
                                            std::uint32_t option);

        void AddTimeOffsetCentralization(const std::vector<double *> &TIME_OFFSET_StoRef, double weight,
                                         std::uint32_t option, bool imuTrueLidarFalse = true);

        void AddIMUMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &imuTopic,
                               std::uint32_t option, double gyroWeight, double acceWeight);

        void AddCentralLiDARRecoveryMeasurement(const std::string &lidarTopic, const Posed &CurLmToMm, double mapTime,
                                                double so3Weight, double posWeight, int option);

        void AddSE3Measurement(const Posed &ItoG, std::uint32_t option, double so3Weight, double posWeight);

        void AddSO3Measurement(const Posed &ItoG, std::uint32_t option, double so3Weight);

        void AddPOSMeasurement(const Posed &ItoG, std::uint32_t option, double posWeight);

        void AddPointPlaneCorrespondence(const PointPlaneCorrespondence &corr, const std::string &topic,
                                         std::uint32_t option, double pointPlaneWeight);

        void AddIMURecovery(
                double timestamp, double *gyro, double *acce,
                const Eigen::Vector3d &gravity, double gyroWeight, double acceWeight
        );

        void AddIMUMeasurement(
                const IMUFrame::Ptr &imuFrame, const Eigen::Vector3d &gravityInRef,
                std::uint32_t option, double gyroWeight, double acceWeight
        );

        void FixSO3ControlPointAt(int idx);

        void FixPOSControlPointAt(int idx);

        void FixFirActiveSO3ControlPoint();

        void FixFirActivePOSControlPoint();

    protected:
        enum class AddCtrlPointsDataFlag {
            SO3_KNOTS, POS_KNOTS
        };

        void AddCtrlPointsData(
                std::vector<double *> &paramBlockVec, AddCtrlPointsDataFlag flag,
                const basalt::SplineMeta<CalibConfig::BSpline::SplineOrder> &splineMeta,
                bool setToConst
        );

        template<typename CostFunctor, int Stride = 4>
        ceres::ResidualBlockId
        AddResidualBlockToProblem(ceres::DynamicAutoDiffCostFunction<CostFunctor, Stride> *costFunc,
                                  ceres::LossFunction *lossFunc, const std::vector<double *> &paramBlocks) {
            _factors.emplace_back(costFunc, paramBlocks, typeid(CostFunctor).hash_code());
            return Problem::AddResidualBlock(costFunc, lossFunc, paramBlocks);
        }

        static auto ExtractRange(const aligned_vector<IMUFrame::Ptr> &data, double st, double et) {
            auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > st;
            });
            auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < et;
            }).base();
            return std::pair(sIter, eIter);
        }
    };
}

#endif //LIC_CALIB_TRAJECTORY_ESTIMATOR_H
