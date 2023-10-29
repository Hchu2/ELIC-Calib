//
// Created by csl on 10/2/22.
//

#include "core/trajectory_estimator.h"
#include "config/calib_config.h"
#include "functors/imu_functor.hpp"
#include "functors/imu_gyro_functor.hpp"
#include "functors/point_plane_functor.hpp"
#include "functors/se3_functor.hpp"
#include "functors/imu_traj_recovery_functor.hpp"
#include "functors/centralization_functor.hpp"
#include "util/status.hpp"
#include "utility"
#include "functors/imu_recovery_functor.hpp"
#include "functors/velocity_functor.hpp"
#include "functors/relative_se3_functor.hpp"
#include "functors/central_lidar_recovery.hpp"

namespace ns_elic {

    TrajectoryEstimator::TrajectoryEstimator(Trajectory::Ptr trajectory, CalibParamManager::Ptr calibParamManager,
                                             const ceres::Problem::Options &options)
            : ceres::Problem(options), _trajectory(std::move(trajectory)),
              _calibParamManager(std::move(calibParamManager)), QUATER_MANIFOLD(new ceres::EigenQuaternionManifold()),
              S2_MANIFOLD(new ceres::SphereManifold<3>()) {}

    TrajectoryEstimator::Ptr TrajectoryEstimator::Create(const Trajectory::Ptr &trajectory,
                                                         const CalibParamManager::Ptr &calibParamManager,
                                                         const ceres::Problem::Options &options) {
        return std::make_shared<TrajectoryEstimator>(trajectory, calibParamManager, options);
    }

    ceres::Solver::Summary TrajectoryEstimator::Solve(const ceres::Solver::Options &options) {
        ceres::Solver::Summary summary;
        ceres::Solve(options, this, &summary);
        return summary;
    }

    ceres::Problem::Options TrajectoryEstimator::DefaultProblemOptions() {

        // organize the default problem options
        ceres::Problem::Options defaultProblemOptions;
        defaultProblemOptions.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        defaultProblemOptions.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

        return defaultProblemOptions;
    }

    ceres::Solver::Options TrajectoryEstimator::DefaultSolverOptions(bool useCUDA) {

        // organize the default solver option
        ceres::Solver::Options defaultSolverOptions;
        defaultSolverOptions.minimizer_type = ceres::TRUST_REGION;
        if (useCUDA) {
            defaultSolverOptions.linear_solver_type = ceres::DENSE_SCHUR;
            defaultSolverOptions.dense_linear_algebra_library_type = ceres::CUDA;
        } else {
            defaultSolverOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        }
        defaultSolverOptions.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        defaultSolverOptions.minimizer_progress_to_stdout = CalibConfig::Optimization::ProgressToStdout;
        defaultSolverOptions.num_threads = CalibConfig::Optimization::ThreadNum;
        defaultSolverOptions.max_num_iterations = CalibConfig::Optimization::CeresIterations;

        return defaultSolverOptions;
    }

    void TrajectoryEstimator::AddCtrlPointsData(std::vector<double *> &paramBlockVec,
                                                TrajectoryEstimator::AddCtrlPointsDataFlag flag,
                                                const basalt::SplineMeta<CalibConfig::BSpline::SplineOrder> &splineMeta,
                                                bool setToConst) {

        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor '1E-9' is the treatment of numerical accuracy
            auto idxMaster = _trajectory->computeTIndex(seg.t0 + 1E-9).second;

            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                // switch
                double *data;
                switch (flag) {
                    case AddCtrlPointsDataFlag::SO3_KNOTS:
                        data = _trajectory->getKnotSO3(i).data();
                        // the local parameterization is very very important!!!
                        // TODO: what is this setting going to do
                        this->AddParameterBlock(data, 4, QUATER_MANIFOLD);
                        break;
                    case AddCtrlPointsDataFlag::POS_KNOTS:
                        data = _trajectory->getKnotPos(i).data();
                        this->AddParameterBlock(data, 3);
                        break;
                    default:
                        throw Status(
                                Status::Flag::FETAL,
                                "add unknown control points in the function 'TrajectoryEstimator::AddCtrlPointsData'"
                        );
                }
                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) {
                    this->SetParameterBlockConstant(data);
                }
            }
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | POS | ... | POS | GYRO_BIAS | ACCE_BIAS | G_REFINE | GYRO_MAP_COEFF | ACCE_MAP_COEFF | SO3_AtoG ]
     * [ SO3_IjToIr | POS_IjInIr | TIME_OFFSET_IjToIr ]
     */
    void TrajectoryEstimator::AddIMUMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &imuTopic,
                                                std::uint32_t option, double gyroWeight, double acceWeight) {
        SplineMeta splineMeta;

        // different relative control points finding [single vs. range]
        if (OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_IjToIr, option)) {
            double frameIMUTimeMin = imuFrame->GetTimestamp() - CalibConfig::Optimization::TimeOffsetPadding;
            double frameIMUTimeMax = imuFrame->GetTimestamp() + CalibConfig::Optimization::TimeOffsetPadding;
            // invalid time stamp
            if (!_trajectory->TimeStampInRange(frameIMUTimeMin) || !_trajectory->TimeStampInRange(frameIMUTimeMax)) {
                return;
            }
            _trajectory->CalculateSplineMeta({{frameIMUTimeMin, frameIMUTimeMax}}, splineMeta);
        } else {
            double frameIMUTime =
                    imuFrame->GetTimestamp() + _calibParamManager->TEMPORAL.TIME_OFFSET_IjToIr.at(imuTopic);

            // check point time stamp
            if (!_trajectory->TimeStampInRange(frameIMUTime)) {
                return;
            }

            _trajectory->CalculateSplineMeta({{frameIMUTime, frameIMUTime}}, splineMeta);
        }

        auto costFunc = IMUFunctor::Create(splineMeta, imuFrame, gyroWeight, acceWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < splineMeta.NumParameters(); i++) {
            costFunc->AddParameterBlock(4);
        }
        // pos knots param block [each has three sub params]
        for (int i = 0; i < splineMeta.NumParameters(); i++) {
            costFunc->AddParameterBlock(3);
        }
        // gyro bias
        costFunc->AddParameterBlock(3);
        // acce bias
        costFunc->AddParameterBlock(3);
        // gravity refine
        costFunc->AddParameterBlock(3);
        // gyro map coeff
        costFunc->AddParameterBlock(6);
        // acce map coeff
        costFunc->AddParameterBlock(6);
        // imu rotational misalignment
        costFunc->AddParameterBlock(4);
        // SO3_IjToIr
        costFunc->AddParameterBlock(4);
        // POS_IjInIr
        costFunc->AddParameterBlock(3);
        // TIME_OFFSET_IjToIr
        costFunc->AddParameterBlock(1);

        costFunc->SetNumResiduals(6);

        std::vector<double *> paramBlockVec;

        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS, splineMeta,
                !OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3, option)
        );

        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS, splineMeta,
                !OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS, option)
        );
        auto &IMU = _calibParamManager->INTRI.IMU.at(imuTopic);
        auto gyroBias = IMU.GYRO.BIAS.data();
        auto acceBias = IMU.ACCE.BIAS.data();
        auto gRefine = _calibParamManager->EXTRI.GRAVITY.data();
        auto gyroCoeff = IMU.GYRO.MAP_COEFF.data();
        auto acceCoeff = IMU.ACCE.MAP_COEFF.data();
        auto SO3_AtoG = IMU.SO3_AtoG.data();
        auto SO3_IjToIr = _calibParamManager->EXTRI.SO3_IjToIr.at(imuTopic).data();
        auto POS_IjInIr = _calibParamManager->EXTRI.POS_IjInIr.at(imuTopic).data();
        auto TIME_OFFSET_IjToIr = &(_calibParamManager->TEMPORAL.TIME_OFFSET_IjToIr.at(imuTopic));

        paramBlockVec.push_back(gyroBias);
        paramBlockVec.push_back(acceBias);
        paramBlockVec.push_back(gRefine);

        paramBlockVec.emplace_back(gyroCoeff);
        paramBlockVec.emplace_back(acceCoeff);
        paramBlockVec.emplace_back(SO3_AtoG);

        paramBlockVec.emplace_back(SO3_IjToIr);
        paramBlockVec.emplace_back(POS_IjInIr);
        paramBlockVec.emplace_back(TIME_OFFSET_IjToIr);

        this->AddParameterBlock(SO3_AtoG, 4, QUATER_MANIFOLD);
        this->AddParameterBlock(SO3_IjToIr, 4, QUATER_MANIFOLD);
        this->AddParameterBlock(gRefine, 3, S2_MANIFOLD);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);

        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_GYRO_BIAS, option)) {
            this->SetParameterBlockConstant(gyroBias);
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_ACCE_BIAS, option)) {
            this->SetParameterBlockConstant(acceBias);
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_GRAVITY_REFINE, option)) {
            this->SetParameterBlockConstant(gRefine);
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_GYRO_MAP_COEFF, option)) {
            this->SetParameterBlockConstant(gyroCoeff);
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_ACCE_MAP_COEFF, option)) {
            this->SetParameterBlockConstant(acceCoeff);
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_AtoG, option)) {
            this->SetParameterBlockConstant(SO3_AtoG);
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_IjToIr, option)) {
            this->SetParameterBlockConstant(SO3_IjToIr);
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS_IjInIr, option)) {
            this->SetParameterBlockConstant(POS_IjInIr);
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_IjToIr, option)) {
            this->SetParameterBlockConstant(TIME_OFFSET_IjToIr);
        } else {
            // set bound
            this->SetParameterLowerBound(TIME_OFFSET_IjToIr, 0, -CalibConfig::Optimization::TimeOffsetPadding);
            this->SetParameterUpperBound(TIME_OFFSET_IjToIr, 0, CalibConfig::Optimization::TimeOffsetPadding);
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | POS | ... | POS ]
     */
    void TrajectoryEstimator::AddSE3Measurement(const Posed &ItoG, std::uint32_t option, double so3Weight,
                                                double posWeight) {
        // check time stamp
        if (!_trajectory->TimeStampInRange(ItoG.timeStamp)) {
            return;
        }

        // find the affected control points
        SplineMeta splineMeta;
        _trajectory->CalculateSplineMeta({{ItoG.timeStamp, ItoG.timeStamp}}, splineMeta);

        // create a cost function
        auto costFunc = SE3Functor::Create(splineMeta, ItoG, so3Weight, posWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // pos knots param block [each has three sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(3);
        }
        // set Residuals
        costFunc->SetNumResiduals(6);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS,
                splineMeta, !OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3, option)
        );

        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS,
                splineMeta, !OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS, option)
        );

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | POS | ... | POS | SO3_LmToLr | POS_LmInLr | SO3_LrToIr | POS_LrInIr | TIME_OFFSET_LmToLr | TIME_OFFSET_LrToIr ]
     */
    void
    TrajectoryEstimator::AddPointPlaneCorrespondence(const PointPlaneCorrespondence &corr, const std::string &topic,
                                                     std::uint32_t option, double pointPlaneWeight) {
        // find the affected control points
        SplineMeta splineMeta;

        // different relative control points finding [single vs. range]
        if (OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_LmToLr, option)) {
            double ptIMUTimeMin, ptIMUTimeMax;
            if (OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_LrToIr, option)) {
                ptIMUTimeMin = corr.pointTimeStamp - 2.0 * CalibConfig::Optimization::TimeOffsetPadding;
                ptIMUTimeMax = corr.pointTimeStamp + 2.0 * CalibConfig::Optimization::TimeOffsetPadding;
            } else {
                ptIMUTimeMin = corr.pointTimeStamp + _calibParamManager->TEMPORAL.TIME_OFFSET_LrToIr -
                               CalibConfig::Optimization::TimeOffsetPadding;
                ptIMUTimeMax = corr.pointTimeStamp + _calibParamManager->TEMPORAL.TIME_OFFSET_LrToIr +
                               CalibConfig::Optimization::TimeOffsetPadding;
            }

            // invalid time stamp
            if (!_trajectory->TimeStampInRange(ptIMUTimeMin) || !_trajectory->TimeStampInRange(ptIMUTimeMax)) {
                return;
            }
            _trajectory->CalculateSplineMeta({{ptIMUTimeMin, ptIMUTimeMax}}, splineMeta);
        } else {
            if (OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_LrToIr, option)) {
                double ptIMUTimeMin = corr.pointTimeStamp + _calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr.at(topic) -
                                      CalibConfig::Optimization::TimeOffsetPadding;
                double ptIMUTimeMax = corr.pointTimeStamp + _calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr.at(topic) +
                                      CalibConfig::Optimization::TimeOffsetPadding;
                // invalid time stamp
                if (!_trajectory->TimeStampInRange(ptIMUTimeMin) || !_trajectory->TimeStampInRange(ptIMUTimeMax)) {
                    return;
                }
                _trajectory->CalculateSplineMeta({{ptIMUTimeMin, ptIMUTimeMax}}, splineMeta);
            } else {
                double ptIMUTime = corr.pointTimeStamp + _calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr.at(topic) +
                                   _calibParamManager->TEMPORAL.TIME_OFFSET_LrToIr;

                // check point time stamp
                if (!_trajectory->TimeStampInRange(ptIMUTime)) {
                    return;
                }

                _trajectory->CalculateSplineMeta({{ptIMUTime, ptIMUTime}}, splineMeta);
            }
        }

        // create a cost function
        auto costFunc = PointPlaneFunctor::Create(splineMeta, corr, pointPlaneWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // pos knots param block [each has three sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(3);
        }
        costFunc->AddParameterBlock(4); // SO3_LmToLr
        costFunc->AddParameterBlock(3); // POS_LmInLr
        costFunc->AddParameterBlock(4); // SO3_LrToIr
        costFunc->AddParameterBlock(3); // POS_LrInIr
        costFunc->AddParameterBlock(1); // TIME_OFFSET_LmToLr
        costFunc->AddParameterBlock(1); // TIME_OFFSET_LrToIr

        // the Residual: point to plane distance
        costFunc->SetNumResiduals(1);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddCtrlPointsData(paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS,
                          splineMeta, !OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3, option));

        AddCtrlPointsData(paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS,
                          splineMeta, !OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS, option));

        paramBlockVec.push_back(_calibParamManager->EXTRI.SO3_LmToLr.at(topic).data());
        // important!!!
        this->AddParameterBlock(_calibParamManager->EXTRI.SO3_LmToLr.at(topic).data(), 4, QUATER_MANIFOLD);

        paramBlockVec.push_back(_calibParamManager->EXTRI.POS_LmInLr.at(topic).data());

        paramBlockVec.push_back(_calibParamManager->EXTRI.SO3_LrToIr.data());
        // important!!!
        this->AddParameterBlock(_calibParamManager->EXTRI.SO3_LrToIr.data(), 4, QUATER_MANIFOLD);

        paramBlockVec.push_back(_calibParamManager->EXTRI.POS_LrInIr.data());

        paramBlockVec.push_back(&_calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr.at(topic));
        paramBlockVec.push_back(&_calibParamManager->TEMPORAL.TIME_OFFSET_LrToIr);

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, new ceres::CauchyLoss(1.0), paramBlockVec);

        // lock param or not
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_LmToLr, option)) {
            this->SetParameterBlockConstant(_calibParamManager->EXTRI.SO3_LmToLr.at(topic).data());
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS_LmInLr, option)) {
            this->SetParameterBlockConstant(_calibParamManager->EXTRI.POS_LmInLr.at(topic).data());
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_LrToIr, option)) {
            this->SetParameterBlockConstant(_calibParamManager->EXTRI.SO3_LrToIr.data());
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS_LrInIr, option)) {
            this->SetParameterBlockConstant(_calibParamManager->EXTRI.POS_LrInIr.data());
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_LmToLr, option)) {
            this->SetParameterBlockConstant(&_calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr.at(topic));
        } else {
            // set bound
            this->SetParameterLowerBound(
                    &_calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr.at(topic), 0,
                    -CalibConfig::Optimization::TimeOffsetPadding
            );
            this->SetParameterUpperBound(
                    &_calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr.at(topic), 0,
                    CalibConfig::Optimization::TimeOffsetPadding
            );
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_LrToIr, option)) {
            this->SetParameterBlockConstant(&_calibParamManager->TEMPORAL.TIME_OFFSET_LrToIr);
        } else {
            // set bound
            this->SetParameterLowerBound(
                    &_calibParamManager->TEMPORAL.TIME_OFFSET_LrToIr, 0, -CalibConfig::Optimization::TimeOffsetPadding
            );
            this->SetParameterUpperBound(
                    &_calibParamManager->TEMPORAL.TIME_OFFSET_LrToIr, 0, CalibConfig::Optimization::TimeOffsetPadding
            );
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | SO3_IjToIr ]
     */
    void TrajectoryEstimator::AddIMUGyroIdealMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &imuTopic,
                                                         std::uint32_t option, double gyroWeight) {
        // check time stamp
        if (!_trajectory->TimeStampInRange(imuFrame->GetTimestamp())) {
            return;
        }

        // find the affected control points
        SplineMeta splineMeta;
        _trajectory->CalculateSplineMeta({{imuFrame->GetTimestamp(), imuFrame->GetTimestamp()}}, splineMeta);

        // create a cost function
        auto costFunc = IMUGyroIdealFunctor::Create(splineMeta, imuFrame, gyroWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(4);
        }

        // SO3_IjToIr
        costFunc->AddParameterBlock(4);

        // set Residuals
        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS, splineMeta,
                !OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3, option)
        );
        auto SO3_IjToIr = _calibParamManager->EXTRI.SO3_IjToIr.at(imuTopic).data();
        paramBlockVec.push_back(SO3_IjToIr);
        this->AddParameterBlock(SO3_IjToIr, 4, QUATER_MANIFOLD);

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 ]
     */
    void TrajectoryEstimator::AddSO3Measurement(const Posed &ItoG, std::uint32_t option, double so3Weight) {
        // check time stamp
        if (!_trajectory->TimeStampInRange(ItoG.timeStamp)) {
            return;
        }

        // find the affected control points
        SplineMeta splineMeta;
        _trajectory->CalculateSplineMeta({{ItoG.timeStamp, ItoG.timeStamp}}, splineMeta);

        // create a cost function
        auto costFunc = SO3Functor::Create(splineMeta, ItoG, so3Weight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // set Residuals
        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS, splineMeta,
                !OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3, option)
        );

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
    }

    /**
     * param blocks:
     * [ POS | ... | POS ]
     */
    void TrajectoryEstimator::AddPOSMeasurement(const Posed &ItoG, std::uint32_t option, double posWeight) {
        // check time stamp
        if (!_trajectory->TimeStampInRange(ItoG.timeStamp)) {
            return;
        }

        // find the affected control points
        SplineMeta splineMeta;
        _trajectory->CalculateSplineMeta({{ItoG.timeStamp, ItoG.timeStamp}}, splineMeta);

        // create a cost function
        auto costFunc = PO3Functor::Create(splineMeta, ItoG, posWeight);

        // pos knots param block [each has three sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(3);
        }
        // set Residuals
        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS,
                splineMeta, !OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS, option)
        );

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | POS | ... | POS | GYRO | ACCE ]
     */
    void TrajectoryEstimator::AddIMURecovery(double timestamp, double *gyro, double *acce,
                                             const Eigen::Vector3d &gravity, double gyroWeight, double acceWeight) {
        // check time stamp
        if (!_trajectory->TimeStampInRange(timestamp)) {
            return;
        }

        SplineMeta splineMeta;
        _trajectory->CalculateSplineMeta({{timestamp, timestamp}}, splineMeta);

        auto costFunc = IMURecoveryFunctor::Create(splineMeta, timestamp, gravity, gyroWeight, acceWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < splineMeta.NumParameters(); i++) {
            costFunc->AddParameterBlock(4);
        }
        // pos knots param block [each has three sub params]
        for (int i = 0; i < splineMeta.NumParameters(); i++) {
            costFunc->AddParameterBlock(3);
        }
        // gyro
        costFunc->AddParameterBlock(3);
        // acce
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(6);

        std::vector<double *> paramBlockVec;

        AddCtrlPointsData(paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS, splineMeta, true);

        AddCtrlPointsData(paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS, splineMeta, true);

        paramBlockVec.push_back(gyro);
        paramBlockVec.push_back(acce);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
    }

    LMEquation TrajectoryEstimator::Evaluate(const std::map<const double *, std::string> &targetParamsInfoMapRaw) {
        std::map<const double *, std::string> targetParamsInfoMap;
        for (const auto &[key, value]: targetParamsInfoMapRaw) {
            if (this->HasParameterBlock(key)) { targetParamsInfoMap.insert({key, value}); }
        }

        std::size_t rows = 0, cols = 0;
        // param address, param dime, param start column
        std::map<const double *, std::pair<std::size_t, std::size_t>> paramDimeInfo;
        for (auto &factor: _factors) {
            factor.Evaluate(targetParamsInfoMap);
            // record size info
            rows += factor.GetResiduals().size();
            for (const auto &[pAddress, pJacobian]: factor.GetJacobians()) {
                // using map assign operator, if the 'pAddress' not exists, a new element will be created,
                // otherwise, the value of the key will be updated
                paramDimeInfo[pAddress] = {pJacobian.cols(), 0};
            }
        }

        // desc, address, handle the param ordered by the desc
        std::map<std::string, const double *> invTargetParamsInfoMap;
        for (const auto &[key, value]: targetParamsInfoMap) { invTargetParamsInfoMap.insert({value, key}); }

        // desc, dime
        std::vector<std::pair<std::string, std::size_t>> paramDesc;
        for (auto &[desc, pAddress]: invTargetParamsInfoMap) {
            auto &column = paramDimeInfo.at(pAddress);
            column.second = cols;
            cols += column.first;
            paramDesc.emplace_back(desc, column.first);
        }

        Eigen::MatrixXd jMat(rows, cols), hMat(cols, cols);
        Eigen::VectorXd rVec(rows), bVec(cols);
        jMat.setZero(), rVec.setZero();

        // factor type id, residuals
        std::map<std::size_t, aligned_vector<Eigen::VectorXd>> residualsMap;
        // organize the jMat and rVec
        int cr = 0;
        for (const auto &factor: _factors) {
            const auto &jacobians = factor.GetJacobians();
            const auto &residuals = factor.GetResiduals();
            // residuals vector
            rVec.block(cr, 0, residuals.rows(), 1) = residuals;

            // jacobians matrix
            for (const auto &[pAddress, pJacobian]: jacobians) {
                jMat.block(
                        cr, static_cast<int>(paramDimeInfo.find(pAddress)->second.second),
                        pJacobian.rows(), pJacobian.cols()
                ) = pJacobian;
            }

            // draw error
            residualsMap[factor.GetCostFunctorHashCode()].push_back(residuals);

            cr += static_cast<int>(residuals.rows());
        }
        hMat = jMat.transpose() * jMat;
        bVec = -jMat.transpose() * rVec;

        return {hMat, bVec, paramDesc, residualsMap};
    }

    LMEquation
    TrajectoryEstimator::Evaluate(
            const std::initializer_list<std::map<const double *, std::string>> &targetParamsInfoMaps) {

        std::map<const double *, std::string> targetParamsInfoMap;
        for (const auto &item: targetParamsInfoMaps) {
            for (const auto &p: item) {
                targetParamsInfoMap.insert(p);
            }
        }

        return Evaluate(targetParamsInfoMap);
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | POS | ... | POS ]
     */
    void TrajectoryEstimator::AddIMUMeasurement(const IMUFrame::Ptr &imuFrame, const Eigen::Vector3d &gravityInRef,
                                                std::uint32_t option, double gyroWeight, double acceWeight) {
        // check time stamp
        if (!_trajectory->TimeStampInRange(imuFrame->GetTimestamp())) {
            return;
        }

        SplineMeta splineMeta;
        _trajectory->CalculateSplineMeta({{imuFrame->GetTimestamp(), imuFrame->GetTimestamp()}}, splineMeta);

        auto costFunc = IMUTrajRecoveryFunctor::Create(splineMeta, imuFrame, gravityInRef, gyroWeight, acceWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < splineMeta.NumParameters(); i++) {
            costFunc->AddParameterBlock(4);
        }
        // pos knots param block [each has three sub params]
        for (int i = 0; i < splineMeta.NumParameters(); i++) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(6);

        std::vector<double *> paramBlockVec;

        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS, splineMeta,
                !OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3, option)
        );

        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS, splineMeta,
                !OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS, option)
        );

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
    }

    void TrajectoryEstimator::AddSO3Centralization(const std::vector<Sophus::SO3d *> &SO3_StoRef,
                                                   double weight, std::uint32_t option, bool imuTrueLidarFalse) {
        auto costFunc = SO3CentralizationFunctor::Create(SO3_StoRef.size(), weight);

        std::vector<double *> paramBlockVec;

        for (auto &item: SO3_StoRef) {
            // SO3_StoRef
            costFunc->AddParameterBlock(4);
            paramBlockVec.emplace_back(item->data());
            this->AddParameterBlock(item->data(), 4, QUATER_MANIFOLD);
        }
        costFunc->SetNumResiduals(3);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        if (imuTrueLidarFalse) {
            if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_IjToIr, option)) {
                for (auto &item: SO3_StoRef) { this->SetParameterBlockConstant(item->data()); }
            }
        } else {
            if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_LmToLr, option)) {
                for (auto &item: SO3_StoRef) { this->SetParameterBlockConstant(item->data()); }
            }
        }

    }

    void TrajectoryEstimator::AddPOSCentralization(const std::vector<Eigen::Vector3d *> &POS_SinRef,
                                                   double weight, std::uint32_t option, bool imuTrueLidarFalse) {
        auto costFunc = POSCentralizationFunctor::Create(POS_SinRef.size(), weight);

        std::vector<double *> paramBlockVec;

        for (auto &item: POS_SinRef) {
            // POS_SinRef
            costFunc->AddParameterBlock(3);
            paramBlockVec.emplace_back(item->data());
        }
        costFunc->SetNumResiduals(3);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        if (imuTrueLidarFalse) {
            if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS_IjInIr, option)) {
                for (auto &item: POS_SinRef) { this->SetParameterBlockConstant(item->data()); }
            }
        } else {
            if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS_LmInLr, option)) {
                for (auto &item: POS_SinRef) { this->SetParameterBlockConstant(item->data()); }
            }
        }
    }

    void TrajectoryEstimator::AddTimeOffsetCentralization(const std::vector<double *> &TIME_OFFSET_StoRef,
                                                          double weight, std::uint32_t option, bool imuTrueLidarFalse) {
        auto costFunc = TimeOffsetCentralizationFunctor::Create(TIME_OFFSET_StoRef.size(), weight);

        std::vector<double *> paramBlockVec;

        for (auto &item: TIME_OFFSET_StoRef) {
            // TIME_OFFSET_StoRef
            costFunc->AddParameterBlock(1);
            paramBlockVec.emplace_back(item);
        }
        costFunc->SetNumResiduals(1);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);

        if (imuTrueLidarFalse) {
            if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_IjToIr, option)) {
                for (auto &item: TIME_OFFSET_StoRef) { this->SetParameterBlockConstant(item); }
            } else {
                // set bound
                for (auto &item: TIME_OFFSET_StoRef) {
                    this->SetParameterLowerBound(item, 0, -CalibConfig::Optimization::TimeOffsetPadding);
                    this->SetParameterUpperBound(item, 0, CalibConfig::Optimization::TimeOffsetPadding);
                }
            }
        } else {
            if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_TIME_OFFSET_LmToLr, option)) {
                for (auto &item: TIME_OFFSET_StoRef) { this->SetParameterBlockConstant(item); }
            } else {
                // set bound
                for (auto &item: TIME_OFFSET_StoRef) {
                    this->SetParameterLowerBound(item, 0, -CalibConfig::Optimization::TimeOffsetPadding);
                    this->SetParameterUpperBound(item, 0, CalibConfig::Optimization::TimeOffsetPadding);
                }
            }
        }
    }

    /**
     * param blocks:
     * [ SO3_LmToLr | SO3_LrToIr ]
     */
    void TrajectoryEstimator::AddMultiLiDARRotCentralization(const Sophus::SO3d &SO3_LmToIr, const std::string &topic,
                                                             double weight, std::uint32_t option) {
        auto costFunc = MultiLiDARRotCentralizationFunctor::Create(SO3_LmToIr, weight);
        costFunc->AddParameterBlock(4);
        costFunc->AddParameterBlock(4);

        std::vector<double *> paramBlockVec;
        paramBlockVec.push_back(_calibParamManager->EXTRI.SO3_LmToLr.at(topic).data());
        paramBlockVec.push_back(_calibParamManager->EXTRI.SO3_LrToIr.data());

        this->AddParameterBlock(_calibParamManager->EXTRI.SO3_LmToLr.at(topic).data(), 4, QUATER_MANIFOLD);
        this->AddParameterBlock(_calibParamManager->EXTRI.SO3_LrToIr.data(), 4, QUATER_MANIFOLD);

        costFunc->SetNumResiduals(3);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);

        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_LmToLr, option)) {
            this->SetParameterBlockConstant(_calibParamManager->EXTRI.SO3_LmToLr.at(topic).data());
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_LrToIr, option)) {
            this->SetParameterBlockConstant(_calibParamManager->EXTRI.SO3_LrToIr.data());
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | POS | ... | POS |SO3_LmToLr | POS_LmInLr ]
     */
    void TrajectoryEstimator::AddCentralLiDARRecoveryMeasurement(const std::string &lidarTopic, const Posed &CurLmToMm,
                                                                 double mapTime, double so3Weight, double posWeight,
                                                                 int option) {
        if (!_trajectory->TimeStampInRange(CurLmToMm.timeStamp) || !_trajectory->TimeStampInRange(mapTime)) {
            return;
        }
        // find the affected control points
        SplineMeta splineMeta;
        if (CurLmToMm.timeStamp < mapTime) {
            _trajectory->CalculateSplineMeta({{CurLmToMm.timeStamp, CurLmToMm.timeStamp},
                                              {mapTime,             mapTime}}, splineMeta);
        } else {
            _trajectory->CalculateSplineMeta({{mapTime,             mapTime},
                                              {CurLmToMm.timeStamp, CurLmToMm.timeStamp}}, splineMeta);
        }
        auto costFunc = CentralLiDARRecoveryFunctor::Create(splineMeta, CurLmToMm, mapTime, so3Weight, posWeight);
        // so3 knots param block [each has four sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // pos knots param block [each has three sub params]
        for (int i = 0; i < splineMeta.NumParameters(); ++i) {
            costFunc->AddParameterBlock(3);
        }
        // SO3_LmToLr
        costFunc->AddParameterBlock(4);
        // POS_LmInLr
        costFunc->AddParameterBlock(3);

        // set Residuals
        costFunc->SetNumResiduals(6);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS,
                splineMeta, !OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3, option)
        );

        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS,
                splineMeta, !OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS, option)
        );
        paramBlockVec.push_back(_calibParamManager->EXTRI.SO3_LmToLr.at(lidarTopic).data());
        paramBlockVec.push_back(_calibParamManager->EXTRI.POS_LmInLr.at(lidarTopic).data());

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);

        this->SetManifold(_calibParamManager->EXTRI.SO3_LmToLr.at(lidarTopic).data(), QUATER_MANIFOLD);

        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_SO3_LmToLr, option)) {
            this->SetParameterBlockConstant(_calibParamManager->EXTRI.SO3_LmToLr.at(lidarTopic).data());
        }
        if (!OptimizationOption::IsOptionWith(OptimizationOption::OPT_POS_LmInLr, option)) {
            this->SetParameterBlockConstant(_calibParamManager->EXTRI.POS_LmInLr.at(lidarTopic).data());
        }
    }

    void TrajectoryEstimator::FixSO3ControlPointAt(int idx) {
        auto data = _trajectory->getSo3Spline().getKnot(idx).data();
        this->AddParameterBlock(const_cast<double *>(data), 4, QUATER_MANIFOLD);
        this->SetParameterBlockConstant(data);
    }

    void TrajectoryEstimator::FixPOSControlPointAt(int idx) {
        auto data = _trajectory->getPosSpline().getKnot(idx).data();
        this->AddParameterBlock(const_cast<double *>(data), 3);
        this->SetParameterBlockConstant(_trajectory->getPosSpline().getKnot(idx).data());
    }

    void TrajectoryEstimator::FixFirActiveSO3ControlPoint() {
        for (const auto &knot: _trajectory->getSo3Spline().getKnots()) {
            if (this->HasParameterBlock(knot.data())) {
                this->SetParameterBlockConstant(knot.data());
                break;
            }
        }
    }

    void TrajectoryEstimator::FixFirActivePOSControlPoint() {
        for (const auto &knot: _trajectory->getPosSpline().getKnots()) {
            if (this->HasParameterBlock(knot.data())) {
                this->SetParameterBlockConstant(knot.data());
                break;
            }
        }
    }

}
