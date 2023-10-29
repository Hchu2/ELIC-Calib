//
// Created by csl on 5/17/23.
//

#ifndef ELIC_CALIB_CALIB_SOLVER_H
#define ELIC_CALIB_CALIB_SOLVER_H

#include <utility>
#include "calib/calib_data_manager.h"
#include "calib/calib_param_manager.h"
#include "ceres/ceres.h"
#include "core/lidar_odometer.h"
#include "core/se3_spline.h"
#include "core/sfm_association.h"
#include "core/surfel_association.h"
#include "util/utils.hpp"
#include "viewer/pose_viewer.h"
#include "core/trajectory_estimator.h"
#include "core/ref_lidar_odometer.h"

namespace ns_elic {
    class CalibSolver {
    public:
        using Ptr = std::shared_ptr<CalibSolver>;

    protected:
        CalibDataManager::Ptr _calibDataManager;

        CalibParamManager::Ptr _calibParamManager;

        Trajectory::Ptr _trajectory;

        Viewer _viewer;

        const std::string _sceneShotSaveDir;

    public:
        explicit CalibSolver(
                CalibDataManager::Ptr calibDataManager, CalibParamManager::Ptr calibParamManager,
                std::string sceneShotSaveDir = ""
        );

        virtual void Process(std::uint32_t reprojectAssocOption) = 0;

        virtual void VisualizationScene(bool surfelMode) {}

        virtual void VisualizationColoredMap() {}

        void VisualizationSensors();

        void SaveEquationGraph(const TrajectoryEstimator::Ptr &estimator);

        [[nodiscard]] const Trajectory::Ptr &GetTrajectory() const;

        virtual bool SaveTrajectories(int hz = 100) const;

    protected:
        void VisualizationSceneHelper(const SurfelConstructor::Ptr &surfelConstructor, bool surfelMode);

    protected:
        virtual void Initialization() = 0;

        /**
         * the function to build the data association
         *
         * @param scanUndistortionOption
         * options: 'UNDIST_SO3', 'UNDIST_POS'
         *
         * @attention 'UNDIST_SO3 | UNDIST_POS' is not good for first time
         * the good choice is use 'UNDIST_SO3' option first, and then use 'UNDIST_SO3 | UNDIST_POS'
         *
         */
        virtual void DataAssociation(std::uint32_t scanUndistortionOption, std::uint32_t reprojectAssocOption) = 0;

        virtual void
        DataAssociationInRefine(std::uint32_t scanUndistortionOption, std::uint32_t reprojectAssocOption) = 0;

        /**
         * the function to do batch optimization
         *
         * @param pointPlaneOptOption
         * options: 'OPT_SO3', 'OPT_POS', 'OPT_SO3_LtoIr', 'OPT_POS_LinIr', 'OPT_TIME_OFFSET_LtoI'
         *
         * @param reprojectOptOption
         * options: 'OPT_SO3', 'OPT_POS', 'OPT_SO3_CtoIr', 'OPT_POS_CinIr', 'OPT_TIME_OFFSET_CtoI', 'OPT_STRUCTURE_SCALE', 'OPT_INV_DEPTH'
         *
         * @param imuMeasureOptOption
         * options: 'OPT_SO3', 'OPT_POS', 'OPT_GRAVITY_REFINE', 'OPT_ACCE_BIAS', 'OPT_GYRO_BIAS', 'OPT_ACCE_MAP_COEFF', 'OPT_GYRO_MAP_COEFF'
         */
        virtual void BatchOptimization(
                std::uint32_t pointPlaneOptOption, std::uint32_t reprojectOptOption,
                std::uint32_t imuMeasureOptOption, std::uint32_t reprojectAssocOption) = 0;

        virtual void Refinement(int iterationIndex, std::uint32_t reprojectAssocOption) = 0;

    };
}


#endif //ELIC_CALIB_CALIB_SOLVER_H
