//
// Created by csl on 10/1/22.
//

#ifndef LIC_CALIB_LIS_CALIB_SOLVER_H
#define LIC_CALIB_LIS_CALIB_SOLVER_H

#include "calib/calib_solver.h"

namespace ns_elic {

    struct LIsCeresDebugCallBack : public ceres::IterationCallback {
        CalibParamManager::Ptr _calibParamManager;

        explicit LIsCeresDebugCallBack(CalibParamManager::Ptr calibParamManager)
                : _calibParamManager(std::move(calibParamManager)) {}

        static auto Create(const CalibParamManager::Ptr &calibParamManager) {
            return new LIsCeresDebugCallBack(calibParamManager);
        }

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {
            // for drawing
            const static std::string paramDir = CalibConfig::CalibData::OutputData::OutputDataDir + "/params_iter";
            const std::string iterInfoFilename = paramDir + "/iter_info.csv";

            static int count = 0;
            if (count == 0) {
                std::filesystem::remove_all(paramDir);
                std::filesystem::create_directory(paramDir);

                std::ofstream file(iterInfoFilename, std::ios::out);
                file << "cost,gradient,tr_radius(1/lambda)" << std::endl;
                file.close();
            }
            if (!std::filesystem::exists(paramDir)) {
                LOG_WARNING("the directory to save param files is invalid!")
            } else {
                // save param
                const std::string paramFilename = paramDir + "/params_" + std::to_string(count) + ".json";
                _calibParamManager->Save(paramFilename);

                // save iter info
                std::ofstream file(iterInfoFilename, std::ios::app);
                file << count << ',' << summary.cost << ','
                     << summary.gradient_norm << ',' << summary.trust_region_radius << std::endl;
                file.close();

                ++count;
            }
            return ceres::SOLVER_CONTINUE;
        }

    };

    class LIsCalibSolver : public CalibSolver {
    public:
        using Ptr = std::shared_ptr<LIsCalibSolver>;

    private:
        aligned_map<std::string, LiDAROdometer::Ptr> _lidarOdometer;

        SurfelConstructor::Ptr _surfelConstructor;

        aligned_map<std::string, SurfelAssociation::Ptr> _surfelAssociation;

        RefLiDAROdometer::Ptr _refLiDarOdometer;

    public:
        explicit LIsCalibSolver(const CalibDataManager::Ptr &calibDataManager,
                                const CalibParamManager::Ptr &calibParamManager,
                                const std::string &sceneShotSaveDir = "");

        static LIsCalibSolver::Ptr Create(
                const CalibDataManager::Ptr &calibDataManager, const CalibParamManager::Ptr &calibParamManager,
                const std::string &sceneShotSaveDir = ""
        );

        void Process(std::uint32_t reprojectAssocOption) override;

        void VisualizationScene(bool surfelMode) override;

    protected:
        void Initialization() override;

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
        void DataAssociation(std::uint32_t scanUndistortionOption, std::uint32_t reprojectAssocOption) override;

        void DataAssociationInRefine(std::uint32_t scanUndistortionOption, std::uint32_t reprojectAssocOption) override;

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
        void BatchOptimization(
                std::uint32_t pointPlaneOptOption, std::uint32_t reprojectOptOption,
                std::uint32_t imuMeasureOptOption, std::uint32_t reprojectAssocOption) override;

        void Refinement(int iterationIndex, std::uint32_t reprojectAssocOption) override;
    };
}


#endif //LIC_CALIB_LIS_CALIB_SOLVER_H
