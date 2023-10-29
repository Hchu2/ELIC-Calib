//
// Created by csl on 5/17/23.
//

#include "calib/calib_solver.h"
#include "functors/point_plane_functor.hpp"
#include "functors/imu_functor.hpp"
#include "viewer/aligned_map.h"
#include "opencv2/calib3d.hpp"
#include "viewer/surfel_map.h"

namespace ns_elic {

    CalibSolver::CalibSolver(CalibDataManager::Ptr calibDataManager, CalibParamManager::Ptr calibParamManager,
                             std::string sceneShotSaveDir)
            : _calibDataManager(std::move(calibDataManager)), _calibParamManager(std::move(calibParamManager)),
              _sceneShotSaveDir(std::move(sceneShotSaveDir)), _trajectory(nullptr)/*, _viewer(sceneShotSaveDir)*/ {
        // check config statue firstly, then init object
        CalibConfig::CheckConfigureStatus();
        LOG_INFO("ready to create the calibration solver...")

        // create the trajectory
        // the 1E-9 factor id just to stagger values
        _trajectory = Trajectory::Create(
                CalibConfig::BSpline::KnotTimeDistance,
                _calibDataManager->GetAlignedStartTimestamp() - 1E-9, _calibDataManager->GetAlignedEndTimestamp()
        );
        LOG_PLAINTEXT("create the trajectory for reference IMU B-spline, info:")
        LOG_PLAINTEXT("KnotTimeDistance: ", _trajectory->getDt())
        LOG_PLAINTEXT("minTime: ", _trajectory->minTime())
        LOG_PLAINTEXT("maxTime: ", _trajectory->maxTime())
        LOG_ENDL()
    }

    void CalibSolver::SaveEquationGraph(const TrajectoryEstimator::Ptr &estimator) {
        const static std::string dir = CalibConfig::CalibData::OutputData::OutputDataDir + "/lm_equ_graph";
        static int count = 0;
        if (count == 0) {
            std::filesystem::remove_all(dir);
            std::filesystem::create_directory(dir);
        }
        if (!std::filesystem::exists(dir)) {
            LOG_WARNING("the directory to save lm equation image is invalid!")
        } else {
            // save figure
            auto lmEquation = estimator->Evaluate(
                    {_calibParamManager->EXTRI.GetParamAddressWithDesc(),
                     _calibParamManager->TEMPORAL.GetParamAddressWithDesc(),
                     _calibParamManager->INTRI.GetParamAddressWithDesc()}
            );
            {
                // zero space
                std::ofstream file(dir + "/null_space.txt", std::ios::out | std::ios::app);
                file << lmEquation.ZeroSpace().transpose() << '\n';
                file.close();
            }

            {
                // equation
                std::string namePrefix = dir + "/batch_opt_" + std::to_string(count);
                cv::imwrite(namePrefix + ".png",
                            lmEquation.SaveEquationToDisk(namePrefix + ".json").EquationGraph());
            }

            {
                // residuals
                std::ofstream file(dir + "/residuals_" + std::to_string(count) + ".json", std::ios::out);
                std::map<std::size_t, std::string> idToName;
                idToName.insert({PointPlaneFunctor::TypeHashCode(), "PointPlaneFunctor"});
                idToName.insert({IMUFunctor::TypeHashCode(), "IMUFunctor"});

                std::map<std::string, std::vector<std::vector<double>>> data;

                for (const auto &[typeId, residuals]: lmEquation.GetResidualsMap()) {
                    if (idToName.find(typeId) == idToName.cend()) { continue; }
                    std::vector<std::vector<double>> valVec(residuals.size());
                    for (int i = 0; i < residuals.size(); ++i) {
                        valVec.at(i) = EigenVecXToVector(residuals.at(i));
                    }
                    data.insert({idToName.at(typeId), valVec});
                }
                cereal::JSONOutputArchive ar(file);
                ar(cereal::make_nvp("residuals", data));
            }

            ++count;
        }
    }

    const Trajectory::Ptr &CalibSolver::GetTrajectory() const {
        return _trajectory;
    }

    void CalibSolver::VisualizationSensors() {
        LOG_INFO("creating scene...")
        Viewer viewer(_sceneShotSaveDir, "Calib Result Sensors");
        viewer.RemoveEntities();
        _calibParamManager->VisualizationSensors(viewer);
        viewer.RunSingleThread();
    }

    void CalibSolver::VisualizationSceneHelper(const SurfelConstructor::Ptr &surfelConstructor, bool surfelMode) {
        LOG_INFO("creating scene...")
        Viewer viewer(_sceneShotSaveDir, "Calib Result Surfel Scene");

        if (surfelMode) {
            auto surfelMap = SurfelMap::Create(surfelConstructor->GetSurfelPlanes());
            auto dir = CalibConfig::CalibData::OutputData::OutputDataDir + "/maps/surfel_map";
            bool dirValid = true;
            if (!std::filesystem::exists(dir)) { dirValid = std::filesystem::create_directories(dir); }
            if (dirValid) {
                surfelMap->Save(dir + "/surfel_map.bin", true);
            }
            surfelMap->ShowSurfelMap(viewer);
        } else {
            PosTPointCloud::Ptr cloudOut(new PosTPointCloud);
            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                //RadiusOutlierRemoval
                LOG_PLAINTEXT("running 'RadiusOutlierRemoval' for lidar '", topic, "', this may take some time...")
                _calibDataManager->ComputeLidarUndistFramesInRef(_trajectory, _calibParamManager, topic);
                *cloudOut += *_calibDataManager->CreateLidarUndistMapFrameInRef(topic, false)->GetScan();
            }
            auto alignedMap = AlignedMap::Create(cloudOut, _calibParamManager->EXTRI.GRAVITY);
            auto dir = CalibConfig::CalibData::OutputData::OutputDataDir + "/maps/aligned_map";
            bool dirValid = true;
            if (!std::filesystem::exists(dir)) { dirValid = std::filesystem::create_directories(dir); }
            if (dirValid) {
                alignedMap->Save(dir + "/aligned_map.bin", true);
            }
            alignedMap->ShowAlignedMap(viewer);
        }

        LOG_PLAINTEXT("compute lidar, camera and imu trajectories...")
        aligned_vector<Posed> lidarTraj, refImuTraj;
        std::map<std::string, aligned_vector<Posed>> imuTraj;

        const auto &L0Topic = _calibDataManager->GetLidarRawFrames().cbegin()->first;
        for (const auto &rawLidarFrame: _calibDataManager->GetLidarRawFrames().at(L0Topic)) {
            const double timestamp = rawLidarFrame->GetTimestamp();

            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                if (auto curLtoRef = _trajectory->LiDARToRef(timestamp, topic, _calibParamManager)) {
                    lidarTraj.push_back(Posed::FromSE3(*curLtoRef));
                }
            }

            auto curIrToM = _trajectory->pose(rawLidarFrame->GetTimestamp());
            refImuTraj.push_back(Posed::FromSE3(curIrToM));

            for (const auto &imuTopic: CalibConfig::CalibData::Topic::IMUTopics) {
                auto curIjToM = curIrToM * _calibParamManager->EXTRI.SE3_IjToIr(imuTopic);
                imuTraj[imuTopic].push_back(Posed::FromSE3(curIjToM));
            }
        }
        LOG_PLAINTEXT("add trajectories to viewer...")
        viewer.ShowPoseSequence(aligned_vector<PoseSeqDisplay>{
                PoseSeqDisplay(refImuTraj, PoseSeqDisplay::Mode::IMU, ns_viewer::Colour::Black()),
                PoseSeqDisplay(lidarTraj, PoseSeqDisplay::Mode::LiDAR, ns_viewer::Colour::Blue())}, 0.1f);
        for (const auto &[topic, traj]: imuTraj) {
            viewer.ShowPoseSequence(aligned_vector<PoseSeqDisplay>{
                    PoseSeqDisplay(traj, PoseSeqDisplay::Mode::IMU, ns_viewer::Colour::Red())
            }, 0.1f);
        }

        viewer.RunSingleThread();
    }

    bool CalibSolver::SaveTrajectories(int hz) const {
        std::string saveDir = CalibConfig::CalibData::OutputData::OutputDataDir + "/trajectory";
        if (!std::filesystem::exists(saveDir)) {
            if (!std::filesystem::create_directories(saveDir)) {
                LOG_ERROR("directory to save trajectories dose not exist: ", saveDir, "...")
                return false;
            }
        }
        LOG_INFO("save trajectories to directory: ", saveDir, "...")

        // reference imu
        auto refPoseSeq = _trajectory->Sampling(1.0 / hz);
        {
            std::string refTrajPath = saveDir + "/ref_trajectory.json";
            std::ofstream file(refTrajPath, std::ios::out);
            cereal::JSONOutputArchive ar(file);
            ar(cereal::make_nvp("trajectory", refPoseSeq));
        }

        // imus
        {
            std::map<std::string, std::string> dirs;
            for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
                std::string dir = CalibConfig::CalibData::OutputData::OutputDataDir + "/trajectory/" + topic;
                if (!std::filesystem::exists(dir)) {
                    if (std::filesystem::create_directories(dir)) { dirs.insert({topic, dir}); }
                } else { dirs.insert({topic, dir}); }
            }
            for (const auto &[topic, dir]: dirs) {
                std::vector<Posed> poseVec;
                for (const auto &item: refPoseSeq) {
                    Sophus::SE3d SE3_IrToIr0(item.se3());
                    Sophus::SE3d SE3_IjToTr0 = SE3_IrToIr0 * _calibParamManager->EXTRI.SE3_IjToIr(topic);
                    poseVec.push_back(Posed::FromSE3(SE3_IjToTr0));
                }

                std::string trajPath = dir + "/trajectory.json";
                std::ofstream file(trajPath, std::ios::out);
                cereal::JSONOutputArchive ar(file);
                ar(cereal::make_nvp("trajectory", poseVec));
            }
        }

        // lidars
        {
            std::map<std::string, std::string> dirs;
            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                std::string dir = CalibConfig::CalibData::OutputData::OutputDataDir + "/trajectory/" + topic;
                if (!std::filesystem::exists(dir)) {
                    if (std::filesystem::create_directories(dir)) { dirs.insert({topic, dir}); }
                } else { dirs.insert({topic, dir}); }
            }
            for (const auto &[topic, dir]: dirs) {
                std::vector<Posed> poseVec;
                for (const auto &item: refPoseSeq) {
                    Sophus::SE3d SE3_IrToIr0(item.se3());
                    Sophus::SE3d SE3_LmToTr0 = SE3_IrToIr0 * _calibParamManager->EXTRI.SE3_LrToIr() *
                                               _calibParamManager->EXTRI.SE3_LmToLr(topic);
                    poseVec.push_back(Posed::FromSE3(SE3_LmToTr0));
                }

                std::string trajPath = dir + "/trajectory.json";
                std::ofstream file(trajPath, std::ios::out);
                cereal::JSONOutputArchive ar(file);
                ar(cereal::make_nvp("trajectory", poseVec));
            }
        }

        return true;
    }

}