//
// Created by csl on 2/16/23.
//

#include "core/ref_lidar_odometer.h"
#include "core/trajectory_estimator.h"
#include <utility>

namespace ns_elic {

    LiDAROdometerPack::LiDAROdometerPack(const aligned_vector<Posed> &poseSeq, LiDARFrame::Ptr lidarMap,
                                         const Sophus::SE3d &lmToLr)
            : poseSeq(poseSeq), lidarMap(std::move(lidarMap)), LmToLr(lmToLr) {}

    void RefLiDAROdometer::Initialization(const aligned_vector<LiDAROdometerPack> &lidarPacks, double minTime,
                                          double maxTime) {
        // trajectory
        LOG_INFO("ready to create the reference lidar trajectory...")
        _trajLrToMr = Trajectory::Create(0.2, minTime, maxTime);
        auto estimator = TrajectoryEstimator::Create(_trajLrToMr, _calibParamManager);

        for (const auto &item: lidarPacks) {
            for (const auto &pose: item.poseSeq) {
                auto SE3_LrToMr = item.LmToLr * pose.se3() * item.LmToLr.inverse();

                estimator->AddSE3Measurement(
                        Posed::FromSE3(SE3_LrToMr, pose.timeStamp),
                        OptimizationOption::OPT_SO3 | OptimizationOption::OPT_POS,
                        CalibConfig::Optimization::OptWeight::SO3Weight,
                        CalibConfig::Optimization::OptWeight::POSWeight
                );
            }
        }

        ceres::Solver::Summary initTrajSummary = estimator->Solve(TrajectoryEstimator::DefaultSolverOptions(false));

        LOG_PLAINTEXT("solve finished, info:")
        LOG_PLAINTEXT(initTrajSummary.BriefReport())
        LOG_ENDL()

        // construct reference map
        for (const auto &item: lidarPacks) {
            PosTPointCloud::Ptr scanInRef = PosTPointCloud::Ptr(new PosTPointCloud);
            pcl::transformPointCloud(*item.lidarMap->GetScan(), *scanInRef, item.LmToLr.matrix().cast<float>());
            *_map += *scanInRef;
        }
    }

    RefLiDAROdometer::RefLiDAROdometer(const aligned_vector<LiDAROdometerPack> &lidarPacks,
                                       CalibParamManager::Ptr calibParamManager, double minTime, double maxTime)
            : _map(new PosTPointCloud), _trajLrToMr(), _calibParamManager(std::move(calibParamManager)) {
        Initialization(lidarPacks, minTime, maxTime);
    }

    const PosTPointCloud::Ptr &RefLiDAROdometer::GetMap() const {
        return _map;
    }

    const Trajectory::Ptr &RefLiDAROdometer::GetTrajLrToMr() const {
        return _trajLrToMr;
    }

    RefLiDAROdometer::Ptr RefLiDAROdometer::Create(const aligned_vector<LiDAROdometerPack> &lidarPacks,
                                                   const CalibParamManager::Ptr &calibParamManager, double minTime,
                                                   double maxTime) {
        return std::make_shared<RefLiDAROdometer>(lidarPacks, calibParamManager, minTime, maxTime);
    }

}
