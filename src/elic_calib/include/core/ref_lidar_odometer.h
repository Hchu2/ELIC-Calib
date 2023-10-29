//
// Created by csl on 2/16/23.
//

#ifndef ELIC_CALIB_REF_LIDAR_ODOMETER_H
#define ELIC_CALIB_REF_LIDAR_ODOMETER_H

#include "core/se3_spline.h"
#include "core/pose.hpp"
#include "sensor/lidar.h"
#include <utility>
#include "core/se3_spline.h"

namespace ns_elic {

    struct LiDAROdometerPack {
    public:
        aligned_vector<Posed> poseSeq;
        LiDARFrame::Ptr lidarMap;
        Sophus::SE3d LmToLr;

    public:
        LiDAROdometerPack(const aligned_vector<Posed> &poseSeq, LiDARFrame::Ptr lidarMap,
                          const Sophus::SE3d &lmToLr);
    };

    class RefLiDAROdometer {
    public:
        using Ptr = std::shared_ptr<RefLiDAROdometer>;

    private:
        PosTPointCloud::Ptr _map;
        Trajectory::Ptr _trajLrToMr;
        CalibParamManager::Ptr _calibParamManager;

    public:
        RefLiDAROdometer(const aligned_vector<LiDAROdometerPack> &lidarPacks,
                         CalibParamManager::Ptr calibParamManager, double minTime, double maxTime);

        static RefLiDAROdometer::Ptr Create(const aligned_vector<LiDAROdometerPack> &lidarPacks,
                                            const CalibParamManager::Ptr &calibParamManager, double minTime,
                                            double maxTime);

        [[nodiscard]] const PosTPointCloud::Ptr &GetMap() const;

        [[nodiscard]] const Trajectory::Ptr &GetTrajLrToMr() const;

    protected:
        void Initialization(const aligned_vector<LiDAROdometerPack> &lidarPacks, double minTime, double maxTime);
    };
}


#endif //ELIC_CALIB_REF_LIDAR_ODOMETER_H
