//
// Created by csl on 10/2/22.
//

#ifndef LIC_CALIB_SE3_SPLINE_H
#define LIC_CALIB_SE3_SPLINE_H

#include "basalt/spline/se3_spline.h"
#include "calib/calib_param_manager.h"
#include "config/calib_config.h"
#include "optional"
#include "core/pose.hpp"
#include "sensor/imu.h"
#include "fstream"

namespace ns_elic {
    class Trajectory : public basalt::Se3Spline<CalibConfig::BSpline::SplineOrder, double> {
    public:
        using Ptr = std::shared_ptr<Trajectory>;
        using parent_type = basalt::Se3Spline<CalibConfig::BSpline::SplineOrder, double>;

    public:
        Trajectory(double timeInterval, double startTime, double endTime);

        static Trajectory::Ptr Create(double timeInterval, double startTime, double endTime);

        bool TimeStampInRange(double timeStamp);

        std::optional<Sophus::SE3d>
        LiDARToRefIMU(double lidarTimeStamp, double imuTimeStamp, const std::string &topic,
                      const CalibParamManager::Ptr &calibParamManager);

        std::optional<Sophus::SE3d>
        CameraToRefIMU(const std::string &camTopic, double cameraTimeStamp, double imuTimeStamp,
                       const CalibParamManager::Ptr &calibParamManager);

        std::optional<Sophus::SE3d>
        LiDARToRef(double lidarTimeStamp, const std::string &topic, const CalibParamManager::Ptr &calibParamManager);

        std::optional<Sophus::SE3d>
        CameraToRef(const std::string &camTopic, double cameraTimeStamp,
                    const CalibParamManager::Ptr &calibParamManager);

        std::optional<Sophus::SE3d>
        IMUToRef(double imuTimeStamp, const std::string &topic, const CalibParamManager::Ptr &calibParamManager);

        aligned_vector<Posed> Sampling(double timeDis = INVALID_TIME_STAMP, double sTime = INVALID_TIME_STAMP,
                                       double eTime = INVALID_TIME_STAMP);

        void SamplingToFile(const std::string &filename, double timeDis = INVALID_TIME_STAMP,
                            double sTime = INVALID_TIME_STAMP, double eTime = INVALID_TIME_STAMP) const;

        aligned_vector<IMUFrame::Ptr>
        ComputeIMUMeasurement(const Eigen::Vector3d &gravityInRef, double timeDis = INVALID_TIME_STAMP,
                              double sTime = INVALID_TIME_STAMP, double eTime = INVALID_TIME_STAMP);

        Eigen::Vector3d LinearAcceInRef(double t);

        Eigen::Vector3d LinearVeloInRef(double t);

        Eigen::Vector3d AngularVeloInRef(double t);

        Eigen::Vector3d AngularAcceInRef(double t);

    public:
        void Save(const std::string &filename) const {
            std::ofstream file(filename);
            cereal::JSONOutputArchive ar(file);
            ar(cereal::make_nvp("trajectory", *this));
        }

        static Trajectory::Ptr Load(const std::string &filename) {
            auto traj = Trajectory::Create(0, 0, 0);
            std::ifstream file(filename);
            cereal::JSONInputArchive ar(file);
            ar(cereal::make_nvp("trajectory", *traj));
            return traj;
        }

    };
}

#endif //LIC_CALIB_SE3_SPLINE_H
