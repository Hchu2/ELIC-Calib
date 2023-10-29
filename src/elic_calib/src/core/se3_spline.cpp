//
// Created by csl on 10/2/22.
//

#include "core/se3_spline.h"

namespace ns_elic {

    Trajectory::Trajectory(double timeInterval, double startTime, double endTime)
            : Se3Spline(timeInterval, startTime) {
        this->extendKnotsTo(endTime, Sophus::SO3d(), Eigen::Vector3d::Zero());
    }

    Trajectory::Ptr Trajectory::Create(double timeInterval, double startTime, double endTime) {
        return std::make_shared<Trajectory>(timeInterval, startTime, endTime);
    }

    bool Trajectory::TimeStampInRange(double timeStamp) {
        return timeStamp >= this->minTime() && timeStamp <= this->maxTime();
    }

    std::optional<Sophus::SE3d>
    Trajectory::LiDARToRefIMU(double lidarTimeStamp, double imuTimeStamp, const std::string &topic,
                              const CalibParamManager::Ptr &calibParamManager) {
        if (!TimeStampInRange(imuTimeStamp)) {
            return {};
        }
        // lidar-time frame to global
        if (auto curLtoRef = LiDARToRef(lidarTimeStamp, topic, calibParamManager)) {
            // imu-time imu frame to global
            auto curItoRef = this->pose(imuTimeStamp);
            return curItoRef.inverse() * *curLtoRef;
        }
        return {};
    }

    std::optional<Sophus::SE3d>
    Trajectory::CameraToRefIMU(const std::string &camTopic, double cameraTimeStamp, double imuTimeStamp,
                               const CalibParamManager::Ptr &calibParamManager) {
        if (!TimeStampInRange(imuTimeStamp)) {
            return {};
        }
        // camera-time frame to global
        if (auto curCtoRef = CameraToRef(camTopic, cameraTimeStamp, calibParamManager)) {
            // imu-time imu frame to global
            auto curItoRef = this->pose(imuTimeStamp);

            return curItoRef.inverse() * *curCtoRef;
        }
        return {};
    }

    std::optional<Sophus::SE3d>
    Trajectory::LiDARToRef(double lidarTimeStamp, const std::string &topic,
                           const CalibParamManager::Ptr &calibParamManager) {
        double imuTimeStamp = lidarTimeStamp + calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr.at(topic) +
                              calibParamManager->TEMPORAL.TIME_OFFSET_LrToIr;
        if (!TimeStampInRange(imuTimeStamp)) {
            return {};
        }
        // lidar-time imu frame to global
        auto imuTimeItoRef = this->pose(imuTimeStamp);
        auto LtoI = calibParamManager->EXTRI.SE3_LmToIr(topic);

        return imuTimeItoRef * LtoI;
    }

    std::optional<Sophus::SE3d>
    Trajectory::CameraToRef(const std::string &camTopic, double cameraTimeStamp,
                            const CalibParamManager::Ptr &calibParamManager) {
        double imuTimeStamp = cameraTimeStamp + calibParamManager->TEMPORAL.TIME_OFFSET_CkToIr.at(camTopic);
        if (!TimeStampInRange(imuTimeStamp)) {
            return {};
        }
        // camera-time imu frame to global
        auto imuTimeItoRef = this->pose(imuTimeStamp);
        auto CtoI = calibParamManager->EXTRI.SE3_CkToIr(camTopic);

        return imuTimeItoRef * CtoI;
    }

    aligned_vector<Posed> Trajectory::Sampling(double timeDis, double sTime, double eTime) {
        if (timeDis < 0.0) {
            timeDis = this->getDt();
        }
        if (sTime < 0.0 || sTime > this->maxTime()) {
            sTime = this->minTime();
        }
        if (eTime < 0.0 || eTime > this->maxTime()) {
            eTime = this->maxTime();
        }

        aligned_vector<Posed> poseSeq;

        for (double time = sTime; time < eTime;) {
            auto pose = this->pose(time);
            poseSeq.emplace_back(pose.so3(), pose.translation(), time);
            time += timeDis;
        }
        return poseSeq;
    }

    void Trajectory::SamplingToFile(const std::string &filename, double timeDis, double sTime, double eTime) const {
        if (timeDis < 0.0) {
            timeDis = this->getDt();
        }
        if (sTime < 0.0 || sTime > this->maxTime()) {
            sTime = this->minTime();
        }
        if (eTime < 0.0 || eTime > this->maxTime()) {
            eTime = this->maxTime();
        }

        ns_log::FileLogger logger(filename);
        logger.setPrecision(CalibConfig::CalibData::OutputData::Precision);
        for (double time = sTime; time < eTime;) {
            auto pose = this->pose(time);

            Eigen::Quaterniond q(pose.rotationMatrix());
            Eigen::Vector3d t(pose.translation());
            logger.plaintext(time, " ", q.x(), " ", q.y(), " ", q.z(), " ", q.w(), " ", t.transpose());

            time += timeDis;
        }
    }

    aligned_vector<IMUFrame::Ptr>
    Trajectory::ComputeIMUMeasurement(const Eigen::Vector3d &gravityInRef, double timeDis, double sTime, double eTime) {
        if (timeDis < 0.0) {
            timeDis = this->getDt();
        }
        if (sTime < 0.0 || sTime > this->maxTime()) {
            sTime = this->minTime();
        }
        if (eTime < 0.0 || eTime > this->maxTime()) {
            eTime = this->maxTime();
        }
        aligned_vector<IMUFrame::Ptr> measurementVec;
        const auto &so3Spline = this->getSo3Spline();
        const auto &posSpline = this->getPosSpline();
        for (double t = sTime; t < eTime;) {
            auto SO3_ItoRef = so3Spline.evaluate(t);
            Eigen::Vector3d gyro = so3Spline.velocityBody(t);
            Eigen::Vector3d acceInRef = posSpline.acceleration(t);
            Eigen::Vector3d acce = SO3_ItoRef.inverse() * (acceInRef - gravityInRef);
            measurementVec.push_back(IMUFrame::Create(t, gyro, acce));
            t += timeDis;
        }
        return measurementVec;
    }

    std::optional<Sophus::SE3d> Trajectory::IMUToRef(double imuTimeStamp, const std::string &topic,
                                                     const CalibParamManager::Ptr &calibParamManager) {
        double timestamp = imuTimeStamp + calibParamManager->TEMPORAL.TIME_OFFSET_IjToIr.at(topic);
        if (!TimeStampInRange(timestamp)) {
            return {};
        }
        // imu frame to global
        auto IrToRef = this->pose(timestamp);
        auto IjToIr = calibParamManager->EXTRI.SE3_IjToIr(topic);

        return IrToRef * IjToIr;
    }


    Eigen::Vector3d Trajectory::LinearAcceInRef(double t) {
        if (!TimeStampInRange(t)) { return Eigen::Vector3d::Zero(); }
        const auto &posSpline = this->getPosSpline();
        Eigen::Vector3d acceInRef = posSpline.acceleration(t);
        return acceInRef;
    }

    Eigen::Vector3d Trajectory::AngularVeloInRef(double t) {
        if (!TimeStampInRange(t)) { return Eigen::Vector3d::Zero(); }
        const auto &so3Spline = this->getSo3Spline();
        const auto &posSpline = this->getPosSpline();
        auto SO3_BodyToRef = so3Spline.evaluate(t);
        Eigen::Vector3d angularVelInRef = SO3_BodyToRef * so3Spline.velocityBody(t);
        return angularVelInRef;
    }

    Eigen::Vector3d Trajectory::LinearVeloInRef(double t) {
        if (!TimeStampInRange(t)) { return Eigen::Vector3d::Zero(); }
        const auto &posSpline = this->getPosSpline();
        Eigen::Vector3d veloInRef = posSpline.velocity(t);
        return veloInRef;
    }

    Eigen::Vector3d Trajectory::AngularAcceInRef(double t) {
        if (!TimeStampInRange(t)) { return Eigen::Vector3d::Zero(); }
        const auto &so3Spline = this->getSo3Spline();
        const auto &posSpline = this->getPosSpline();
        auto SO3_BodyToRef = so3Spline.evaluate(t);
        Eigen::Vector3d angularAcceInRef = SO3_BodyToRef * so3Spline.accelerationBody(t);
        return angularAcceInRef;
    }
}