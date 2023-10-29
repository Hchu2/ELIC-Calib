//
// Created by csl on 10/2/22.
//

#include "calib/calib_data_manager.h"
#include "calib/lidar_data_loader.h"
#include "config/calib_config.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "util/enum_cast.hpp"
#include "util/status.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/radius_outlier_removal.h"

namespace ns_elic {

    CalibDataManager::CalibDataManager()
            : _rawStartTimestamp(INVALID_TIME_STAMP), _rawEndTimestamp(INVALID_TIME_STAMP),
              _alignedStartTimestamp(INVALID_TIME_STAMP), _alignedEndTimestamp(INVALID_TIME_STAMP) {
        this->LoadCalibData();
        this->AdjustCalibDataSequence();
        this->SaveDataToFile();
        this->AlignTimestamp();
    }

    CalibDataManager::Ptr CalibDataManager::Create() {
        return std::make_shared<CalibDataManager>();
    }

    const ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> &
    CalibDataManager::GetLidarRawFrames() const {
        return _lidarRawFrames;
    }

    const ns_elic::aligned_map<std::string, aligned_vector<IMUFrame::Ptr>> &CalibDataManager::GetImuRawFrames() const {
        return _imuRawFrames;
    }

    ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> &
    CalibDataManager::GetLidarUndistFrames() {
        return _lidarUndistFrames;
    }

    double CalibDataManager::GetRawStartTimestamp() const {
        ASSERT_TIME_STAMP(_rawStartTimestamp)
        return _rawStartTimestamp;
    }

    double CalibDataManager::GetRawEndTimestamp() const {
        ASSERT_TIME_STAMP(_rawEndTimestamp)
        return _rawEndTimestamp;
    }

    double CalibDataManager::GetAlignedStartTimestamp() const {
        ASSERT_TIME_STAMP(_alignedStartTimestamp)
        return _alignedStartTimestamp;
    }

    double CalibDataManager::GetAlignedEndTimestamp() const {
        ASSERT_TIME_STAMP(_alignedEndTimestamp)
        return _alignedEndTimestamp;
    }

    void CalibDataManager::LoadCalibData() {
        // important
        CalibConfig::CheckConfigureStatus();

        LOG_INFO("ready to load the calibration data from rosbag: '", CalibConfig::CalibData::BagPath, "'...")

        // ros bag path and topics
        auto BagPath = CalibConfig::CalibData::BagPath;
        auto IMUTopics = CalibConfig::CalibData::Topic::IMUTopics;
        auto LiDARTopics = CalibConfig::CalibData::Topic::LiDARTopics;
        std::vector<std::string> topicsToQuery;
        // add topics to vector
        for (const auto &item: IMUTopics) { topicsToQuery.push_back(item); }
        for (const auto &item: LiDARTopics) { topicsToQuery.push_back(item); }

        // open the ros bag
        auto bag = std::make_unique<rosbag::Bag>();
        bag->open(BagPath, rosbag::BagMode::Read);

        auto view = rosbag::View();

        // using a temp view to check the time range of the source ros bag
        auto viewTemp = rosbag::View();
        viewTemp.addQuery(*bag, rosbag::TopicQuery(topicsToQuery));
        auto begTime = viewTemp.getBeginTime();
        auto endTime = viewTemp.getEndTime();

        // adjust the data time range
        if (CalibConfig::CalibData::BeginTime > 0.0) {
            begTime += ros::Duration(CalibConfig::CalibData::BeginTime);
            if (begTime > endTime) {
                LOG_WARNING(
                        "begin time '", begTime, "' is out of the bag's data range, set begin time to '",
                        viewTemp.getBeginTime(), "'."
                )
                begTime = viewTemp.getBeginTime();
            }
        }
        if (CalibConfig::CalibData::Duration > 0.0) {
            endTime = begTime + ros::Duration(CalibConfig::CalibData::Duration);
            if (endTime > viewTemp.getEndTime()) {
                LOG_WARNING(
                        "end time '", begTime, " + ", ros::Duration(CalibConfig::CalibData::Duration),
                        "' is out of the bag's data range, set end time to '",
                        viewTemp.getEndTime(), "'."
                )
                endTime = viewTemp.getEndTime();
            }
        }
        LOG_PLAINTEXT("expect data duration: from '", begTime, "' to '", endTime, "'.")

        view.addQuery(
                *bag, rosbag::TopicQuery(topicsToQuery), begTime, endTime
        );

        // create different lidar data loader
        std::map<std::string, LiDARDataLoader::Ptr> lidarDataLoaders;
        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            LiDARDataLoader::Ptr lidarDataLoader = nullptr;

            // get lidar type enum from the string
            auto lidarModel = EnumCast::stringToEnum<LidarModelType>(CalibConfig::CalibData::LiDARModels.at(topic));

            switch (lidarModel) {
                // for these two lidar type, we can use Velodyne16 to unpack data
                case LidarModelType::VLP_16_SIMU:
                case LidarModelType::VLP_16_PACKET:
                    lidarDataLoader = Velodyne16::Create(lidarModel);
                    break;
                case LidarModelType::VLP_16_POINTS:
                case LidarModelType::VLP_32E_POINTS:
                    lidarDataLoader = VelodynePoints::Create(lidarModel);
                    break;
                case LidarModelType::OUSTER_16_POINTS:
                case LidarModelType::OUSTER_32_POINTS:
                case LidarModelType::OUSTER_64_POINTS:
                case LidarModelType::OUSTER_128_POINTS:
                    lidarDataLoader = OusterLiDAR::Create(lidarModel);
                    break;
                    // TODO: add more lidar data loader
                case LidarModelType::LIVOX_HORIZON:
                case LidarModelType::LIVOX_MID40:
                case LidarModelType::LIVOX_MID70:
                case LidarModelType::LIVOX_MID70_SIMU:
                    lidarDataLoader = LivoxLiDAR::Create(lidarModel);
                    break;
                case LidarModelType::RS_16:
                case LidarModelType::RS_M1:
                    throw Status(
                            Status::Flag::FETAL,
                            "the lidar model type '" + std::string(EnumCast::enumToString(lidarModel)) +
                            "' is not supported right now"
                    );
            }
            lidarDataLoaders.insert({topic, lidarDataLoader});
        }

        // read raw data
        for (const auto &item: view) {
            const std::string &topic = item.getTopic();
            if (IMUTopics.cend() != IMUTopics.find(topic)) {
                // imu data item
                sensor_msgs::ImuConstPtr msg = item.instantiate<sensor_msgs::Imu>();

                auto acce = Eigen::Vector3d(
                        msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z
                );
                auto gyro = Eigen::Vector3d(
                        msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z
                );

                auto imuFrame = IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
                _imuRawFrames[topic].push_back(imuFrame);

            } // else if (LiDARTopics.cend() != LiDARTopics.find(topic)) {
            else if ((find(LiDARTopics.begin(), LiDARTopics.end(), topic)) != LiDARTopics.end()) {
                // lidar data item
                LiDARFrame::Ptr lidarFrame = lidarDataLoaders.at(topic)->UnpackScan(item);
                _lidarRawFrames[topic].push_back(lidarFrame);

            }
        }

        bag->close();

        // output
        LOG_ENDL()
        LOG_PLAINTEXT("load data finished, here are the data info:")
        OutputDataStatus();
        LOG_ENDL()
    }

    void CalibDataManager::AdjustCalibDataSequence() {
        LOG_INFO("adjust the data sequences according timestamp...")

        double firstValidTimeStamp, lastValidTimeStamp;

        if (CalibConfig::IMUIntegrated()) {
            // make sure the first and last frame is imu frame
            double firstTimeStampForIMUs =
                    std::max_element(_imuRawFrames.begin(), _imuRawFrames.end(), [](const auto &p1, const auto &p2) {
                        return p1.second.front()->GetTimestamp() < p2.second.front()->GetTimestamp();
                    })->second.front()->GetTimestamp();

            double lastTimeStampForIMUs =
                    std::min_element(_imuRawFrames.begin(), _imuRawFrames.end(), [](const auto &p1, const auto &p2) {
                        return p1.second.back()->GetTimestamp() < p2.second.back()->GetTimestamp();
                    })->second.back()->GetTimestamp();

            firstValidTimeStamp = firstTimeStampForIMUs + CalibConfig::Optimization::TimeOffsetPadding;
            lastValidTimeStamp = lastTimeStampForIMUs - CalibConfig::Optimization::TimeOffsetPadding;

            _rawStartTimestamp = firstTimeStampForIMUs;
            _rawEndTimestamp = lastTimeStampForIMUs;
        } else {
            // make sure the first and last frame is imu frame
            double firstTimeStampForLiDARs =
                    std::max_element(_lidarRawFrames.begin(), _lidarRawFrames.end(),
                                     [](const auto &p1, const auto &p2) {
                                         return p1.second.front()->GetTimestamp() < p2.second.front()->GetTimestamp();
                                     })->second.front()->GetTimestamp();

            double lastTimeStampForLiDARs =
                    std::min_element(_lidarRawFrames.begin(), _lidarRawFrames.end(),
                                     [](const auto &p1, const auto &p2) {
                                         return p1.second.back()->GetTimestamp() < p2.second.back()->GetTimestamp();
                                     })->second.back()->GetTimestamp();

            firstValidTimeStamp = firstTimeStampForLiDARs + CalibConfig::Optimization::TimeOffsetPadding;
            lastValidTimeStamp = lastTimeStampForLiDARs - CalibConfig::Optimization::TimeOffsetPadding;

            _rawStartTimestamp = firstTimeStampForLiDARs;
            _rawEndTimestamp = lastTimeStampForLiDARs;
        }

        if (CalibConfig::IMUIntegrated()) {
            // move lidar frames that are before the first imu frame and after the last imu frame
            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                EraseSeqHeadData(_lidarRawFrames.at(topic), [firstValidTimeStamp](const LiDARFrame::Ptr &frame) {
                    return frame->GetTimestamp() > firstValidTimeStamp;
                }, "the lidar data is valid, there is no intersection between imu data and lidar data.");

                EraseSeqTailData(_lidarRawFrames.at(topic), [lastValidTimeStamp](const LiDARFrame::Ptr &frame) {
                    return frame->GetTimestamp() < lastValidTimeStamp;
                }, "the lidar data is valid, there is no intersection between imu data and lidar data.");
            }
        } else {
            // move lidar frames that are before the first lidar frame and after the last lidar frame
            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                EraseSeqHeadData(_lidarRawFrames.at(topic), [this](const LiDARFrame::Ptr &frame) {
                    return frame->GetTimestamp() > _rawStartTimestamp;
                }, "the lidar data is valid, there is no intersection between ref data and current lidar data.");

                EraseSeqTailData(_lidarRawFrames.at(topic), [this](const LiDARFrame::Ptr &frame) {
                    return frame->GetTimestamp() < _rawEndTimestamp;
                }, "the lidar data is valid, there is no intersection between ref data and current lidar data.");
            }
        }

        // output
        LOG_PLAINTEXT("adjust data finished:")
        OutputDataStatus();
        LOG_ENDL()
    }

    void CalibDataManager::OutputDataStatus() const {

        if (CalibConfig::IMUIntegrated()) {
            if (!_imuRawFrames.empty()) {
                for (const auto &[topic, data]: _imuRawFrames) {
                    auto imuInfo = fmt::format(
                            "IMU: {}, frames: {}, timespan: [{:.9f}, {:.9f}]", topic, data.size(),
                            data.front()->GetTimestamp(), data.back()->GetTimestamp()
                    );
                    LOG_PLAINTEXT(imuInfo)
                }
            } else {
                throw Status(Status::Flag::FETAL, "no imu frames data in the bag!!!");
            }
        }
        if (CalibConfig::LiDARIntegrated()) {
            if (!_lidarRawFrames.empty()) {
                for (const auto &[topic, data]: _lidarRawFrames) {
                    auto lidarInfo = fmt::format(
                            "LiDAR: {}, frames: {}, timespan: [{:.9f}, {:.9f}]", topic, data.size(),
                            data.front()->GetTimestamp(), data.back()->GetTimestamp()
                    );
                    LOG_PLAINTEXT(lidarInfo)
                }
            } else {
                throw Status(Status::Flag::FETAL, "no lidar frames data in the bag!!!");
            }
        }

        auto rawTimeInfo = fmt::format(
                "raw start time stamp: {:.9f}(s), raw end time stamp: {:.9f}(s), total: {:.9f}(s)",
                _rawStartTimestamp, _rawEndTimestamp,
                _rawEndTimestamp - _rawStartTimestamp);

        auto alignedTimeInfo = fmt::format(
                "aligned start time stamp: {:.9f}(s), aligned end time stamp: {:.9f}(s), total: {:.9f}(s)",
                _alignedStartTimestamp, _alignedEndTimestamp,
                _alignedEndTimestamp - _alignedStartTimestamp);
        LOG_PLAINTEXT(rawTimeInfo)
        LOG_PLAINTEXT(alignedTimeInfo)
    }

    void CalibDataManager::ComputeLidarUndistFramesInRef(const aligned_vector<OdomPosed> &framePose,
                                                         const CalibParamManager::Ptr &calibParamManager,
                                                         const std::string &topic) {
        if (framePose.size() != _lidarUndistFrames.at(topic).size()) {
            throw Status(Status::Flag::ERROR, "the odometer pose vector size is not equal to the undistorted frames");
        }
        _lidarUndistFramesInRef[topic].clear();
        for (int i = 0; i < framePose.size(); ++i) {
            const auto &curLtoM = framePose.at(i);
            const Eigen::Matrix4d curLtoRef = calibParamManager->EXTRI.SE3_LmToLr(topic).matrix() * curLtoM.pose;
            const auto &curFrame = _lidarUndistFrames.at(topic).at(i);
            if (curFrame != nullptr) {
                // trans to ref
                PosTPointCloud::Ptr scanInRef = PosTPointCloud::Ptr(new PosTPointCloud);
                pcl::transformPointCloud(*curFrame->GetScan(), *scanInRef, curLtoRef.cast<float>());
                _lidarUndistFramesInRef[topic].push_back(LiDARFrame::Create(curFrame->GetTimestamp(), scanInRef));
            } else {
                _lidarUndistFramesInRef[topic].push_back(nullptr);
            }
        }
    }

    void CalibDataManager::ComputeLidarUndistFramesInRef(const Trajectory::Ptr &trajectory,
                                                         const CalibParamManager::Ptr &calibParamManager,
                                                         const std::string &topic) {
        _lidarUndistFramesInRef[topic].clear();
        for (const auto &curFrame: _lidarUndistFrames.at(topic)) {
            if (curFrame == nullptr) {
                _lidarUndistFramesInRef[topic].push_back(nullptr);
                continue;
            }
            // query pose
            if (auto curLtoRef = trajectory->LiDARToRef(curFrame->GetTimestamp(), topic, calibParamManager)) {
                // trans to ref
                PosTPointCloud::Ptr scanInRef = PosTPointCloud::Ptr(new PosTPointCloud);
                pcl::transformPointCloud(*curFrame->GetScan(), *scanInRef, curLtoRef->matrix().cast<float>());
                _lidarUndistFramesInRef[topic].push_back(LiDARFrame::Create(curFrame->GetTimestamp(), scanInRef));
            } else {
                _lidarUndistFramesInRef[topic].push_back(nullptr);
            }
        }
    }

    ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> &
    CalibDataManager::GetLidarUndistFramesInRef() {
        return _lidarUndistFramesInRef;
    }

    void CalibDataManager::SaveDataToFile() {
        if (CalibConfig::CalibData::OutputData::OutputIMUFrame) {
            for (const auto &[topic, data]: _imuRawFrames) {
                std::string dir = CalibConfig::CalibData::OutputData::OutputIMUFrameDirs.at(topic);
                if (IMUFrame::SaveFramesToDisk(dir, data, CalibConfig::CalibData::OutputData::Precision)) {
                    LOG_INFO("save imu frames for topic ", topic, " to dir '", dir, "' finished.")
                } else {
                    LOG_WARNING("unknown error happened when save imu frames for topic ", topic, " to dir '", dir, "'.")
                }
            }
        }
        if (CalibConfig::CalibData::OutputData::OutputLiDARFrame) {
            for (const auto &[topic, data]: _lidarRawFrames) {
                std::string dir = CalibConfig::CalibData::OutputData::OutputLiDARFrameDirs.at(topic);
                if (LiDARFrame::SaveFramesToDisk(dir, data, CalibConfig::CalibData::OutputData::Precision)) {
                    LOG_INFO("save lidar frames for topic ", topic, "to dir '", dir, "' finished.")
                } else {
                    LOG_WARNING(
                            "unknown error happened when save lidar frames for topic ", topic, " to dir '", dir, "'."
                    )
                }
            }
        }

        if (CalibConfig::CalibData::OutputData::OutputIMUFrame ||
            CalibConfig::CalibData::OutputData::OutputLiDARFrame) {
            throw Status(Status::Flag::FINE,
                         "the data has been output to the target directory, press any key to continue...");
        }
    }

    void CalibDataManager::AlignTimestamp() {

        LOG_INFO("align timestamp...")
        // align time stamp to zero
        for (const auto &[topic, data]: _imuRawFrames) {
            for (const auto &item: data) {
                item->SetTimestamp(item->GetTimestamp() - _rawStartTimestamp);
            }
        }

        for (const auto &[topic, data]: _lidarRawFrames) {
            for (const auto &item: data) {
                item->SetTimestamp(item->GetTimestamp() - _rawStartTimestamp);
                for (auto &p: *item->GetScan()) {
                    p.timestamp -= _rawStartTimestamp;
                }
            }
        }

        // update
        _alignedEndTimestamp = _rawEndTimestamp - _rawStartTimestamp;
        // will be zero
        _alignedStartTimestamp = _rawStartTimestamp - _rawStartTimestamp;

        LOG_PLAINTEXT("align data time finished:")
        OutputDataStatus();
        LOG_ENDL()
    }

    LiDARFrame::Ptr CalibDataManager::CreateLidarUndistMapFrameInRef(const std::string &topic, bool filterPoints) {
        PosTPointCloud::Ptr mapCloud(new PosTPointCloud), mapCloudNonOutlier(new PosTPointCloud);
        for (const auto &frame: _lidarUndistFramesInRef.at(topic)) {
            *mapCloud += *frame->GetScan();
        }
        pcl::RadiusOutlierRemoval<PosTPoint> ror;
        ror.setInputCloud(mapCloud);
        ror.setRadiusSearch(0.1f);
        ror.setMinNeighborsInRadius(20);
        ror.filter(*mapCloudNonOutlier);
        if (filterPoints) {
            // down sample
            pcl::VoxelGrid<PosTPoint> filter;
            filter.setInputCloud(mapCloudNonOutlier);
            auto leafSize = static_cast<float>(CalibConfig::LiDAROdometer::NDTKeyFrameDownSample);
            filter.setLeafSize(leafSize, leafSize, leafSize);

            PosTPointCloud::Ptr filteredCloud(new PosTPointCloud);
            filter.filter(*filteredCloud);
            return ns_elic::LiDARFrame::Create(INVALID_TIME_STAMP, filteredCloud);

        } else {
            return ns_elic::LiDARFrame::Create(INVALID_TIME_STAMP, mapCloudNonOutlier);
        }
    }
}
