//
// Created by csl on 10/4/22.
//

#include "core/scan_undistortion.h"
#include "utility"

namespace ns_elic {

    ScanUndistortion::ScanUndistortion(Trajectory::Ptr trajectory,
                                       CalibParamManager::Ptr calibParamManager)
            : _trajectory(std::move(trajectory)), _calibParamManager(std::move(calibParamManager)) {}

    ScanUndistortion::Ptr
    ScanUndistortion::Create(const Trajectory::Ptr &trajectory, const CalibParamManager::Ptr &calibParamManager) {
        return std::make_shared<ScanUndistortion>(trajectory, calibParamManager);
    }


    void ScanUndistortion::UndistortToScan(const CalibDataManager::Ptr &calibDataManager, const std::string &topic,
                                           std::uint32_t option) {
        auto &lidarRawFrames = calibDataManager->GetLidarRawFrames().at(topic);
        auto &lidarUndistFrames = calibDataManager->GetLidarUndistFrames()[topic];
        lidarUndistFrames.clear();

        bool correctPos = IsOptionWith(UNDIST_POS, option);
        for (const auto &frame: lidarRawFrames) {
            if (auto undistLidarFrame = UndistortToScan(frame, topic, correctPos)) {
                lidarUndistFrames.push_back(*undistLidarFrame);
            } else {
                lidarUndistFrames.push_back(nullptr);
            }
        }
    }

    std::optional<LiDARFrame::Ptr>
    ScanUndistortion::UndistortToScan(const LiDARFrame::Ptr &lidarFrame, const std::string &topic, bool correctPos) {

        auto scanToRef = _trajectory->LiDARToRef(lidarFrame->GetTimestamp(), topic, _calibParamManager);
        // id this time stamp is invalid, return
        if (!scanToRef) {
            return {};
        }
        Sophus::SE3d refToScan = scanToRef->inverse();

        // prepare
        PosTPointCloud::Ptr undistScan(new PosTPointCloud);
        auto rawScan = lidarFrame->GetScan();
        // assign
        undistScan->header = rawScan->header;
        undistScan->height = rawScan->height;
        undistScan->width = rawScan->width;
        undistScan->resize(rawScan->height * rawScan->width);
        undistScan->is_dense = rawScan->is_dense;

        for (int h = 0; h < rawScan->height; h++) {
            for (int w = 0; w < rawScan->width; w++) {
                // for each point
                // const auto &rawPoint = rawScan->at(w, h);
                const auto &rawPoint = rawScan->points[h * rawScan->width + w];
                PosTPoint undistPoint;
                // if it's a nan point
                if (IS_POINT_NAN(rawPoint)) {
                    SET_POST_POINT_NAN(undistPoint)
                } else {
                    if (auto pointToRef = _trajectory->LiDARToRef(rawPoint.timestamp, topic, _calibParamManager)) {
                        // lidar point to imu global
                        Sophus::SE3d pointToScan = refToScan * *pointToRef;

                        // trans
                        Eigen::Vector3d rp(rawPoint.x, rawPoint.y, rawPoint.z), up;
                        if (correctPos) {
                            up = pointToScan * rp;
                        } else {
                            up = pointToScan.so3() * rp;
                        }

                        undistPoint.x = static_cast<float>(up(0));
                        undistPoint.y = static_cast<float>(up(1));
                        undistPoint.z = static_cast<float>(up(2));
                        undistPoint.timestamp = rawPoint.timestamp;
                    } else {
                        // we can't undistort it (no condition)
                        SET_POST_POINT_NAN(undistPoint)
                    }
                }
                // undistScan->at(w, h) = undistPoint;
                undistScan->points[h * rawScan->width + w] = undistPoint;
            }
        }

        return LiDARFrame::Create(lidarFrame->GetTimestamp(), undistScan);
    }

    void ScanUndistortion::UndistortToRef(const CalibDataManager::Ptr &calibDataManager, const std::string &topic,
                                          std::uint32_t option) {
        auto &lidarRawFrames = calibDataManager->GetLidarRawFrames().at(topic);
        auto &lidarUndistFramesInRef = calibDataManager->GetLidarUndistFramesInRef()[topic];
        lidarUndistFramesInRef.clear();

        bool correctPos = IsOptionWith(UNDIST_POS, option);
        for (const auto &frame: lidarRawFrames) {
            if (auto undistLidarFrame = UndistortToRef(frame, topic, correctPos)) {
                lidarUndistFramesInRef.push_back(*undistLidarFrame);
            } else {
                lidarUndistFramesInRef.push_back(nullptr);
            }
        }
    }

    std::optional<LiDARFrame::Ptr>
    ScanUndistortion::UndistortToRef(const LiDARFrame::Ptr &lidarFrame, const std::string &topic, bool correctPos) {
        // prepare
        PosTPointCloud::Ptr undistScan(new PosTPointCloud);
        auto rawScan = lidarFrame->GetScan();
        // assign
        undistScan->header = rawScan->header;
        undistScan->height = rawScan->height;
        undistScan->width = rawScan->width;
        undistScan->resize(rawScan->height * rawScan->width);
        undistScan->is_dense = rawScan->is_dense;

        for (int h = 0; h < rawScan->height; h++) {
            for (int w = 0; w < rawScan->width; w++) {
                // for each point
                // const auto &rawPoint = rawScan->at(w, h);
                const auto &rawPoint = rawScan->points[h * rawScan->width + w];
                PosTPoint undistPoint;
                // if it's a nan point
                if (IS_POINT_NAN(rawPoint)) {
                    SET_POST_POINT_NAN(undistPoint)
                } else {
                    if (auto pointToRef = _trajectory->LiDARToRef(rawPoint.timestamp, topic, _calibParamManager)) {
                        // trans
                        Eigen::Vector3d rp(rawPoint.x, rawPoint.y, rawPoint.z), up;
                        if (correctPos) {
                            up = *pointToRef * rp;
                        } else {
                            up = pointToRef->so3() * rp;
                        }

                        undistPoint.x = static_cast<float>(up(0));
                        undistPoint.y = static_cast<float>(up(1));
                        undistPoint.z = static_cast<float>(up(2));
                        undistPoint.timestamp = rawPoint.timestamp;
                    } else {
                        // we can't undistort it (no condition)
                        SET_POST_POINT_NAN(undistPoint)
                    }
                }
                // undistScan->at(w, h) = undistPoint;
                undistScan->points[h * rawScan->width + w] = undistPoint;
            }
        }

        return LiDARFrame::Create(lidarFrame->GetTimestamp(), undistScan);
    }
}