//
// Created by csl on 10/2/22.
//

#ifndef LIC_CALIB_CALIB_DATA_MANAGER_H
#define LIC_CALIB_CALIB_DATA_MANAGER_H

#include "openMVG/cameras/Camera_Pinhole_Brown.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "sensor/imu.h"
#include "sensor/lidar.h"
#include "sensor_msgs/PointCloud2.h"
#include "core/pose.hpp"
#include "core/se3_spline.h"
#include "util/status.hpp"

namespace ns_elic {
    /**
     * the class for data manage
     */
    class CalibDataManager {
    public:
        using Ptr = std::shared_ptr<CalibDataManager>;

    private:
        // the lidar raw frame sequence
        ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> _lidarRawFrames;
        // the lidar undistorted frame sequence
        // ATTENTION: there is one-to-noe correspondence between '_lidarUndistFrames' and '_lidarRawFrames'
        // but not all '_lidarUndistFrames' are valid because some frames' timestamps are out of the b-spline time-range
        ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> _lidarUndistFrames;
        // trans the '_lidarUndistFrames' to global map using b-spline trajectory
        ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> _lidarUndistFramesInRef;

        // [ imu topic, the imu frame sequence ]
        ns_elic::aligned_map<std::string, aligned_vector<IMUFrame::Ptr>> _imuRawFrames;

        double _rawStartTimestamp, _rawEndTimestamp;

        // the start and end timestamp of data for this calibration, usually they are assigned by imu data timestamp range
        double _alignedStartTimestamp, _alignedEndTimestamp;

    public:
        // using config information to load and adjust data in this constructor
        CalibDataManager();

        // the creator
        static CalibDataManager::Ptr Create();

        // get raw lidar frames
        [[nodiscard]] const ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> &
        GetLidarRawFrames() const;

        // get raw imu frames
        [[nodiscard]] const ns_elic::aligned_map<std::string, aligned_vector<IMUFrame::Ptr>> &GetImuRawFrames() const;

        // get undistorted lidar frames
        ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> &GetLidarUndistFrames();

        // get undistorted lidar frame in the reference coordinate
        ns_elic::aligned_map<std::string, ns_elic::aligned_vector<LiDARFrame::Ptr>> &GetLidarUndistFramesInRef();

        LiDARFrame::Ptr CreateLidarUndistMapFrameInRef(const std::string &topic, bool filterPoints = true);

        // using odom pose sequence to trans local undistorted lidar frame to reference coordinate
        void ComputeLidarUndistFramesInRef(const ns_elic::aligned_vector<OdomPosed> &framePose,
                                           const CalibParamManager::Ptr &calibParamManager, const std::string &topic);

        // using trajectory query pose to trans local undistorted lidar frame to reference coordinate
        void ComputeLidarUndistFramesInRef(
                const Trajectory::Ptr &trajectory, const CalibParamManager::Ptr &calibParamManager,
                const std::string &topic
        );

        // access
        [[nodiscard]] double GetRawStartTimestamp() const;

        [[nodiscard]] double GetRawEndTimestamp() const;

        [[nodiscard]] double GetAlignedStartTimestamp() const;

        [[nodiscard]] double GetAlignedEndTimestamp() const;

    protected:
        // load camera, lidar, imu data from the ros bag [according to the config file]
        void LoadCalibData();

        // if data saving is set in the configuration file, the data will be kept in the corresponding location
        void SaveDataToFile();

        // make sure the first imu frame is before camera and lidar data
        // assign the '_alignedStartTimestamp' and '_alignedEndTimestamp'
        void AdjustCalibDataSequence();

        // align the timestamp to zero
        void AlignTimestamp();

        // remove the head data according to the pred
        template<typename ElemType, typename Pred>
        void EraseSeqHeadData(ns_elic::aligned_vector<ElemType> &seq, Pred pred, const std::string &errorMsg) {
            auto iter = std::find_if(seq.begin(), seq.end(), pred);
            if (iter == seq.end()) {
                // find failed
                throw Status(Status::Flag::ERROR, errorMsg);
            } else {
                // adjust
                seq.erase(seq.begin(), iter);
            }
        }

        // remove the tail data according to the pred
        template<typename ElemType, typename Pred>
        void EraseSeqTailData(ns_elic::aligned_vector<ElemType> &seq, Pred pred, const std::string &errorMsg) {
            auto iter = std::find_if(seq.rbegin(), seq.rend(), pred);
            if (iter == seq.rend()) {
                // find failed
                throw Status(Status::Flag::ERROR, errorMsg);
            } else {
                // adjust
                seq.erase(iter.base(), seq.end());
            }
        }

        // output the data status
        void OutputDataStatus() const;

    };
}


#endif //LIC_CALIB_CALIB_DATA_MANAGER_H
