//
// Created by csl on 10/3/22.
//

#ifndef LIC_CALIB_LIDAR_ODOMETER_H
#define LIC_CALIB_LIDAR_ODOMETER_H

#include "core/pose.hpp"
#include "core/se3_spline.h"
#include "pclomp/gicp_omp.hpp"
#include "pclomp/ndt_omp.hpp"
#include "pclomp/voxel_grid_covariance_omp.hpp"
#include "sensor/lidar.h"
#include "util/type_define.hpp"

namespace ns_elic {

    class LiDAROdometer {
    public:
        using Ptr = std::shared_ptr<LiDAROdometer>;

    private:
        // the key frame index in the '_frames'
        ns_elic::aligned_vector<std::size_t> _keyFrameIdx;
        ns_elic::aligned_vector<LiDARFrame::Ptr> _frames;
        // the global map
        LiDARFrame::Ptr _map;
        // ndt
        pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr _ndt;

        bool _initialized;
        // pose sequence
        ns_elic::aligned_vector<OdomPosed> _poseSeq;
    public:
        LiDAROdometer();

        static LiDAROdometer::Ptr Create();

        OdomPosed FeedFrame(
                const LiDARFrame::Ptr &frame, const Eigen::Matrix4d &predCurToLast = Eigen::Matrix4d::Identity(),
                bool updateMap = true
        );

        [[nodiscard]] std::size_t KeyFrameSize() const;

        [[nodiscard]] std::size_t FrameSize() const;

        // getters

        [[nodiscard]] const aligned_vector<size_t> &GetKeyFrameIdxVec() const;

        [[nodiscard]] const aligned_vector<OdomPosed> &GetOdomPoseVec() const;

        [[nodiscard]] aligned_vector<Posed> GetPoseVec() const;

        [[nodiscard]] const LiDARFrame::Ptr &GetMap() const;

        [[nodiscard]] const aligned_vector<LiDARFrame::Ptr> &GetFramesVec() const;

        [[nodiscard]] const pclomp::NormalDistributionsTransform<ns_elic::PosTPoint, ns_elic::PosTPoint>::Ptr &
        GetNdt() const;

        void SaveToFile(const std::string &filename) const;

    protected:
        bool CheckKeyFrame(const OdomPosed &LtoM);

        void UpdateMap(const LiDARFrame::Ptr &frame, const OdomPosed &LtoM);

        static void DownSampleCloud(
                const PosTPointCloud::Ptr &inCloud, const PosTPointCloud::Ptr &outCloud, float leafSize);

        static void RemoveNaNDenseCloud(const PosTPointCloud::Ptr &inCloud, const PosTPointCloud::Ptr &outCloud);
    };
}

#endif //LIC_CALIB_LIDAR_ODOMETER_H
