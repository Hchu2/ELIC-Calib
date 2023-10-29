//
// Created by csl on 10/3/22.
//

#include "core/lidar_odometer.h"
#include "config/calib_config.h"
#include "util/utils.hpp"

namespace ns_elic {

    LiDAROdometer::LiDAROdometer()
        : _map(nullptr),
          _ndt(new pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>),
          _initialized(false) {
        // init the ndt omp object
        _ndt->setResolution(static_cast<float>(CalibConfig::LiDAROdometer::NDTResolution));
        _ndt->setNumThreads(CalibConfig::Optimization::ThreadNum);
        _ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        _ndt->setTransformationEpsilon(1E-3);
        _ndt->setStepSize(0.01);
        _ndt->setMaximumIterations(50);
    }

    LiDAROdometer::Ptr LiDAROdometer::Create() {
        return std::make_shared<LiDAROdometer>();
    }

    OdomPosed LiDAROdometer::FeedFrame(const LiDARFrame::Ptr &frame, const Eigen::Matrix4d &predCurToLast,
                                       bool updateMap) {
        // LOG_PLAINTEXT(fmt::format("align lidar frame at: [{:.9f}]", frame->GetTimestamp()));
        OdomPosed curLtoM;
        if (!_initialized) {
            // identity
            curLtoM = OdomPosed(frame->GetTimestamp());
            // create map
            _map = LiDARFrame::Create(frame->GetTimestamp());
            // here the pose id identity
            _initialized = true;
        } else {
            // down sample
            PosTPointCloud::Ptr filterCloud(new PosTPointCloud());
            if (frame->GetScan()->is_dense) {
                RemoveNaNDenseCloud(frame->GetScan(), filterCloud);
                DownSampleCloud(frame->GetScan(), filterCloud, 0.05);
            } else
                DownSampleCloud(frame->GetScan(), filterCloud, 0.5);
            _ndt->setInputSource(filterCloud);

            // organize the pred pose from cur frame to map

            Eigen::Matrix4d predCurLtoM = this->_poseSeq.back().pose * predCurToLast;
            PosTPointCloud::Ptr outputCloud(new PosTPointCloud());
            _ndt->align(*outputCloud, predCurLtoM.cast<float>());

            // get pose
            Eigen::Matrix4d pose = _ndt->getFinalTransformation().cast<double>();
            curLtoM = OdomPosed(frame->GetTimestamp(), pose);
            // LOG_PLAINTEXT("align lidar frame finished.");
        }

        if (updateMap && CheckKeyFrame(curLtoM)) {
            UpdateMap(frame, curLtoM);
            _keyFrameIdx.push_back(_frames.size());
            // LOG_PLAINTEXT("update map.")
        }

        _poseSeq.push_back(curLtoM);
        _frames.push_back(frame);
        // LOG_ENDL()

        return curLtoM;
    }

    bool LiDAROdometer::CheckKeyFrame(const OdomPosed &LtoM) {
        static Eigen::Vector3d lastPos(0.0, 0.0, 0.0);
        static Eigen::Vector3d lastYPR(0.0, 0.0, 0.0);

        Eigen::Vector3d curPos = LtoM.pose.block<3, 1>(0, 3);
        double posDist = (curPos - lastPos).norm();

        // get current rotMat, ypr
        const Eigen::Matrix3d rotMat(LtoM.pose.block<3, 3>(0, 0));
        Eigen::Vector3d curYPR = ns_elic::RotMatToYPR(rotMat);
        Eigen::Vector3d deltaAngle = curYPR - lastYPR;
        for (int i = 0; i < 3; i++) {
            deltaAngle(i) = ns_elic::NormalizeAngle(deltaAngle(i));
        }
        deltaAngle = deltaAngle.cwiseAbs();

        if (_frames.empty() || posDist > 0.2 ||
            deltaAngle(0) > 5.0 || deltaAngle(1) > 5.0 || deltaAngle(2) > 5.0) {
            // update state
            lastPos = curPos;
            lastYPR = curYPR;
            return true;
        }
        return false;
    }

    void LiDAROdometer::UpdateMap(const LiDARFrame::Ptr &frame, const OdomPosed &LtoM) {
        // TODO: update the first map frame using all points after this program is fine
        //        if (_frames.empty()) {
        //            // copy the frame point cloud to the map
        //            *_map->GetScan() += *frame->GetScan();
        //        } else {
        // down sample
        PosTPointCloud::Ptr filteredCloud(new PosTPointCloud);
        if (frame->GetScan()->is_dense) {
            RemoveNaNDenseCloud(frame->GetScan(), filteredCloud);
        } else
            DownSampleCloud(
                    frame->GetScan(),
                    filteredCloud,
                    static_cast<float>(CalibConfig::LiDAROdometer::NDTKeyFrameDownSample));


        // transform
        PosTPointCloud::Ptr transformCloud(new PosTPointCloud);
        pcl::transformPointCloud(*filteredCloud, *transformCloud, LtoM.pose.cast<float>());

        *_map->GetScan() += *transformCloud;
        //        }
        // set the target point cloud
        _ndt->setInputTarget(_map->GetScan());
    }

    void LiDAROdometer::DownSampleCloud(const PosTPointCloud::Ptr &inCloud,
                                        const PosTPointCloud::Ptr &outCloud,
                                        float leafSize) {
        pcl::VoxelGrid<PosTPoint> filter;
        filter.setInputCloud(inCloud);
        filter.setLeafSize(leafSize, leafSize, leafSize);
        filter.filter(*outCloud);
    }

    void LiDAROdometer::RemoveNaNDenseCloud(const PosTPointCloud::Ptr &inCloud,
                                            const PosTPointCloud::Ptr &outCloud) {
        inCloud->is_dense = false;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*inCloud, *outCloud, indices);
        outCloud->is_dense = true;
        inCloud->is_dense = true;
    }

    std::size_t LiDAROdometer::KeyFrameSize() const {
        return _keyFrameIdx.size();
    }

    std::size_t LiDAROdometer::FrameSize() const {
        return _frames.size();
    }

    const aligned_vector<size_t> &LiDAROdometer::GetKeyFrameIdxVec() const {
        return _keyFrameIdx;
    }

    const aligned_vector<OdomPosed> &LiDAROdometer::GetOdomPoseVec() const {
        return _poseSeq;
    }

    const LiDARFrame::Ptr &LiDAROdometer::GetMap() const {
        return _map;
    }

    const aligned_vector<LiDARFrame::Ptr> &LiDAROdometer::GetFramesVec() const {
        return _frames;
    }

    const pclomp::NormalDistributionsTransform<ns_elic::PosTPoint, ns_elic::PosTPoint>::Ptr &
    LiDAROdometer::GetNdt() const {
        return _ndt;
    }

    void LiDAROdometer::SaveToFile(const std::string &filename) const {
        ns_log::FileLogger logger(filename);
        logger.setPrecision(CalibConfig::CalibData::OutputData::Precision);
        for (const auto &p: this->_poseSeq) {
            Eigen::Quaterniond q(p.pose.topLeftCorner<3, 3>());
            Eigen::Vector3d t(p.pose.topRightCorner<3, 1>());
            logger.plaintext(p.timeStamp, " ", q.x(), " ", q.y(), " ", q.z(), " ", q.w(), " ", t.transpose());
        }
    }

    aligned_vector<Posed> LiDAROdometer::GetPoseVec() const {
        aligned_vector<Posed> poseVec(_poseSeq.size());
        for (int i = 0; i < poseVec.size(); ++i) {
            poseVec.at(i) = Posed::FromT(_poseSeq.at(i).pose, _poseSeq.at(i).timeStamp);
        }
        return poseVec;
    }

}