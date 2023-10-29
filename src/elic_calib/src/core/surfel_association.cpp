//
// Created by csl on 10/5/22.
//

#include "core/surfel_association.h"
#include "config/calib_config.h"
#include "utility"

namespace ns_elic {


    SurfelAssociation::Ptr SurfelAssociation::Create() {
        return std::make_shared<SurfelAssociation>();
    }

    const aligned_vector<ns_elic::SurfelPlane> &SurfelAssociation::GetSurfelPlanes() const {
        return _surfelConstructor->GetSurfelPlanes();
    }

    const aligned_vector<ns_elic::aligned_vector<ns_elic::SurfelPoint>> &SurfelAssociation::GetSpointPerSurfel() const {
        return _spointPerSurfel;
    }

    const aligned_vector<SurfelPoint> &SurfelAssociation::GetSpointAll() const {
        return _spointAll;
    }

    void SurfelAssociation::SetUp(const SurfelConstructor::Ptr &surfelConstructor,
                                  const ns_elic::aligned_vector<LiDARFrame::Ptr> &lidarFrames,
                                  const ns_elic::aligned_vector<LiDARFrame::Ptr> &lidarFramesInRef, bool Is_simu) {
        _surfelConstructor = surfelConstructor;

        // resize for recording, index by _surfelPlanes' id
        _spointPerSurfel.resize(_surfelConstructor->GetSurfelPlanes().size());

        for (int i = 0; i < lidarFrames.size(); ++i) {
            const auto &lidarFrame = lidarFrames.at(i);
            const auto &lidarFrameInRef = lidarFramesInRef.at(i);
            // create the point to plane association, the result will save into '_spointPerSurfel' and '_spointAll'
            CreateAssociation(lidarFrame, lidarFrameInRef, lidarFrame->GetScan()->is_dense);
        }

        // down sample the all spoint
        // for simulation, further down sample livox point
        if (lidarFrames.at(0)->GetScan()->is_dense && Is_simu)
            for (int i = 0; i < _spointAll.size(); i += 50) {
                _spointDownSampled.push_back(_spointAll.at(i));
            }
        else
            for (int i = 0; i < _spointAll.size(); i += 10) {
                _spointDownSampled.push_back(_spointAll.at(i));
            }

        LOG_PLAINTEXT("all surfel points count: ", _spointAll.size())
        LOG_PLAINTEXT("surfel points down sampled count: ", _spointDownSampled.size())
    }

    /**
     * two main helper functions
     */
    void
    SurfelConstructor::CreateSurfelPlanes(const pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr &ndt) {
        Eigen::Vector3i counter(0, 0, 0);
        // create surfel planes
        for (const auto &item: ndt->getTargetCells().getLeaves()) {
            auto leaf = item.second;

            // check leaf points size
            if (leaf.nr_points < 10) {
                continue;
            }

            // check plane type: eigenValue, eigenVector
            // -1: invalid plane, 0: Normal vector near the x-axis, 1: Normal vector near the y-axis, 2: Normal vector near the z-axis
            int planeType = CheckPlaneType(leaf.getEvals(), leaf.getEvecs());

            if (planeType < 0) {
                continue;
            } else {
                counter(planeType) += 1;
            }

            // fit plane [ransac method]
            Eigen::Vector4d surfCoeff;
            PosTPointCloud::Ptr cloudInliers(new PosTPointCloud);
            if (!FitPlane(leaf.pointList_.makeShared(), surfCoeff, cloudInliers)) {
                continue;
            }

            // construct the surfel plane
            SurfelPlane surfel(surfCoeff, leaf.pointList_, *cloudInliers);

            _surfelPlanes.push_back(surfel);
        }
        LOG_PLAINTEXT(fmt::format("total surfel planes: {}", _surfelPlanes.size()))
        LOG_PLAINTEXT(fmt::format("'0' type surfel planes count: {}", counter(0)))
        LOG_PLAINTEXT(fmt::format("'1' type surfel planes count: {}", counter(1)))
        LOG_PLAINTEXT(fmt::format("'2' type surfel planes count: {}", counter(2)))
    }

    SurfelConstructor::SurfelConstructor(const pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr &ndt) {
        CreateSurfelPlanes(ndt);
    }

    void SurfelAssociation::CreateAssociation(const LiDARFrame::Ptr &lidarFrame,
                                              const LiDARFrame::Ptr &lidarFrameInRef,
                                              bool is_dense) {
        // get lidar scan
        const auto &lidarScan = lidarFrame->GetScan();
        const auto &lidarScanInRef = lidarFrameInRef->GetScan();
        const auto &surfelPlanes = _surfelConstructor->GetSurfelPlanes();

        // get the width and height of this scan
        const std::size_t width = lidarScan->width;
        const std::size_t height = lidarScan->height;

        // the points threshold for a ring
        const std::size_t perRingSelectedNum = 2;

        // if this value is negative, no plane is associated with this point
        // otherwise, the value is the associated plane id
        int associatedFlag[width * height];
        // init flag vector
        for (unsigned int i = 0; i < width * height; i++) {
            associatedFlag[i] = -1;
        }
        // using omp to speed up
#pragma omp parallel for num_threads(omp_get_max_threads())
        for (int planeId = 0; planeId < surfelPlanes.size(); planeId++) {
            // this container save the info of the surfel plane and the point
            // [which ring(h)][valid points in this ring(index)]
            std::vector<std::vector<int>> ringMasks;

            AssociateScanToSurfel(surfelPlanes.at(planeId), lidarScanInRef,
                                  CalibConfig::DataAssociate::AssociateRadius, ringMasks);

            // for organized points such as VLP and Ouster
            if (!is_dense) {
                for (int h = 0; h < height; h++) {
                    // if the points are a few, continue
                    if (ringMasks.at(h).size() < perRingSelectedNum * 2)
                        continue;
                    // compute step for choosing point
                    std::size_t step = ringMasks.at(h).size() / (perRingSelectedNum + 1);
                    step = std::max(step, 1UL);
                    // for each ring mask
                    for (int selected = 0; selected < perRingSelectedNum; selected++) {
                        int w = ringMasks.at(h).at(step * (selected + 1) - 1);
                        // record the associated plane id
                        associatedFlag[h * width + w] = planeId;
                    }
                }
            } else {
                for (int h = 0; h < height; h++) {
                    // if the points are a few, continue
                    if (ringMasks.at(h).size() < perRingSelectedNum * 2)
                        continue;
                    // compute step for choosing point
                    // std::size_t step = ringMasks.at(h).size() / (perRingSelectedNum + 1);
                    // step = std::max(step, 1UL);
                    // for each ring mask
                    for (int i = 0; i < ringMasks.at(h).size(); i++)
                        associatedFlag[ringMasks.at(h).at(i)] = planeId;
                }
            }
        }

        // in chronological order
        SurfelPoint spoint;

        for (int w = 0; w < width; w++) {
            for (int h = 0; h < height; h++) {
                // check the point is associated with a plane
                if (associatedFlag[h * width + w] == -1)
                    continue;

                // construct a spoint
                int index = h * width + w;
                spoint.pointInScan = Eigen::Vector3d(
                        //         lidarScan->at(w, h).x, lidarScan->at(w, h).y, lidarScan->at(w, h).z
                        // );
                        lidarScan->points[index].x, lidarScan->points[index].y, lidarScan->points[index].z);
                // just for visualization
                spoint.pointInRef = Eigen::Vector3d(
                        //         lidarScanInRef->at(w, h).x, lidarScanInRef->at(w, h).y, lidarScanInRef->at(w, h).z
                        // );
                        lidarScanInRef->points[index].x, lidarScanInRef->points[index].y, lidarScanInRef->points[index].z);

                spoint.planeId = associatedFlag[h * width + w];
                // spoint.timestamp = lidarScan->at(w, h).timestamp;
                spoint.timestamp = lidarScan->points[index].timestamp;

                // record
                _spointPerSurfel.at(spoint.planeId).push_back(spoint);
                _spointAll.push_back(spoint);
            }
        }
    }

    /**
     * static helper functions
     */
    int SurfelConstructor::CheckPlaneType(const Eigen::Vector3d &eigenValues,
                                          const Eigen::Matrix3d &eigenVectors) {
        Eigen::Vector3d sortedVec;
        Eigen::Vector3i ind;
        SortEigenVec3d(eigenValues, sortedVec, ind);

        double p = 2 * (sortedVec[1] - sortedVec[2]) / (sortedVec[2] + sortedVec[1] + sortedVec[0]);

        if (p < CalibConfig::DataAssociate::PlaneLambdaThd) {
            return -1;
        }

        int minIdx = ind[2];
        Eigen::Vector3d planeNormal = eigenVectors.block<3, 1>(0, minIdx);
        // The eigenvector corresponding to the minimum eigenvalue can be roughly regarded as the normal vector of the plane
        planeNormal = planeNormal.array().abs();

        SortEigenVec3d(planeNormal, sortedVec, ind);
        return ind[0];
    }

    bool SurfelConstructor::FitPlane(const PosTPointCloud::Ptr &cloud, Eigen::Vector4d &coeff,
                                     const PosTPointCloud::Ptr &cloudInliers) {

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<PosTPoint> seg;

        // Optional
        seg.setOptimizeCoefficients(true);

        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);

        // solve
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < 20) {
            return false;
        }

        for (int i = 0; i < 4; i++) {
            coeff(i) = coefficients->values[i];
        }

        pcl::copyPointCloud<PosTPoint>(*cloud, *inliers, *cloudInliers);
        return true;
    }

    const aligned_vector<SurfelPlane> &SurfelConstructor::GetSurfelPlanes() const {
        return _surfelPlanes;
    }

    SurfelConstructor::Ptr
    SurfelConstructor::Create(const pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr &ndt) {
        return std::make_shared<SurfelConstructor>(ndt);
    }

    void SurfelAssociation::AssociateScanToSurfel(const SurfelPlane &surfelPlane, const PosTPointCloud::Ptr &scan,
                                                  const double &radius, std::vector<std::vector<int>> &ringMasks) {

        // the box od this surfel plane
        Eigen::Vector3d boxMin = surfelPlane.boxMin;
        Eigen::Vector3d boxMax = surfelPlane.boxMax;
        // the plane coeff
        Eigen::Vector4d planeCoeff = surfelPlane.coeff;

        for (int j = 0; j < scan->height; j++) {
            std::vector<int> maskPerRing;
            for (int i = 0; i < scan->width; i++) {
                // for each point in range
                int index = j * scan->width + i;
                // const auto &p = scan->at(i, j);
                const auto &p = scan->points[index];
                if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z) && IsPointInBox(boxMin, boxMax, p)) {
                    // if point is in range
                    Eigen::Vector3d point(p.x, p.y, p.z);
                    // compute the distance
                    if (Point2PlaneDistance(point, planeCoeff) < radius) {
                        // if the distance is in range, save its index
                        maskPerRing.push_back(i);
                    }
                }
            }
            // end of one row (ring)
            ringMasks.push_back(maskPerRing);
        }
    }

    double SurfelAssociation::Point2PlaneDistance(const Eigen::Vector3d &pt, const Eigen::Vector4d &planeCoeff) {
        Eigen::Vector3d normal = planeCoeff.head<3>();
        double dist = pt.dot(normal) + planeCoeff(3);
        return abs(dist);
    }

    bool SurfelAssociation::IsPointInBox(const Eigen::Vector3d &boxMin,
                                         const Eigen::Vector3d &boxMax,
                                         const PosTPoint &p) {
        return p.x > boxMin[0] && p.x < boxMax[0] &&
               p.y > boxMin[1] && p.y < boxMax[1] &&
               p.z > boxMin[2] && p.z < boxMax[2];
    }

    const aligned_vector<ns_elic::SurfelPoint> &SurfelAssociation::GetSpointDownSampled() const {
        return _spointDownSampled;
    }

    aligned_vector<PointPlaneCorrespondence> SurfelAssociation::ConstructCorrespondence() const {
        const auto &surfelPlanes = _surfelConstructor->GetSurfelPlanes();

        aligned_vector<PointPlaneCorrespondence> correspondence;
        for (const auto &spoint: _spointDownSampled) {
            PointPlaneCorrespondence planePointCorr;
            planePointCorr.pointTimeStamp = spoint.timestamp;
            planePointCorr.planeCoeff = surfelPlanes.at(spoint.planeId).coeff;
            planePointCorr.point = spoint.pointInScan;

            correspondence.push_back(planePointCorr);
        }
        return correspondence;
    }

}// namespace ns_elic