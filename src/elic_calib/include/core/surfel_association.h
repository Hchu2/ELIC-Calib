//
// Created by csl on 10/5/22.
//

#ifndef LIC_CALIB_SURFELASSOCIATION_H
#define LIC_CALIB_SURFELASSOCIATION_H

#include "ostream"
#include "pcl/segmentation/impl/sac_segmentation.hpp"
#include "pcl/segmentation/sac_segmentation.h"
#include "core/lidar_odometer.h"
#include "util/utils.hpp"
#include "utility"

namespace ns_elic {

    struct SurfelPoint {
    public:
        double timestamp{};
        Eigen::Vector3d pointInScan;
        Eigen::Vector3d pointInRef;
        std::size_t planeId{};

    public:
        SurfelPoint() = default;

        SurfelPoint(double timestamp, Eigen::Vector3d pointInScan, size_t planeId)
                : timestamp(timestamp), pointInScan(std::move(pointInScan)), planeId(planeId) {}

        friend std::ostream &operator<<(std::ostream &os, const SurfelPoint &point) {
            os << "timestamp: " << point.timestamp << ", pointInScan: " << point.pointInScan.transpose()
               << ", pointInRef: " << point.pointInRef.transpose() << ", planeId: " << point.planeId;
            return os;
        }
    };

    struct SurfelPlane {
    public:
        // for a plane: Ax + By + Cz + D = 0, and A * A + B * B + C * C = 1
        // coeff = [A, B, C, D]^T
        Eigen::Vector4d coeff;
        // pi = -D * [A, B, C]^T, pi is in this plane, and is the closest point to the origin point
        // -D * (A * A + B * B + C * C) + D = 0
        Eigen::Vector3d Pi;
        Eigen::Vector3d boxMin;
        Eigen::Vector3d boxMax;
        PosTPointCloud cloud;
        PosTPointCloud cloudInlier;


    public:
        SurfelPlane() = default;

        SurfelPlane(Eigen::Vector4d planeCoeff, PosTPointCloud cloud, PosTPointCloud cloudInlier)
                : coeff(std::move(planeCoeff)), cloud(std::move(cloud)), cloudInlier(std::move(cloudInlier)) {

            Pi = -this->coeff(3) * this->coeff.head<3>();

            // find box min and max
            PosTPoint min, max;
            pcl::getMinMax3D<PosTPoint>(this->cloud, min, max);
            boxMin = Eigen::Vector3d(min.x, min.y, min.z);
            boxMax = Eigen::Vector3d(max.x, max.y, max.z);
        }

        friend std::ostream &operator<<(std::ostream &os, const SurfelPlane &plane) {
            os << "coeff: " << plane.coeff.transpose() << ", Pi: " << plane.Pi.transpose()
               << ", boxMin: " << plane.boxMin.transpose() << ", boxMax: " << plane.boxMax.transpose()
               << ", cloud: " << plane.cloud.size() << ", cloudInlier: " << plane.cloudInlier.size();
            return os;
        }

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            archive(
                    CEREAL_NVP(coeff), CEREAL_NVP(Pi), CEREAL_NVP(boxMin),
                    CEREAL_NVP(boxMax), CEREAL_NVP(cloud), CEREAL_NVP(cloudInlier)
            );
        }
    };

    struct PointPlaneCorrespondence {
    public:
        Eigen::Vector4d planeCoeff;
        Eigen::Vector3d point;
        double pointTimeStamp{};

        PointPlaneCorrespondence() = default;

        PointPlaneCorrespondence(Eigen::Vector4d planeCoeff, Eigen::Vector3d point, double pointTimeStamp)
                : planeCoeff(std::move(planeCoeff)), point(std::move(point)), pointTimeStamp(pointTimeStamp) {}

        friend std::ostream &operator<<(std::ostream &os, const PointPlaneCorrespondence &correspondence) {
            os << "planeCoeff: " << correspondence.planeCoeff.transpose()
               << ", point: " << correspondence.point.transpose()
               << ", pointTimeStamp: " << correspondence.pointTimeStamp;
            return os;
        }
    };

    struct SurfelConstructor {
    public:
        using Ptr = std::shared_ptr<SurfelConstructor>;

    protected:
        aligned_vector<SurfelPlane> _surfelPlanes;

    public:
        explicit SurfelConstructor(const pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr &ndt);

        static SurfelConstructor::Ptr
        Create(const pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr &ndt);

        [[nodiscard]] const aligned_vector<SurfelPlane> &GetSurfelPlanes() const;

    protected:
        void CreateSurfelPlanes(const pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr &ndt);

        static int CheckPlaneType(const Eigen::Vector3d &eigenValues, const Eigen::Matrix3d &eigenVectors);

        static bool FitPlane(
                const PosTPointCloud::Ptr &cloud, Eigen::Vector4d &coeff, const PosTPointCloud::Ptr &cloudInliers
        );
    };

    class SurfelAssociation {
    public:
        using Ptr = std::shared_ptr<SurfelAssociation>;

    private:
        aligned_vector<aligned_vector<SurfelPoint>> _spointPerSurfel;

        aligned_vector<SurfelPoint> _spointAll;

        aligned_vector<SurfelPoint> _spointDownSampled;

        SurfelConstructor::Ptr _surfelConstructor;
    public:

        SurfelAssociation() = default;

        static SurfelAssociation::Ptr Create();

        void SetUp(const SurfelConstructor::Ptr &surfelConstructor,
                   const ns_elic::aligned_vector<LiDARFrame::Ptr> &lidarFrames,
                   const ns_elic::aligned_vector<LiDARFrame::Ptr> &lidarFramesInRef, bool Is_simu);

        [[nodiscard]] const aligned_vector<ns_elic::SurfelPlane> &GetSurfelPlanes() const;

        [[nodiscard]] const aligned_vector<ns_elic::aligned_vector<ns_elic::SurfelPoint>> &GetSpointPerSurfel() const;

        [[nodiscard]] const aligned_vector<SurfelPoint> &GetSpointAll() const;

        [[nodiscard]] const aligned_vector<ns_elic::SurfelPoint> &GetSpointDownSampled() const;

        [[nodiscard]] aligned_vector<PointPlaneCorrespondence> ConstructCorrespondence() const;

    protected:

        void CreateAssociation(const LiDARFrame::Ptr &lidarFrame, const LiDARFrame::Ptr &lidarFrameInRef,
                               bool is_dense = false);

    protected:

        static void AssociateScanToSurfel(
                const SurfelPlane &surfelPlane, const PosTPointCloud::Ptr &scan,
                const double &radius, std::vector<std::vector<int>> &ringMasks
        );

        static double Point2PlaneDistance(const Eigen::Vector3d &pt, const Eigen::Vector4d &planeCoeff);

        static bool IsPointInBox(const Eigen::Vector3d &boxMin, const Eigen::Vector3d &boxMax, const PosTPoint &p);

    };
}


#endif //LIC_CALIB_SURFELASSOCIATION_H
