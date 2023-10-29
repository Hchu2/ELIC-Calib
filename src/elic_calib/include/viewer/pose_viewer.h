//
// Created by csl on 10/7/22.
//

#ifndef LIC_CALIB_POSE_VIEWER_H
#define LIC_CALIB_POSE_VIEWER_H

#include "pcl/common/transforms.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "sensor/lidar.h"
#include "slam-scene-viewer/scene_viewer.h"
#include "thread"
#include "core/pose.hpp"
#include "core/surfel_association.h"
#include "util/type_define.hpp"

namespace ns_elic {

    struct PoseSeqDisplay {
        enum class Mode {
            ARROW, COORD, CAMERA, LiDAR, IMU
        };

        const aligned_vector<Posed> &poseSeq;
        Mode mode;
        ns_viewer::Colour colour;

        PoseSeqDisplay(const aligned_vector<Posed> &poseSeq, Mode mode,
                       const ns_viewer::Colour &colour = ns_viewer::Colour::Red())
                : poseSeq(poseSeq), mode(mode), colour(colour) {}
    };

    struct Viewer : public ns_viewer::SceneViewer {
    public:
        using Ptr = std::shared_ptr<Viewer>;
        using parent_type = ns_viewer::SceneViewer;

        explicit Viewer(const std::string &saveDir = "", const std::string &winName = "")
                : ns_viewer::SceneViewer(saveDir, winName) {
            // for better visualization
            COLOUR_WHEEL = ns_viewer::ColourWheel();
        }

        static auto Create(const std::string &saveDir = "", const std::string &winName = "") {
            return std::make_shared<Viewer>(saveDir, winName);
        }

    public:
        // for lidar frames

        void ShowLiDARFramesInMapWithColorWheel(const aligned_vector<LiDARFrame::Ptr> &lidarSeq);

        void ShowLiDARFramesInMapWithColorWheel(
                const aligned_vector<LiDARFrame::Ptr> &lidarSeq, const PoseSeqDisplay &seq
        );

        void ShowLiDARFramesInMapWithIntensity(const aligned_vector<LiDARFrame::Ptr> &lidarSeq);

        void ShowLiDARFramesInMapWithIntensity(
                const aligned_vector<LiDARFrame::Ptr> &lidarSeq, const PoseSeqDisplay &poseSeq
        );

    public:
        // for single lidar frame

        void ShowLiDARFrameWithColorWheel(const LiDARFrame::Ptr &frame, const std::string &id);

        void ShowLiDARFrameWithIntensity(const LiDARFrame::Ptr &frame, const std::string &id);

        pcl::PointCloud<pcl::PointXYZI>::Ptr ShowLiDARMapAlignedToGravity(
                const LiDARFrame::Ptr &map, const Eigen::Vector3f &gravity, const std::string &id,
                float min = std::numeric_limits<float>::lowest(), float max = std::numeric_limits<float>::max()
        );


    public:
        // for pose sequence
        void ShowPoseSequence(const aligned_vector<PoseSeqDisplay> &seq, float size = 0.3f);

    public:
        // for surfel
        void ShowSurfelPlanes(const aligned_vector<SurfelPlane> &surfPlanes, float size = 1.0f);

        void ShowSurfelPlanePoints(
                const aligned_vector<SurfelPlane> &surfelPlanes,
                const aligned_vector<aligned_vector<SurfelPoint>> &spointPerSurfel
        );

    public:

        template<class PointType>
        static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
        ToColorPointCloud(const typename pcl::PointCloud<PointType>::Ptr &cloudIn, const ns_viewer::Colour &colour) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>);
            cloudOut->resize(cloudIn->size());
#pragma omp parallel for num_threads(omp_get_max_threads())
            for (int i = 0; i < cloudIn->size(); ++i) {
                auto &postPoint = cloudIn->at(i);
                auto &colorPoint = cloudOut->at(i);

                colorPoint.x = postPoint.x;
                colorPoint.y = postPoint.y;
                colorPoint.z = postPoint.z;

                colorPoint.r = static_cast<std::uint8_t>(colour.r * 255.0f);
                colorPoint.g = static_cast<std::uint8_t>(colour.g * 255.0f);
                colorPoint.b = static_cast<std::uint8_t>(colour.b * 255.0f);
                colorPoint.a = static_cast<std::uint8_t>(colour.a * 255.0f);
            }
            return cloudOut;
        }

        template<class PointType>
        static pcl::PointCloud<pcl::PointXYZI>::Ptr
        ToIntensityPointCloud(const typename pcl::PointCloud<PointType>::Ptr &cloudIn) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
            cloudOut->resize(cloudIn->size());
#pragma omp parallel for num_threads(omp_get_max_threads())
            for (int i = 0; i < cloudIn->size(); ++i) {
                auto &postPoint = cloudIn->at(i);
                auto &colorPoint = cloudOut->at(i);

                const auto x = postPoint.x, y = postPoint.y, z = postPoint.z;

                if (IS_POINT_NAN(postPoint)) {
                    // this point is invalid
                    colorPoint.x = colorPoint.y = colorPoint.z = NAN;
                    colorPoint.intensity = -1.0f;
                } else {
                    colorPoint.x = x, colorPoint.y = y, colorPoint.z = z;
                    // make the distance to origin be the intensity
                    colorPoint.intensity = std::log10(std::sqrt(x * x + y * y + z * z));
                }
            }
            return cloudOut;
        }
    };


}


#endif //LIC_CALIB_POSE_VIEWER_H
