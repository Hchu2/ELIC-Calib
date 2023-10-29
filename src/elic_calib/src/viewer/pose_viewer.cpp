//
// Created by csl on 10/7/22.
//

#include "viewer/pose_viewer.h"
#include "pcl/visualization/point_cloud_color_handlers.h"
#include "thirdparty/logger/src/include/logger.h"

namespace ns_elic {
    // for lidar frames

    void Viewer::ShowLiDARFramesInMapWithColorWheel(const aligned_vector<LiDARFrame::Ptr> &lidarSeq) {

        for (int i = 0; i < lidarSeq.size(); ++i) {
            _viewer->addPointCloud(
                    ToColorPointCloud<PosTPoint>(lidarSeq.at(i)->GetScan(), COLOUR_WHEEL.GetUniqueColour()),
                    "Scan-" + std::to_string(i));
        }
    }

    void Viewer::ShowLiDARFramesInMapWithColorWheel(const aligned_vector<LiDARFrame::Ptr> &lidarSeq,
                                                    const PoseSeqDisplay &seq) {

        for (int i = 0; i < lidarSeq.size(); ++i) {
            PosTPointCloud::Ptr transformCloud(new PosTPointCloud);
            pcl::transformPointCloud(*lidarSeq.at(i)->GetScan(), *transformCloud, seq.poseSeq.at(i).T().cast<float>());

            // add point cloud
            _viewer->addPointCloud(
                    ToColorPointCloud<PosTPoint>(transformCloud, COLOUR_WHEEL.GetUniqueColour()),
                    "Scan-" + std::to_string(i));
            if (seq.mode == PoseSeqDisplay::Mode::COORD) {
                // add coordinate system
                Eigen::Isometry3d curLtoM(seq.poseSeq.at(i).so3.unit_quaternion());
                curLtoM.pretranslate(seq.poseSeq.at(i).t);

                _viewer->addCoordinateSystem(
                        0.5f,
                        Eigen::Affine3f(curLtoM.cast<float>().affine()),
                        "ScanCoord-" + std::to_string(i));
            } else if (seq.mode == PoseSeqDisplay::Mode::ARROW) {
                const auto &pose = seq.poseSeq.at(i);

                Eigen::Vector3d dirVec = Eigen::AngleAxisd(pose.so3.unit_quaternion()).axis();
                auto trans = pose.t;

                const auto &c = seq.colour;

                // add arrow
                pcl::PointXYZ from(trans(0), trans(1), trans(2));
                pcl::PointXYZ to(trans(0) + dirVec(0), trans(1) + dirVec(1), trans(2) + dirVec(2));

                _viewer->addArrow(to, from, c.r, c.g, c.b, false, "Arrow-" + std::to_string(i));
            }
        }
    }

    void Viewer::ShowLiDARFramesInMapWithIntensity(const aligned_vector<LiDARFrame::Ptr> &lidarSeq) {
        static int index = 0;
        for (int i = 0; i < lidarSeq.size(); ++i) {

            auto cloud = ToIntensityPointCloud<PosTPoint>(lidarSeq.at(i)->GetScan());
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorHandler(cloud, "intensity");

            // add point cloud
            _viewer->addPointCloud(cloud, colorHandler, "Scan-" + std::to_string(index));
            index++;
        }
    }

    void Viewer::ShowLiDARFramesInMapWithIntensity(const aligned_vector<LiDARFrame::Ptr> &lidarSeq,
                                                   const PoseSeqDisplay &seq) {
        static int index = 0;
        for (int i = 0; i < lidarSeq.size(); ++i) {
            PosTPointCloud::Ptr transformCloud(new PosTPointCloud);
            pcl::transformPointCloud(*lidarSeq.at(i)->GetScan(), *transformCloud, seq.poseSeq.at(i).T().cast<float>());

            auto cloud = ToIntensityPointCloud<PosTPoint>(transformCloud);
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorHandler(cloud, "intensity");

            // add point cloud

            _viewer->addPointCloud(cloud, colorHandler, "Scan-" + std::to_string(index));
            index++;

            if (seq.mode == PoseSeqDisplay::Mode::COORD) {
                // add coordinate system
                Eigen::Isometry3d curLtoM(seq.poseSeq.at(i).so3.unit_quaternion());
                curLtoM.pretranslate(seq.poseSeq.at(i).t);

                _viewer->addCoordinateSystem(
                        0.5f,
                        Eigen::Affine3f(curLtoM.cast<float>().affine()),
                        "ScanCoord-" + std::to_string(index));
            } else if (seq.mode == PoseSeqDisplay::Mode::ARROW) {
                const auto &pose = seq.poseSeq.at(i);

                Eigen::Vector3d dirVec = Eigen::AngleAxisd(pose.so3.unit_quaternion()).axis();
                auto trans = pose.t;

                const auto &c = seq.colour;

                // add arrow
                pcl::PointXYZ from(trans(0), trans(1), trans(2));
                pcl::PointXYZ to(trans(0) + dirVec(0), trans(1) + dirVec(1), trans(2) + dirVec(2));

                _viewer->addArrow(
                        to, from, c.r, c.g, c.b, false, "Arrow-" + std::to_string(i));
            }
        }
    }

    // for single lidar frame

    void Viewer::ShowLiDARFrameWithIntensity(const LiDARFrame::Ptr &frame, const std::string &id) {

        auto cloud = ToIntensityPointCloud<PosTPoint>(frame->GetScan());
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorHandler(cloud, "intensity");

        // add point cloud
        _viewer->addPointCloud(cloud, colorHandler, id);
    }

    void Viewer::ShowLiDARFrameWithColorWheel(const LiDARFrame::Ptr &frame, const std::string &id) {

        auto cloud = ToColorPointCloud<PosTPoint>(frame->GetScan(), COLOUR_WHEEL.GetUniqueColour());
        // add point cloud
        _viewer->addPointCloud(cloud, id);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    Viewer::ShowLiDARMapAlignedToGravity(const LiDARFrame::Ptr &map, const Eigen::Vector3f &gravity,
                                         const std::string &id, float min, float max) {
        // remove nan point
        std::vector<int> indices;
        PosTPointCloud::Ptr cloudIn(new PosTPointCloud);
        pcl::removeNaNFromPointCloud(*map->GetScan(), *cloudIn, indices);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
        cloudOut->reserve(cloudIn->size());

        Eigen::Vector3f normGravity = gravity.normalized();

#pragma omp parallel for num_threads(omp_get_max_threads())
        for (auto &postPoint: *cloudIn) {
            const auto x = postPoint.x, y = postPoint.y, z = postPoint.z;
            float intensity = -Eigen::Vector3f(x, y, z).dot(normGravity);

            if (intensity < min || intensity > max) {
                continue;
            } else {
                pcl::PointXYZI colorPoint;
                colorPoint.x = x, colorPoint.y = y, colorPoint.z = z;
                // compute intensity
                colorPoint.intensity = intensity;
                cloudOut->push_back(colorPoint);
            }
        }

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorHandler(cloudOut, "intensity");
        // add point cloud
        _viewer->addPointCloud(cloudOut, colorHandler, id);

        return cloudOut;
    }

    // for pose sequence

    void Viewer::ShowPoseSequence(const aligned_vector<PoseSeqDisplay> &seq, float size) {

        for (int j = 0; j < seq.size(); ++j) {
            const auto &s = seq.at(j);
            const auto &poseSeq = s.poseSeq;

            const auto &c = s.colour;

            if (s.mode == PoseSeqDisplay::Mode::ARROW) {
                for (int i = 0; i < poseSeq.size(); ++i) {
                    const auto &pose = poseSeq.at(i);

                    Eigen::Vector3d dirVec = Eigen::AngleAxisd(pose.so3.unit_quaternion()).axis();
                    auto trans = pose.t;

                    // add arrow
                    pcl::PointXYZ from(trans(0), trans(1), trans(2));
                    pcl::PointXYZ to(trans(0) + dirVec(0), trans(1) + dirVec(1), trans(2) + dirVec(2));

                    _viewer->addArrow(
                            to, from, c.r, c.g, c.b, false, "Arrow-" + std::to_string(j) + '-' + std::to_string(i));
                }
            } else if (s.mode == PoseSeqDisplay::Mode::COORD) {
                for (const auto &pose: poseSeq) {
                    AddPose(ns_viewer::Posed(pose.R(), pose.t).cast<float>(), size * 0.2f);
                }
            } else if (s.mode == PoseSeqDisplay::Mode::CAMERA) {
                for (const auto &pose: poseSeq) {
                    AddCamera(ns_viewer::Posed(pose.R(), pose.t).cast<float>(), s.colour, size * 0.2f);
                }
            } else if (s.mode == PoseSeqDisplay::Mode::LiDAR) {
                for (const auto &pose: poseSeq) {
                    AddLiDAR(ns_viewer::Posed(pose.R(), pose.t).cast<float>(), s.colour, size * 0.2f);
                }
            } else if (s.mode == PoseSeqDisplay::Mode::IMU) {
                for (const auto &pose: poseSeq) {
                    AddIMU(ns_viewer::Posed(pose.R(), pose.t).cast<float>(), s.colour, size * 0.2f);
                }
            } else {
                throw Status(Status::Flag::ERROR, "Pose sequence display mode is unknown.");
            }
        }
    }

    // for surfel

    void Viewer::ShowSurfelPlanes(const ns_elic::aligned_vector<ns_elic::SurfelPlane> &surfPlanes, float size) {
        for (int i = 0; i < surfPlanes.size(); ++i) {
            const auto &surfPlane = surfPlanes.at(i);
            PosTPointCloud::Ptr cloud(new PosTPointCloud);
            *cloud = surfPlane.cloudInlier;
            std::string name = "SurfelPlane-" + std::to_string(i);
            // add point cloud
            _viewer->addPointCloud(ToColorPointCloud<PosTPoint>(cloud, COLOUR_WHEEL.GetUniqueColour()), name);
            _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
        }
    }

    void Viewer::ShowSurfelPlanePoints(const aligned_vector<SurfelPlane> &surfelPlanes,
                                       const aligned_vector<aligned_vector<SurfelPoint>> &spointPerSurfel) {

        if (surfelPlanes.size() != spointPerSurfel.size()) {
            throw Status(
                    Status::Flag::FETAL,
                    "the sizes of 'surfelPlanes' and 'spointPerSurfel' are different in function 'ShowSurfelPlanePoints'");
        }

        for (int i = 0; i < surfelPlanes.size(); ++i) {
            const auto &splane = surfelPlanes.at(i);
            PosTPointCloud::Ptr cloud(new PosTPointCloud);
            *cloud = splane.cloudInlier;
            // add point cloud
            _viewer->addPointCloud(
                    ToColorPointCloud<PosTPoint>(cloud, COLOUR_WHEEL.GetUniqueColour()),
                    "SurfelPlane-" + std::to_string(i));
            _viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "SurfelPlane-" + std::to_string(i));

            const auto &spoints = spointPerSurfel.at(i);

            PosTPointCloud::Ptr surfelCloud(new PosTPointCloud);
            surfelCloud->resize(spoints.size());

            for (int j = 0; j < spoints.size(); ++j) {
                const auto &spoint = spoints.at(j);
                PosTPoint p;
                p.x = static_cast<float>(spoint.pointInRef(0));
                p.y = static_cast<float>(spoint.pointInRef(1));
                p.z = static_cast<float>(spoint.pointInRef(2));
                p.timestamp = spoint.timestamp;

                surfelCloud->at(j) = p;
            }

            _viewer->addPointCloud(
                    ToColorPointCloud<PosTPoint>(surfelCloud, COLOUR_WHEEL.GetUniqueColour()),
                    "SurfelPoints-" + std::to_string(i));
            _viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8.0, "SurfelPoints-" + std::to_string(i));
        }
    }
}// namespace ns_elic