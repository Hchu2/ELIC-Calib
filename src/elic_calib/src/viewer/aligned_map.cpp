//
// Created by csl on 4/24/23.
//

#include "viewer/aligned_map.h"
#include <utility>

namespace ns_elic {

    AlignedMap::AlignedMap(const PosTPointCloud::Ptr &cloud, Eigen::Vector3d gravity)
            : _map(new pcl::PointCloud<pcl::PointXYZ>), _gravity(std::move(gravity)) {
        std::vector<int> indices;
        PosTPointCloud::Ptr removedNanCloud(new PosTPointCloud);
        pcl::removeNaNFromPointCloud(*cloud, *removedNanCloud, indices);
        pcl::copyPointCloud(*removedNanCloud, *_map);
    }

    AlignedMap::Ptr
    AlignedMap::Create(const PosTPointCloud::Ptr &points, const Eigen::Vector3d &gravity) {
        return std::make_shared<AlignedMap>(points, gravity);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr AlignedMap::ComputeAlignedMap() const {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
        cloudOut->reserve(_map->size());

        Eigen::Vector3d normGravity = _gravity.normalized();

#pragma omp parallel for num_threads(omp_get_max_threads())
        for (auto &postPoint: *_map) {
            const auto x = postPoint.x, y = postPoint.y, z = postPoint.z;
            double intensity = -Eigen::Vector3d(x, y, z).dot(normGravity);

            pcl::PointXYZI colorPoint;
            colorPoint.x = x, colorPoint.y = y, colorPoint.z = z;
            // compute intensity
            colorPoint.intensity = static_cast<float>(intensity);
            cloudOut->push_back(colorPoint);
        }
        return cloudOut;
    }

    void AlignedMap::ShowAlignedMap(ns_viewer::SceneViewer &viewer, float pointSize, float min, float max) const {

        auto cloudAligned = ComputeAlignedMap();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
        cloudOut->reserve(cloudAligned->size());
        for (const auto &p: cloudAligned->points) {
            if (p.intensity < min || p.intensity > max) {
                continue;
            } else {
                cloudOut->push_back(p);
            }
        }

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorHandler(cloudOut, "intensity");
        static std::size_t count = 0;
        auto id = "aligned map" + std::to_string(++count);
        // add point cloud
        viewer.GetViewer()->addPointCloud(cloudOut, colorHandler, id);
        viewer.GetViewer()->setPointCloudRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, pointSize, id
        );
        // gravity
        viewer.AddArrow(
                Eigen::Vector3f::Zero(), _gravity.normalized().cast<float>()*2, ns_viewer::Colour::Black(), 5.0f
        );
    }

    void AlignedMap::ShowAlignedMap(const std::string &saveShotDir, float pointSize, float min, float max) const {
        ns_viewer::SceneViewer viewer(saveShotDir, "aligned map");
        ShowAlignedMap(viewer, pointSize, min, max);
        viewer.RunSingleThread();
    }

    void AlignedMap::Save(const std::string &filename, bool binaryMode) const {
        std::ofstream file(filename);
        if (binaryMode) {
            cereal::BinaryOutputArchive ar(file);
            ar(cereal::make_nvp("aligned_map", *this));
        } else {
            cereal::JSONOutputArchive ar(file);
            ar(cereal::make_nvp("aligned_map", *this));
        }
    }

    AlignedMap::Ptr AlignedMap::Load(const std::string &filename, bool binaryMode) {
        AlignedMap::Ptr alignedMap(new AlignedMap());
        std::ifstream file(filename);
        if (binaryMode) {
            cereal::BinaryInputArchive ar(file);
            ar(cereal::make_nvp("aligned_map", *alignedMap));
        } else {
            cereal::JSONInputArchive ar(file);
            ar(cereal::make_nvp("aligned_map", *alignedMap));
        }
        return alignedMap;
    }

    const pcl::PointCloud<pcl::PointXYZ>::Ptr &AlignedMap::GetMap() const {
        return _map;
    }

    const Eigen::Vector3d &AlignedMap::GetGravity() const {
        return _gravity;
    }
}