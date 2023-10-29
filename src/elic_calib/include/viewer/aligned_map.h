//
// Created by csl on 4/24/23.
//

#ifndef ELIC_CALIB_ALIGNED_MAP_H
#define ELIC_CALIB_ALIGNED_MAP_H

#include "sensor/lidar.h"
#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "opencv2/imgcodecs.hpp"
#include "cereal/types/optional.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/array.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/types/polymorphic.hpp"
#include "viewer/pose_viewer.h"
#include "viewer/color_mapping.hpp"
#include "util/utils.hpp"

namespace ns_elic {
    struct AlignedMap {
    public:
        using Ptr = std::shared_ptr<AlignedMap>;

    protected:
        // construct in Ref
        pcl::PointCloud<pcl::PointXYZ>::Ptr _map;
        Eigen::Vector3d _gravity;

    public:
        AlignedMap(const PosTPointCloud::Ptr &cloud, Eigen::Vector3d gravity);

        AlignedMap() : _map(new pcl::PointCloud<pcl::PointXYZ>) {}

        static Ptr Create(const PosTPointCloud::Ptr &cloud, const Eigen::Vector3d &gravity);

        void ShowAlignedMap(ns_viewer::SceneViewer &viewer, float pointSize = 1,
                            float min = std::numeric_limits<float>::lowest(),
                            float max = std::numeric_limits<float>::max()) const;


        void ShowAlignedMap(const std::string &saveShotDir = "", float pointSize = 1,
                            float min = std::numeric_limits<float>::lowest(),
                            float max = std::numeric_limits<float>::max()) const;


        void Save(const std::string &filename, bool binaryMode = true) const;

        static Ptr Load(const std::string &filename, bool binaryMode = true);

        [[nodiscard]] const pcl::PointCloud<pcl::PointXYZ>::Ptr &GetMap() const;

        const Eigen::Vector3d &GetGravity() const;

    protected:
        [[nodiscard]] pcl::PointCloud<pcl::PointXYZI>::Ptr ComputeAlignedMap() const;

    public:
        // Serialization, make sure this function be public
        template<class Archive>
        void serialize(Archive &archive) {
            archive(cereal::make_nvp("map", *_map), cereal::make_nvp("gravity", _gravity));
        }
    };
}


#endif //ELIC_CALIB_ALIGNED_MAP_H
