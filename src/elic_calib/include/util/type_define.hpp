//
// Created by csl on 10/2/22.
//

#ifndef LIC_CALIB_TYPE_DEFINE_HPP
#define LIC_CALIB_TYPE_DEFINE_HPP

#include "Eigen/Dense"
#include "deque"
#include "map"
#include "pcl/pcl_macros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "unordered_map"
#include "unordered_set"
#include "vector"

struct EIGEN_ALIGN16 PointXYZIRT {
    PCL_ADD_POINT4D;              // quad-word XYZ
    float intensity;              // laser intensity reading
    std::uint16_t ring;           // laser ring number
    float time;                   // laser time reading
    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,                          //
                                  (float, x, x)                         //
                                          (float, y, y)                 //
                                          (float, z, z)                 //
                                          (float, intensity, intensity) //
                                          (std::uint16_t, ring, ring)   //
                                          (float, time, time))          //

struct EIGEN_ALIGN16 PointXYZT {
    PCL_ADD_POINT4D;              // quad-word XYZ
    double timestamp;             // laser timestamp
    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT,                              //
                                  (float, x, x)                           //
                                          (float, y, y)                   //
                                          (float, z, z)                   //
                                          (double, timestamp, timestamp)) //

// OUSTER lidar
struct EIGEN_ALIGN16 PointXYZIR8Y {
    PCL_ADD_POINT4D;   // quad-word XYZ
    float intensity;   // laser intensity reading
    std::uint8_t ring; // laser ring number
    std::uint32_t t;
    float range;

    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR8Y,                         //
                                  (float, x, x)                         //
                                          (float, y, y)                 //
                                          (float, z, z)                 //
                                          (float, intensity, intensity) //
                                          (std::uint8_t, ring, ring)    //
                                          (std::uint32_t, t, t))        //

namespace ns_elic {
    // pcl
    using PosIPoint = pcl::PointXYZI;
    using PosIPointCloud = pcl::PointCloud<PosIPoint>;

    using PosIRTPoint = PointXYZIRT;
    using PosIRTPointCloud = pcl::PointCloud<PosIRTPoint>;

    using PosTPoint = PointXYZT;
    using PosTPointCloud = pcl::PointCloud<PosTPoint>;

    using OusterPoint = PointXYZIR8Y;
    using OusterPointCloud = pcl::PointCloud<OusterPoint>;

    using ColorPoint = pcl::PointXYZRGB;
    using ColorPointCloud = pcl::PointCloud<ColorPoint>;

    //using LivoxPoint = pcl::PointXYZT;
    using LivoxPointCloud = pcl::PointCloud<PosTPoint>;

    // eigen

    template<typename T>
    using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

    template<typename T>
    using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

    template<typename K, typename V>
    using aligned_map = std::map<K, V, std::less<K>,
            Eigen::aligned_allocator<std::pair<K const, V>>>;

    template<typename K, typename V>
    using aligned_unordered_map = std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
            Eigen::aligned_allocator<std::pair<K const, V>>>;
}

namespace Eigen {
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector9d = Eigen::Matrix<double, 9, 1>;
}


#endif //LIC_CALIB_TYPE_DEFINE_HPP
