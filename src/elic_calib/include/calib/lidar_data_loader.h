//
// Created by csl on 10/4/22.
//

#ifndef LIC_CALIB_LIDAR_DATA_LOADER_H
#define LIC_CALIB_LIDAR_DATA_LOADER_H

#include "angles/angles.h"
#include "config/calib_config.h"
#include "iostream"
#include "pcl_ros/point_cloud.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor/lidar.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "vector"
#include "velodyne_msgs/VelodynePacket.h"
#include "velodyne_msgs/VelodyneScan.h"

namespace ns_elic {

    enum class LidarModelType {
        VLP_16_PACKET,
        VLP_16_SIMU,
        VLP_16_POINTS,
        VLP_32E_POINTS,

        VLP_LIVOX_SPLIT,

        LIVOX_HORIZON,
        LIVOX_MID40,
        LIVOX_MID70,
        LIVOX_MID70_SIMU,

        OUSTER_16_POINTS,
        OUSTER_32_POINTS,
        OUSTER_64_POINTS,
        OUSTER_128_POINTS,


        RS_16,
        RS_M1,
    };

    class LiDARDataLoader {
    public:
        using Ptr = std::shared_ptr<LiDARDataLoader>;
    protected:
        LidarModelType _lidarModel;

    public:
        explicit LiDARDataLoader(LidarModelType lidarModel) : _lidarModel(lidarModel) {}

        virtual LiDARFrame::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) = 0;

    };

    class Velodyne16 : public LiDARDataLoader {
    public:
        using Ptr = std::shared_ptr<Velodyne16>;

    public:
        explicit Velodyne16(LidarModelType lidarModel);

        static Velodyne16::Ptr Create(LidarModelType lidarModel);

        // for Velodyne16 ['VLP_16_SIMU' and 'VLP_16_PACKET']
        LiDARFrame::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;

    protected:

        [[nodiscard]] LiDARFrame::Ptr UnpackScan(const velodyne_msgs::VelodyneScan::ConstPtr &lidarMsg) const;

        [[nodiscard]] LiDARFrame::Ptr UnpackScan(const sensor_msgs::PointCloud2::ConstPtr &lidarMsg) const;

        [[nodiscard]] double GetExactTime(int dsr, int firing) const;

        void SetParameters();

        [[nodiscard]] bool PointInRange(float range) const;

    private:
        static const int RAW_SCAN_SIZE = 3;
        static const int SCANS_PER_BLOCK = 32;
        static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);
        constexpr static const float ROTATION_RESOLUTION = 0.01f;
        static const uint16_t ROTATION_MAX_UNITS = 36000u;
        constexpr static const float DISTANCE_RESOLUTION = 0.002f;

        static const uint16_t UPPER_BANK = 0xeeff;
        static const uint16_t LOWER_BANK = 0xddff;

        static const int BLOCKS_PER_PACKET = 12;
        static const int PACKET_STATUS_SIZE = 2;

        int FIRINGS_PER_BLOCK{};
        int SCANS_PER_FIRING{};
        float BLOCK_TDURATION{};
        float DSR_TOFFSET{};
        float FIRING_TOFFSET{};
        float PACKET_TIME{};

        float SIN_ROT_TABLE[ROTATION_MAX_UNITS]{};
        float COS_ROT_TABLE[ROTATION_MAX_UNITS]{};
        float COS_VERT_ANGLE[16]{};
        float SIN_VERT_ANGLE[16]{};
        int SCAN_MAPPING_16[16]{};

        typedef struct RawBlock {
            uint16_t header;    ///< UPPER_BANK or LOWER_BANK
            uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
            uint8_t data[BLOCK_DATA_SIZE];
        } RAW_BLOCK_T;

        union TwoBytes {
            uint16_t uint;
            uint8_t bytes[2];
        };

        union FourBytes {
            uint32_t uint32;
            float_t float32;
        };

        typedef struct RawPacket {
            RAW_BLOCK_T blocks[BLOCKS_PER_PACKET];
            uint32_t revolution;
            uint8_t status[PACKET_STATUS_SIZE];
        } RAW_PACKET_T;

        /** configuration parameters */
        typedef struct {
            double maxRange;  // maximum range to publish
            double minRange;
            int minAngle;  // minimum angle to publish
            int maxAngle;  // maximum angle to publish
        } Config;
        Config CONFIG{};

        double VLP16_TIME_BLOCK[1824][16]{};
    };

    class VelodynePoints : public LiDARDataLoader {
    public:
        using Ptr = std::shared_ptr<VelodynePoints>;

        explicit VelodynePoints(LidarModelType lidarModel);

        static VelodynePoints::Ptr Create(LidarModelType lidarModel);

        static bool CheckCloudField(
                const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, const std::string &desc
        );

        static double ClockwiseAngle(double befAngle, double afterAngle);

        static double RotationTravelledClockwise(double nowAngle, bool resetCnt = false);

        bool InitScanParam(const sensor_msgs::PointCloud2::ConstPtr &lidarMsg);

        LiDARFrame::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;

    private:
        int NUM_FIRING;
        int NUM_LASERS;

        bool FIRST_MSG;
        bool HAS_TIME_FIELD;
        bool HAS_RING_FIELD;

        double HORIZON_RESOLUTION;
        double ONE_SCAN_ANGLE;

        int laserID_mapping[32]{};
    };

    class OusterLiDAR : public LiDARDataLoader {
    public:
        using Ptr = std::shared_ptr<OusterLiDAR>;

    public:
        explicit OusterLiDAR(LidarModelType lidarModel);

        static OusterLiDAR::Ptr Create(LidarModelType lidarModel);

        LiDARFrame::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;

    private:

        int NUM_FIRING;
    };

    class LivoxLiDAR : public LiDARDataLoader {
    public:
        using Ptr = std::shared_ptr<LivoxLiDAR>;

    public:
        explicit LivoxLiDAR(LidarModelType lidarModel);

        static LivoxLiDAR::Ptr Create(LidarModelType lidarModel);

        LiDARFrame::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;

    private:
        int NUM_POINTS;
    };
}


#endif //LIC_CALIB_LIDAR_DATA_LOADER_H
