//
// Created by csl on 10/4/22.
//

#include "calib/lidar_data_loader.h"
#include "util/status.hpp"

namespace ns_elic {

    // ----------
    // Velodyne16
    // ----------
    Velodyne16::Velodyne16(LidarModelType lidarModel) : LiDARDataLoader(lidarModel) { SetParameters(); }

    Velodyne16::Ptr Velodyne16::Create(LidarModelType lidarModel) {
        return std::make_shared<Velodyne16>(lidarModel);
    }

    LiDARFrame::Ptr
    Velodyne16::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        if (_lidarModel == LidarModelType::VLP_16_SIMU) {
            sensor_msgs::PointCloud2::ConstPtr scanMsg = msgInstance.instantiate<sensor_msgs::PointCloud2>();
            return UnpackScan(scanMsg);
        } else if (_lidarModel == LidarModelType::VLP_16_PACKET) {
            velodyne_msgs::VelodyneScan::ConstPtr scanMsg = msgInstance.instantiate<velodyne_msgs::VelodyneScan>();
            return UnpackScan(scanMsg);
        } else {
            throw Status(
                    Status::Flag::FETAL,
                    "'Velodyne16' data loader only supports the lidar type: 'VLP_16_SIMU' and 'VLP_16_PACKET'");
        }
    }

    LiDARFrame::Ptr
    Velodyne16::UnpackScan(const velodyne_msgs::VelodyneScan_<std::allocator<void>>::ConstPtr &lidarMsg) const {

        LiDARFrame::Ptr output = LiDARFrame::Create(lidarMsg->header.stamp.toSec());

        // point cloud
        output->GetScan()->height = 16;
        output->GetScan()->width = 24 * (int) lidarMsg->packets.size();
        output->GetScan()->is_dense = false;
        output->GetScan()->resize(output->GetScan()->height * output->GetScan()->width);

        int blockCounter = 0;

        double scanTimestamp = lidarMsg->header.stamp.toSec();

        for (const auto &packet: lidarMsg->packets) {
            float azimuth;
            float azimuthDiff;
            float lastAzimuthDiff = 0;
            float azimuthCorrectedF;
            int azimuthCorrected;
            float x, y, z;

            const auto *raw = (const RAW_PACKET_T *) &packet.data[0];

            for (int block = 0; block < BLOCKS_PER_PACKET; block++, blockCounter++) {
                // Calculate difference between current and next block's azimuth angle.
                azimuth = (float) (raw->blocks[block].rotation);

                if (block < (BLOCKS_PER_PACKET - 1)) {
                    azimuthDiff =
                            (float) ((ROTATION_MAX_UNITS + raw->blocks[block + 1].rotation -
                                      raw->blocks[block].rotation) %
                                     ROTATION_MAX_UNITS);
                    lastAzimuthDiff = azimuthDiff;
                } else {
                    azimuthDiff = lastAzimuthDiff;
                }

                for (int firing = 0, k = 0; firing < FIRINGS_PER_BLOCK; firing++) {
                    for (int dsr = 0; dsr < SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {
                        /** Position Calculation */
                        union TwoBytes tmp {};
                        tmp.bytes[0] = raw->blocks[block].data[k];
                        tmp.bytes[1] = raw->blocks[block].data[k + 1];

                        /** correct for the laser rotation as a function of timing during
                         * the firings **/
                        azimuthCorrectedF =
                                azimuth + (azimuthDiff * ((dsr * DSR_TOFFSET) + (firing * FIRING_TOFFSET)) /
                                           BLOCK_TDURATION);

                        azimuthCorrected = ((int) round(azimuthCorrectedF)) % ROTATION_MAX_UNITS;

                        /*condition added to avoid calculating points which are not
                      in the interesting defined area (minAngle < area < maxAngle)*/
                        if ((azimuthCorrected >= CONFIG.minAngle &&
                             azimuthCorrected <= CONFIG.maxAngle &&
                             CONFIG.minAngle < CONFIG.maxAngle) ||
                            (CONFIG.minAngle > CONFIG.maxAngle &&
                             (azimuthCorrected <= CONFIG.maxAngle ||
                              azimuthCorrected >= CONFIG.minAngle))) {
                            // convert polar coordinates to Euclidean XYZ
                            float distance = tmp.uint * DISTANCE_RESOLUTION;

                            float cos_vert_angle = COS_VERT_ANGLE[dsr];
                            float sin_vert_angle = SIN_VERT_ANGLE[dsr];

                            float cos_rot_angle = COS_ROT_TABLE[azimuthCorrected];
                            float sin_rot_angle = SIN_ROT_TABLE[azimuthCorrected];

                            x = distance * cos_vert_angle * sin_rot_angle;
                            y = distance * cos_vert_angle * cos_rot_angle;
                            z = distance * sin_vert_angle;

                            /** Use standard ROS coordinate system (right-hand rule) */
                            float x_coord = y;
                            float y_coord = -x;
                            float z_coord = z;

                            double point_timestamp =
                                    scanTimestamp +
                                    GetExactTime(dsr, 2 * blockCounter + firing);

                            // TODO: using macro 'set post point nan'
                            PosTPoint point_xyz;

                            if (PointInRange(distance)) {
                                point_xyz.timestamp = point_timestamp;
                                point_xyz.x = x_coord;
                                point_xyz.y = y_coord;
                                point_xyz.z = z_coord;
                            } else {
                                SET_POST_POINT_NAN(point_xyz)
                            }

                            output->GetScan()->at(2 * blockCounter + firing, SCAN_MAPPING_16[dsr]) = point_xyz;
                        }
                    }
                }
            }
        }

        return output;
    }

    LiDARFrame::Ptr
    Velodyne16::UnpackScan(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &lidarMsg) const {
        PosIPointCloud pointCloudIn;
        pcl::fromROSMsg(*lidarMsg, pointCloudIn);

        double timebase = lidarMsg->header.stamp.toSec();

        LiDARFrame::Ptr output = LiDARFrame::Create(timebase);

        /// point cloud
        output->GetScan()->clear();
        output->GetScan()->height = pointCloudIn.height;
        output->GetScan()->width = pointCloudIn.width;
        output->GetScan()->is_dense = false;
        output->GetScan()->resize(output->GetScan()->height * output->GetScan()->width);

        for (int w = 0; w < pointCloudIn.width; w++) {
            for (int dsr = 0; dsr < pointCloudIn.height; dsr++) {
                double deltaT = GetExactTime(dsr, w);

                PosTPoint point;
                // already ros-coordinate: x-right y-front z-up
                point.x = pointCloudIn.at(w, dsr).x;
                point.y = pointCloudIn.at(w, dsr).y;
                point.z = pointCloudIn.at(w, dsr).z;
                point.timestamp = timebase + deltaT;

                output->GetScan()->at(w, SCAN_MAPPING_16[dsr]) = point;
            }
        }

        return output;
    }

    double Velodyne16::GetExactTime(int dsr, int firing) const {
        return VLP16_TIME_BLOCK[firing][dsr];
    }

    void Velodyne16::SetParameters() {
        CONFIG.maxRange = 150;
        CONFIG.minRange = 1.0;
        CONFIG.minAngle = 0;
        CONFIG.maxAngle = 36000;
        // Set up cached values for sin and cos of all the possible headings
        for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
            float rotation = angles::from_degrees(ROTATION_RESOLUTION * rotIndex);
            COS_ROT_TABLE[rotIndex] = cosf(rotation);
            SIN_ROT_TABLE[rotIndex] = sinf(rotation);
        }

        FIRINGS_PER_BLOCK = 2;
        SCANS_PER_FIRING = 16;
        BLOCK_TDURATION = 110.592f;// [µs]
        DSR_TOFFSET = 2.304f;      // [µs]
        FIRING_TOFFSET = 55.296f;  // [µs]
        PACKET_TIME = (BLOCKS_PER_PACKET * 2 * FIRING_TOFFSET);

        float vertCorrection[16] = {
                -0.2617993877991494, 0.017453292519943295, -0.22689280275926285,
                0.05235987755982989, -0.19198621771937624, 0.08726646259971647,
                -0.15707963267948966, 0.12217304763960307, -0.12217304763960307,
                0.15707963267948966, -0.08726646259971647, 0.19198621771937624,
                -0.05235987755982989, 0.22689280275926285, -0.017453292519943295,
                0.2617993877991494};
        for (int i = 0; i < 16; i++) {
            COS_VERT_ANGLE[i] = std::cos(vertCorrection[i]);
            SIN_VERT_ANGLE[i] = std::sin(vertCorrection[i]);
        }

        SCAN_MAPPING_16[15] = 0;
        SCAN_MAPPING_16[13] = 1;
        SCAN_MAPPING_16[11] = 2;
        SCAN_MAPPING_16[9] = 3;
        SCAN_MAPPING_16[7] = 4;
        SCAN_MAPPING_16[5] = 5;
        SCAN_MAPPING_16[3] = 6;
        SCAN_MAPPING_16[1] = 7;

        SCAN_MAPPING_16[14] = 8;
        SCAN_MAPPING_16[12] = 9;
        SCAN_MAPPING_16[10] = 10;
        SCAN_MAPPING_16[8] = 11;
        SCAN_MAPPING_16[6] = 12;
        SCAN_MAPPING_16[4] = 13;
        SCAN_MAPPING_16[2] = 14;
        SCAN_MAPPING_16[0] = 15;

        for (unsigned int w = 0; w < 1824; w++) {
            for (unsigned int h = 0; h < 16; h++) {
                VLP16_TIME_BLOCK[w][h] = h * 2.304 * 1e-6 + w * 55.296 * 1e-6;//  16*1824
            }
        }
    }

    bool Velodyne16::PointInRange(float range) const {
        return (range >= CONFIG.minRange && range <= CONFIG.maxRange);
    }

    // --------------
    // VelodynePoints
    // --------------
    VelodynePoints::VelodynePoints(LidarModelType lidarModel)
        : LiDARDataLoader(lidarModel), FIRST_MSG(true),
          HAS_TIME_FIELD(false), HAS_RING_FIELD(false), ONE_SCAN_ANGLE(360.) {
        if (_lidarModel == LidarModelType::VLP_16_POINTS) {
            NUM_LASERS = 16;
            NUM_FIRING = 1824;// 76*24
        } else if (_lidarModel == LidarModelType::VLP_32E_POINTS) {
            NUM_LASERS = 32;
            NUM_FIRING = 2170;// 180*12
        } else {
            throw Status(
                    Status::Flag::FETAL,
                    "'VelodynePoints' data loader only supports the lidar type: 'VLP_16_POINTS' and 'VLP_32E_POINTS'");
        }
        HORIZON_RESOLUTION = ONE_SCAN_ANGLE / (double) NUM_FIRING;

        if (_lidarModel == LidarModelType::VLP_32E_POINTS) {
            // laserID(dsr channel)
            // channel 0,1,...,31
            laserID_mapping[31] = 0;
            laserID_mapping[29] = 1;
            laserID_mapping[27] = 2;
            laserID_mapping[25] = 3;
            laserID_mapping[23] = 4;
            laserID_mapping[21] = 5;
            laserID_mapping[19] = 6;
            laserID_mapping[17] = 7;
            laserID_mapping[15] = 8;
            laserID_mapping[13] = 9;
            laserID_mapping[11] = 10;
            laserID_mapping[9] = 11;
            laserID_mapping[7] = 12;
            laserID_mapping[5] = 13;
            laserID_mapping[3] = 14;
            laserID_mapping[1] = 15;

            laserID_mapping[30] = 16;
            laserID_mapping[28] = 17;
            laserID_mapping[26] = 18;
            laserID_mapping[24] = 19;
            laserID_mapping[22] = 20;
            laserID_mapping[20] = 21;
            laserID_mapping[18] = 22;
            laserID_mapping[16] = 23;
            laserID_mapping[14] = 24;
            laserID_mapping[12] = 25;
            laserID_mapping[10] = 26;
            laserID_mapping[8] = 27;
            laserID_mapping[6] = 28;
            laserID_mapping[4] = 29;
            laserID_mapping[2] = 30;
            laserID_mapping[0] = 31;
        }
    }

    VelodynePoints::Ptr VelodynePoints::Create(LidarModelType lidarModel) {
        return std::make_shared<VelodynePoints>(lidarModel);
    }

    bool VelodynePoints::CheckCloudField(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &cloud_msg,
                                         const std::string &desc) {
        bool has_the_field = false;
        for (const auto &field: cloud_msg->fields) {
            if (field.name == desc) {
                has_the_field = true;
                break;
            }
        }

        if (!has_the_field) {
            LOG_WARNING("PointCloud2 not has channel [", desc, "], please configure your point cloud data!");
        }
        return has_the_field;
    }

    double VelodynePoints::ClockwiseAngle(double befAngle, double afterAngle) {
        // atan2(py, px)
        // 1quadrant-1quadrant = 45 - 30
        // 1quadrant-4quadrant = 45 - (-30) = 75
        // 1quadrant-3quadrant = 45 - (-120) = 165
        // 1quadrant-2quadrant = 45 - 120 = -75 + 360 = 285
        double dAngle = befAngle - afterAngle;
        if (dAngle < 0) dAngle += 360;

        return dAngle;
    }

    double VelodynePoints::RotationTravelledClockwise(double nowAngle, bool resetCnt) {
        static double startAngleDegree = 0;
        static bool halfRotPointer = false;
        static double rotCircleCnt = 0;

        if (resetCnt) {
            startAngleDegree = nowAngle;
            halfRotPointer = false;
            rotCircleCnt = 0;

            return 0;
        } else {
            double dAngle = ClockwiseAngle(startAngleDegree, nowAngle);
            // half a circle
            if (dAngle > 100 && dAngle < 270) {
                halfRotPointer = true;
            }
            // a circle
            if (halfRotPointer && dAngle < 80) {
                halfRotPointer = false;
                rotCircleCnt += 360;
            }

            return rotCircleCnt + dAngle;// rot_travelled
        }
    }

    bool VelodynePoints::InitScanParam(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &lidarMsg) {
        HAS_TIME_FIELD = CheckCloudField(lidarMsg, "time");
        HAS_RING_FIELD = CheckCloudField(lidarMsg, "ring");

        // if (!HAS_RING_FIELD) return false;

        if (!HAS_TIME_FIELD) {
            LOG_WARNING("Input PointCloud2 not has channel [time]. Calculate timestamp "
                        "of each point assuming constant rotation speed");
        }

        PosIPointCloud cloud;
        pcl::fromROSMsg(*lidarMsg, cloud);

        bool is_first_horizon_angle = true;

        double rotation_travelled = 0;
        for (auto const &pointIn: cloud) {
            if (IS_POINT_NAN(pointIn))
                continue;
            static double rad2deg = 180.0 / M_PI;
            double horizon_angle = atan2(pointIn.y, pointIn.x) * rad2deg;
            if (is_first_horizon_angle) {
                is_first_horizon_angle = false;
                RotationTravelledClockwise(horizon_angle, true);
            }
            rotation_travelled = RotationTravelledClockwise(horizon_angle);
        }

        ONE_SCAN_ANGLE = round(rotation_travelled / 360.) * 363.;

        HORIZON_RESOLUTION = ONE_SCAN_ANGLE / (double) NUM_FIRING;

        LOG_VAR(ONE_SCAN_ANGLE, HORIZON_RESOLUTION)

        return true;
    }

    LiDARFrame::Ptr VelodynePoints::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        sensor_msgs::PointCloud2::ConstPtr lidarMsg = msgInstance.instantiate<sensor_msgs::PointCloud2>();

        if (FIRST_MSG) {
            InitScanParam(lidarMsg);
            FIRST_MSG = false;
        }

        static const double LIDAR_FOV_DOWN = -15.0;
        static const double LIDAR_FOV_RESOLUTION = 2.0;

        PosIRTPointCloud pcIn;
        pcl::fromROSMsg(*lidarMsg, pcIn);

        PosTPointCloud::Ptr cloud(new PosTPointCloud());
        double timebase = lidarMsg->header.stamp.toSec();

        /// point cloud
        cloud->clear();
        cloud->height = NUM_LASERS;
        cloud->width = NUM_FIRING;
        cloud->is_dense = false;
        cloud->resize(cloud->height * cloud->width);

        int num_points = NUM_LASERS * NUM_FIRING;
        for (int k = 0; k < num_points; k++) {
            SET_POST_POINT_NAN(cloud->points[k])
        }

        int lastFiring = 0;
        int firingOrderPositive = 0, firingOrderPositiveCnt = 0;
        int firingOrderReverse = 0, firingOrderReverseCnt = 0;

        bool isFirstHorizonAngle = true;
        for (int i = 0; i < pcIn.size(); ++i) {
            const PosIRTPoint &pointIn = pcIn.points[i];

            if (IS_POINT_NAN(pointIn))
                continue;
            static double RAD_2_DEG = 180.0 / M_PI;

            double horizonAngle = atan2(pointIn.y, pointIn.x) * RAD_2_DEG;
            if (isFirstHorizonAngle) {
                isFirstHorizonAngle = false;
                RotationTravelledClockwise(horizonAngle, true);
            }
            double rotationTravelled = RotationTravelledClockwise(horizonAngle);

            int firing = round(rotationTravelled / HORIZON_RESOLUTION);

            if (i > 0) {
                int df = firing - lastFiring;
                if (df >= 0) {
                    firingOrderPositive += df;
                    firingOrderPositiveCnt++;
                } else {
                    firingOrderReverse += df;
                    firingOrderReverseCnt++;
                }
            }
            lastFiring = firing;

            if (firing < 0 || firing >= NUM_FIRING) continue;

            double dt = 0;
            if (HAS_TIME_FIELD) {
                dt = pointIn.time;
            } else {
                dt = 0.1 * rotationTravelled / ONE_SCAN_ANGLE;
            }

            double depth = sqrt(pointIn.x * pointIn.x + pointIn.y * pointIn.y +
                                pointIn.z * pointIn.z);

            int ring = 0;
            if (HAS_RING_FIELD) {
                ring = pointIn.ring;
            } else {
                double pitch = asin(pointIn.z / depth) * RAD_2_DEG;
                ring = std::round((pitch - LIDAR_FOV_DOWN) / LIDAR_FOV_RESOLUTION);
                if (ring < 0 || ring >= 16) continue;
            }

            PosTPoint pointOut;
            pointOut.x = pointIn.x;
            pointOut.y = pointIn.y;
            pointOut.z = pointIn.z;
            // TODO: for simulate2
            // dt = 0.0;
            pointOut.timestamp = timebase + dt;

            cloud->at(firing, ring) = pointOut;
        }

        return LiDARFrame::Create(timebase, cloud);
    }

    // -----------
    // OusterLiDAR
    // -----------
    OusterLiDAR::OusterLiDAR(LidarModelType lidarModel) : LiDARDataLoader(lidarModel), NUM_FIRING(2048) {}

    OusterLiDAR::Ptr OusterLiDAR::Create(LidarModelType lidarModel) {
        return std::make_shared<OusterLiDAR>(lidarModel);
    }

    LiDARFrame::Ptr OusterLiDAR::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        sensor_msgs::PointCloud2::ConstPtr lidarMsg = msgInstance.instantiate<sensor_msgs::PointCloud2>();

        OusterPointCloud pcIn;
        pcl::fromROSMsg(*lidarMsg, pcIn);

        int ringNumber;
        switch (_lidarModel) {
            case LidarModelType::OUSTER_16_POINTS:
                ringNumber = 16;
                break;
            case LidarModelType::OUSTER_32_POINTS:
                ringNumber = 32;
                break;
            case LidarModelType::OUSTER_64_POINTS:
                ringNumber = 64;
                break;
            case LidarModelType::OUSTER_128_POINTS:
                ringNumber = 128;
                break;
            default:
                throw Status(
                        Status::Flag::FETAL,
                        "'OusterLiDAR' data loader only supports the lidar type: 'OUSTER_16_POINTS', 'OUSTER_32_POINTS', 'OUSTER_64_POINTS' and 'OUSTER_128_POINTS'");
        }

        int ring_step = pcIn.height / ringNumber;

        assert(ring_step >= 1 && "OusterRingNo too large");

        PosTPointCloud::Ptr cloud(new PosTPointCloud);

        double timebase = lidarMsg->header.stamp.toSec();

        /// point cloud
        cloud->clear();
        cloud->height = ringNumber;
        cloud->width = NUM_FIRING;
        cloud->is_dense = false;
        cloud->resize(cloud->height * cloud->width);

        int numPoints = ringNumber * NUM_FIRING;
        for (int k = 0; k < numPoints; k++) {
            SET_POST_POINT_NAN(cloud->points[k])
        }

        for (int h = 0; h < ringNumber; h++) {
            int hInSelected = h * ring_step;
            for (int w = 0; w < NUM_FIRING; w++) {
                const auto &src = pcIn.at(w, hInSelected);

                double depth = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
                if (depth > 60) continue;

                PosTPoint dstPoint;
                dstPoint.x = src.x;
                dstPoint.y = src.y;
                dstPoint.z = src.z;
                dstPoint.timestamp = timebase + src.t * 1E-9f;

                cloud->at(w, h) = dstPoint;
            }
        }
        return LiDARFrame::Create(timebase, cloud);
    }

    // -----------
    // LivoxLiDAR
    // -----------
    LivoxLiDAR::LivoxLiDAR(LidarModelType lidarModel) : LiDARDataLoader(lidarModel), NUM_POINTS(9840) {}

    LivoxLiDAR::Ptr LivoxLiDAR::Create(LidarModelType lidarModel) {
        return std::make_shared<LivoxLiDAR>(lidarModel);
    }

    LiDARFrame::Ptr LivoxLiDAR::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        switch (_lidarModel) {
            case LidarModelType::LIVOX_HORIZON:
                NUM_POINTS = 9840;
                break;
            case LidarModelType::LIVOX_MID40:
                NUM_POINTS = 9840;
                break;
            case LidarModelType::LIVOX_MID70:
                NUM_POINTS = 9840;
                break;
            case LidarModelType::LIVOX_MID70_SIMU:
                NUM_POINTS = 10000;
                break;
            default:
                throw Status(
                        Status::Flag::FETAL,
                        "'LivoxLiDAR' data loader only supports the lidar type: 'LIVOX_HORIZON', 'LIVOX_MID40', and 'LIVOX_MID70'");
        }

        LivoxPointCloud pcIn;
        double timebase = 0.0;
        //pcl::fromROSMsg(*lidarMsg, pcIn);

        if (_lidarModel == LidarModelType::LIVOX_MID70_SIMU) {
            sensor_msgs::PointCloudConstPtr tmp_lidarMsg = msgInstance.instantiate<sensor_msgs::PointCloud>();
            //sensor_msgs::PointCloud::ConstPtr tmp_lidarMsg = msgInstance.instantiate<sensor_msgs::PointCloud>();
            sensor_msgs::PointCloud2 lidarMsg;
            convertPointCloudToPointCloud2(*tmp_lidarMsg, lidarMsg);
            pcl::fromROSMsg(lidarMsg, pcIn);
            timebase = lidarMsg.header.stamp.toSec();
        } else {
            sensor_msgs::PointCloud2::ConstPtr lidarMsg = msgInstance.instantiate<sensor_msgs::PointCloud2>();
            pcl::fromROSMsg(*lidarMsg, pcIn);
            timebase = lidarMsg->header.stamp.toSec();
        }

        // LivoxPointCloud pcIn;
        // pcl::fromROSMsg(*lidarMsg, pcIn);

        PosTPointCloud::Ptr cloud(new PosTPointCloud);

        //double timebase = lidarMsg->header.stamp.toSec();

        // point cloud
        cloud->clear();
        // no organized pointcloud
        cloud->height = 1;
        cloud->width = pcIn.size();
        cloud->is_dense = true;
        cloud->resize(pcIn.size());

        for (int k = 0; k < pcIn.size(); k++) {
            SET_POST_POINT_NAN(cloud->points[k])
        }
        int valid_size = 0;
        for (int i = 0; i < pcIn.size(); ++i) {
            const auto &pointIn = pcIn.points[i];
            if (IS_POINT_ZERO(pointIn))
                continue;
            if (IS_POINT_NAN(pointIn))
                continue;
            PosTPoint dstPoint;
            dstPoint.x = pointIn.x;
            dstPoint.y = pointIn.y;
            dstPoint.z = pointIn.z;
            valid_size++;
            // for simulate that without motion distortion
            double dt = 0.0;
            if (_lidarModel == LidarModelType::LIVOX_MID70) dt = (double) i / (double) (pcIn.size()) * 0.1;
            dstPoint.timestamp = timebase + dt;

            cloud->points[i] = dstPoint;
        }
        cloud->resize(valid_size);
        // LOG_PROCESS(cloud->size());
        return LiDARFrame::Create(timebase, cloud);
    }


}// namespace ns_elic