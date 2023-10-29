//
// Created by csl on 10/1/22.
//

#ifndef LIC_CALIB_LIDAR_H
#define LIC_CALIB_LIDAR_H

#include "filesystem"
#include "fstream"
#include "ostream"
#include "pcl_conversions/pcl_conversions.h"
#include "util/macros.hpp"
#include "util/type_define.hpp"
#include "util/utils.hpp"

namespace ns_elic {

    struct LiDARFrame {
    public:
        using Ptr = std::shared_ptr<LiDARFrame>;

    private:
        // the timestamp of this lidar scan
        double _timestamp;
        // the lidar scan [x, y, z, timestamp]
        PosTPointCloud::Ptr _scan;

    public:

        // constructor
        explicit LiDARFrame(
                double timestamp = INVALID_TIME_STAMP, PosTPointCloud::Ptr scan = boost::make_shared<PosTPointCloud>()
        );

        // creator
        static LiDARFrame::Ptr Create(
                double timestamp = INVALID_TIME_STAMP,
                const PosTPointCloud::Ptr &scan = boost::make_shared<PosTPointCloud>()
        );

        // access
        [[nodiscard]] double GetMaxTimestamp() const;

        [[nodiscard]] double GetMinTimestamp() const;

        [[nodiscard]] PosTPointCloud::Ptr GetScan() const;

        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        friend std::ostream &operator<<(std::ostream &os, const LiDARFrame &frame);

        // just for debug
        void SaveToFile(const std::string &filename) const;

        // save lidar frames sequence to disk
        static bool SaveFramesToDisk(
                const std::string &directory, const ns_elic::aligned_vector<LiDARFrame::Ptr> &frames, int precision = 10
        );

        static ns_elic::aligned_vector<std::pair<double, std::string>> LoadFramesFromDisk(const std::string &directory);

    };

}

#endif // LIC_CALIB_LIDAR_H
