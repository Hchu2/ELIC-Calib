//
// Created by csl on 10/4/22.
//

#ifndef LIC_CALIB_SCAN_UNDISTORTION_H
#define LIC_CALIB_SCAN_UNDISTORTION_H

#include "calib/calib_data_manager.h"
#include "sensor/lidar.h"
#include "core/pose.hpp"
#include "core/se3_spline.h"

namespace ns_elic {


    class ScanUndistortion {
    public:
        using Ptr = std::shared_ptr<ScanUndistortion>;

    public:
        enum Option : std::uint32_t {
            /**
             * @brief options
             */
            NONE = 1 << 0,
            UNDIST_SO3 = 1 << 1,
            UNDIST_POS = 1 << 2,
            ALL = UNDIST_SO3 | UNDIST_POS
        };

        static bool IsOptionWith(std::uint32_t desired, std::uint32_t curOption) {
            return (desired == (desired & curOption));
        }

        /**
         * @brief override operator '<<' for type 'Option'
         */
        friend std::ostream &operator<<(std::ostream &os, const Option &curOption) {
            std::stringstream stream;
            int count = 0;
            if (IsOptionWith(UNDIST_SO3, curOption)) {
                stream << "UNDIST_SO3";
                ++count;
            }
            if (IsOptionWith(UNDIST_POS, curOption)) {
                stream << " | UNDIST_POS";
                ++count;
            }
            if (count == 0) {
                os << "NONE";
            } else if (count == 2) {
                os << "ALL";
            } else {
                std::string str = stream.str();
                if (str.at(1) == '|') {
                    str = str.substr(3, str.size() - 3);
                }
                os << str;
            }
            return os;
        };


    private:
        Trajectory::Ptr _trajectory;

        CalibParamManager::Ptr _calibParamManager;

    public:
        ScanUndistortion(Trajectory::Ptr trajectory, CalibParamManager::Ptr calibParamManager);

        static ScanUndistortion::Ptr
        Create(const Trajectory::Ptr &trajectory, const CalibParamManager::Ptr &calibParamManager);

        void
        UndistortToScan(const CalibDataManager::Ptr &calibDataManager, const std::string &topic, std::uint32_t option);

        void
        UndistortToRef(const CalibDataManager::Ptr &calibDataManager, const std::string &topic, std::uint32_t option);

    protected:

        std::optional<LiDARFrame::Ptr>
        UndistortToScan(const LiDARFrame::Ptr &lidarFrame, const std::string &topic, bool correctPos);

        std::optional<LiDARFrame::Ptr>
        UndistortToRef(const LiDARFrame::Ptr &lidarFrame, const std::string &topic, bool correctPos);


    };
}

#endif //LIC_CALIB_SCAN_UNDISTORTION_H
