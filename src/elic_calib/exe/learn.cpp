//
// Created by csl on 10/1/22.
//
#include "config/calib_config.h"
#include "nofree/learn_se3_spline.hpp"
#include "nofree/trans_plane.hpp"
#include "nofree/plane_observability.hpp"
#include "ros/ros.h"
#include "thirdparty/logger/src/include/logger.h"
#include "util/status.hpp"
#include "nofree/bspline_2d_exam.hpp"
#include "nofree/multi_imu_spline.hpp"

#include "viewer/aligned_map.h"
#include "viewer/surfel_map.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "elic_calib_learn_node");
    try {
        LOAD_STR_ROS_PARAM("/elic_calib_learn_node/", config_filename)
        LOAD_STR_ROS_PARAM("/elic_calib_learn_node/", shot_screen_save_dir)

        // load the configure file from the yaml-format text file
        ns_elic::CalibConfig::LoadConfigure(config_filename);

        // create a parameter manager, the parameters will be initialized
        auto calibParamManager = ns_elic::CalibParamManager::Create();

    } catch (const ns_elic::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case ns_elic::Status::Flag::FINE:
                // this case usually won't happen
                LOG_INFO(status.what);
                break;
            case ns_elic::Status::Flag::WARNING:
                LOG_WARNING(status.what);
                break;
            case ns_elic::Status::Flag::ERROR:
                LOG_ERROR(status.what);
                break;
            case ns_elic::Status::Flag::FETAL:
                LOG_FATAL(status.what);
                break;
        }
    }
    ros::shutdown();
    return 0;
}