//
// Created by csl on 10/1/22.
//
#include "calib/lis_calib_solver.h"
#include "config/calib_config.h"
#include "ros/ros.h"
#include "thirdparty/pretty-table/src/include/prettytable.hpp"
#include "util/status.hpp"

void printLibInfo() {
    std::cout << "+---------------+-------------------------------------------------+--------------------+\n"
                 "| Library       | GitHub-Link                                     | Version            |\n"
                 "+---------------+-------------------------------------------------+--------------------+\n"
                 "| ELIC-Calib    |                                                 | 2.1.0              |\n"
                 "+---------------+-------------------------------------------------+--------------------+\n"
                 "|  _|_|_|_|  _|        _|_|_|    _|_|_|            _|_|_|            _|  _|  _|        |\n"
                 "|  _|        _|          _|    _|                _|          _|_|_|  _|      _|_|_|    |\n"
                 "|  _|_|_|    _|          _|    _|    _|_|_|_|_|  _|        _|    _|  _|  _|  _|    _|  |\n"
                 "|  _|        _|          _|    _|                _|        _|    _|  _|  _|  _|    _|  |\n"
                 "|  _|_|_|_|  _|_|_|_|  _|_|_|    _|_|_|            _|_|_|    _|_|_|  _|  _|  _|_|_|    |\n"
                 "+---------------+-------------------------------------------------+--------------------+"
              << std::endl;
    // print
    LOG_ENDL()
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "elic_calib_prog_node");

    printLibInfo();

    try {
        LOAD_STR_ROS_PARAM("/elic_calib_prog_node/", config_filename)
        LOAD_STR_ROS_PARAM("/elic_calib_prog_node/", shot_screen_save_dir)

        // load the configure file from the yaml-format text file
        ns_elic::CalibConfig::LoadConfigure(config_filename);

        // create a parameter manager, the parameters will be initialized
        auto calibParamManager = ns_elic::CalibParamManager::Create();
        // create a data manager, the data will be load according to the config
        auto calibDataManager = ns_elic::CalibDataManager::Create();

        // create a calibration solver, pass the data and parameter managers to it
        // multi LiDRAs/IMUs sensor suites
        ns_elic::CalibSolver::Ptr calibSolver;

        // SolveMode
        switch (ns_elic::CalibConfig::SolveMode) {
            case ns_elic::SolveModeConfig::Type::MULTI_LiDAR_IMU:
                calibSolver = ns_elic::LIsCalibSolver::Create(
                        calibDataManager, calibParamManager, shot_screen_save_dir
                );
                break;
            default:
                throw ns_elic::Status(ns_elic::Status::Flag::FETAL, "unknown calibration solve mode!");
        }

        // process
        if (1) {
            calibSolver->Process(ns_elic::ReprojectAssociation::CAM_CAM);
        } else {
            calibSolver->Process(ns_elic::ReprojectAssociation::IMG_IMG);
        }

        // save params
        calibParamManager->Save(ns_elic::CalibConfig::CalibData::ParamSavePath);

        // trajectories
        calibSolver->SaveTrajectories();

        // visualize the calibration results
        calibSolver->VisualizationSensors();
        // visualize the scene (surfel)
        calibSolver->VisualizationScene(true);
        // visualize the scene (intensity)
        calibSolver->VisualizationScene(false);

    } catch (const ns_elic::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case ns_elic::Status::Flag::FINE:
                // this case usually won't happen
                LOG_INFO(status.what)
                break;
            case ns_elic::Status::Flag::WARNING:
                LOG_WARNING(status.what)
                break;
            case ns_elic::Status::Flag::ERROR:
                LOG_ERROR(status.what)
                break;
            case ns_elic::Status::Flag::FETAL:
                LOG_FATAL(status.what)
                break;
        }
    } catch (const std::exception &e) {
        // an unknown exception not thrown by this program
        LOG_FATAL(e.what())
    }

    ros::shutdown();
    return 0;
}