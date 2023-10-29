//
// Created by csl on 10/1/22.
//

#include <utility>

#include "calib/lis_calib_solver.h"
#include "config/calib_config.h"
#include "core/rotation_estimator.h"
#include "core/scan_undistortion.h"
#include "util/status.hpp"
#include "utility"
#include "viewer/aligned_map.h"
#include "functors/point_plane_functor.hpp"
#include "functors/imu_functor.hpp"

namespace ns_elic {

    LIsCalibSolver::LIsCalibSolver(const CalibDataManager::Ptr &calibDataManager,
                                   const CalibParamManager::Ptr &calibParamManager,
                                   const std::string &sceneShotSaveDir)
            : CalibSolver(calibDataManager, calibParamManager, sceneShotSaveDir), _lidarOdometer(),
              _surfelAssociation(), _refLiDarOdometer(nullptr), _surfelConstructor(nullptr) {

        // create the lidar odometer
        LOG_PLAINTEXT("create the lidar odometer for lidar scans, info:")
        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            _lidarOdometer.insert({topic, LiDAROdometer::Create()});
            LOG_PLAINTEXT("NDT Resolution: ", _lidarOdometer.at(topic)->GetNdt()->getResolution())
        }
        LOG_ENDL()

        // association solver
        LOG_PLAINTEXT("create the association handles.")
        LOG_ENDL()
        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            _surfelAssociation.insert({topic, SurfelAssociation::Create()});
        }
        LOG_PLAINTEXT("create the calibration solver finished.")
        LOG_ENDL()

        LOG_PLAINTEXT("here are the init parameters info:")
        _calibParamManager->ShowParamStatus();
        LOG_ENDL()
    }

    LIsCalibSolver::Ptr
    LIsCalibSolver::Create(const CalibDataManager::Ptr &calibDataManager,
                           const CalibParamManager::Ptr &calibParamManager,
                           const std::string &sceneShotSaveDir) {
        return std::make_shared<LIsCalibSolver>(calibDataManager, calibParamManager, sceneShotSaveDir);
    }

    void LIsCalibSolver::Initialization() {
        LOG_PROCESS("start 'Initialization' of the 'LIsCalibSolver'")
        // ---------------------------
        // init the so3 spline for imu
        // ---------------------------
        LOG_INFO("ready to estimate the reference SO3 B-spline for IMU, adding data to the estimator...")
        auto estimator = TrajectoryEstimator::Create(_trajectory, _calibParamManager);
        // add measurement(only gyro measurement)...
        for (const auto &[topic, imuFrames]: _calibDataManager->GetImuRawFrames()) {
            // here only optimize the so3 knots, don't optimize the bias
            for (const auto &frame: imuFrames) {
                estimator->AddIMUGyroIdealMeasurement(
                        frame, topic, OptimizationOption::OPT_SO3, CalibConfig::Optimization::OptWeight::GyroWeight
                );
            }
        }
        estimator->AddSO3Centralization(
                _calibParamManager->EXTRI.SO3_IjToI_AddressVec(), 1E4, OptimizationOption::OPT_SO3_IjToIr, true
        );
        // fix the origin to be identity
        estimator->FixSO3ControlPointAt(0);
        LOG_PLAINTEXT("add data to the estimator finished, start to solve...")
        // solve
        ceres::Solver::Summary summary = estimator->Solve();
        LOG_PLAINTEXT("solve finished, info:")
        LOG_PLAINTEXT(summary.BriefReport())
        LOG_ENDL()

        // -------
        // display
        // -------
        // _viewer("Init SO3 B-Spline for IMU");
        // _viewer.ShowPoseSequence(
        //         {PoseSeqDisplay(_trajectory->Sampling(0.1), PoseSeqDisplay::Mode::COORD)}
        // );
        // _viewer.RunSingleThread();
        // ------------
        // save to file
        // ------------
        // _trajectory->SamplingToFile(
        //         "/home/lsy/ros_ws/LIC-Calib/src/lic_calib/output/debug/init_traj_i_to_g.txt", 0.05
        // );

        // ----------------------------------------------------------
        // init the lidar odometer, compute relative rotation
        // ----------------------------------------------------------
        LOG_INFO("ready to init the rotation between imu and other sensors...")
        for (const auto &[topic, data]: _calibDataManager->GetLidarRawFrames()) {
            auto rotEstimator = RotationEstimator::Create();
            Posed predCurToLast;
            bool First_Flag = true;
            LiDARFrame::Ptr lastFrame;
            for (const auto &frame: data) {
                if(First_Flag) {
                   // predCurToLast = Posed();
                   First_Flag = false;
                   lastFrame = frame;
                }
                else {
                    auto curFrame = frame;
                    auto curLtoRef = _trajectory->LiDARToRef(
                                curFrame->GetTimestamp(), topic, _calibParamManager
                        );
                    auto lastLtoRef = _trajectory->LiDARToRef(
                                lastFrame->GetTimestamp(), topic, _calibParamManager
                    );

                    // is we query pose successfully
                    if (curLtoRef && lastLtoRef) {
                        // here, the translation has not been initialized, we can't use it
                        predCurToLast.t = Eigen::Vector3d::Zero();
                        predCurToLast.so3 = (*lastLtoRef).so3().inverse() * (*curLtoRef).so3();
                    } else {
                        predCurToLast = Posed();
                    }
                    lastFrame = frame;
                }            
                // run the lidar odometer(feed frame to ndt solver)
                if(frame->GetScan()->is_dense)
                    _lidarOdometer.at(topic)->FeedFrame(frame, predCurToLast.T(), 1);
                else
                {
                    _lidarOdometer.at(topic)->FeedFrame(frame);
                }
                // we run rotation solver when frame size is 25, 30, 35, ...
                if(frame->GetScan()->is_dense)
                {
                if (_lidarOdometer.at(topic)->FrameSize() < 200 || (_lidarOdometer.at(topic)->FrameSize() % 5 != 0))
                    continue;
                }
                else
                {
                if (_lidarOdometer.at(topic)->FrameSize() < 20 || (_lidarOdometer.at(topic)->FrameSize() % 5 != 0))
                    continue;
                }


                // estimate the rotation
                rotEstimator->Estimate(_trajectory, _lidarOdometer.at(topic)->GetOdomPoseVec());

                // check solver status
                if (rotEstimator->SolveStatus()) {
                    // update [rot lidar to imu]
                    // attention: here we store the 'SO3_LmToIr' to 'SO3_LmToLr' for convenient, don't misunderstand it!!!
                    _calibParamManager->EXTRI.SO3_LmToLr.at(topic) = rotEstimator->GetSO3TargetToTraj();
                    // once we solve the rotation successfully, just break. so here, the lidar odometer may be incomplete
                    // if(topic == "/livox_raw_frames")
                    // {
                    //    _viewer("Livox NDT ODO");
                    //    _viewer.ShowLiDARFrameWithIntensity(_lidarOdometer.at("/livox_raw_frames")->GetMap(),"livox");
                    //     _viewer.RunSingleThread();
                    // }
                    
                    
                    break;
                }
            }
            if (!rotEstimator->SolveStatus()) {
                throw Status(
                        Status::Flag::ERROR,
                        "init rotation 'SO3_LtoIr' failed, this may be related to the 'NDTResolution' of lidar odometer"
                );
            }
        }
        LOG_INFO("ready to estimate the reference LiDAR for Lidars, adding data to the estimator...")
        estimator = TrajectoryEstimator::Create(_trajectory, _calibParamManager);
        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            estimator->AddMultiLiDARRotCentralization(
                    _calibParamManager->EXTRI.SO3_LmToLr.at(topic), topic,
                    CalibConfig::Optimization::OptWeight::SO3Weight,
                    OptimizationOption::OPT_SO3_LmToLr | OptimizationOption::OPT_SO3_LrToIr
            );
        }
        estimator->AddSO3Centralization(
                _calibParamManager->EXTRI.SO3_LmToLr_AddressVec(), 1E4, OptimizationOption::OPT_SO3_LmToLr, false
        );
        LOG_PLAINTEXT("add data to the estimator finished, start to solve...")
        // solve
        summary = estimator->Solve();
        LOG_PLAINTEXT("solve finished, info:")
        LOG_PLAINTEXT(summary.BriefReport())
        LOG_ENDL()

        // -------
        // display
        // -------
        // _viewer("Init LiDAR Odometer");
        // _viewer.ShowLiDARFramesInMapWithIntensity(
        //         _lidarOdometer->GetFramesVec(),
        //         PoseSeqDisplay(_lidarOdometer->GetPoseVec(), PoseSeqDisplay::Mode::COORD)
        // );
        // _viewer.RunSingleThread();
        // ------------
        // save to file
        // ------------
        // _lidarOdometer->SaveToFile("/home/csl/ros_ws/LIC-Calib/src/lic_calib/output/debug/init_lidar_odometer.txt");

        LOG_PLAINTEXT("init rotation finished, info:")
        _calibParamManager->ShowParamStatus();
        LOG_ENDL()

        LOG_PLAINTEXT("'Initialization' of the 'LIsCalibSolver' finished")
        LOG_ENDL()
    }

    void LIsCalibSolver::DataAssociation(std::uint32_t scanUndistortionOption, std::uint32_t reprojectAssocOption) {
        LOG_PROCESS("start 'DataAssociation' of the 'LIsCalibSolver' with 'scanUndistortionOption' = ",
                    ScanUndistortion::Option(scanUndistortionOption), ".")
        // ---------------------
        // Un distort lidar Scan
        // ---------------------
        {
            LOG_INFO("ready to undistort the lidar scans using imu B-Spline...")
            auto undistHelper = ScanUndistortion::Create(_trajectory, _calibParamManager);
            // using option: "UNDIST_SO3 | UNDIST_POS" is not good, just undistort the rotation only by using UNDIST_SO3 at the fist time
            // the result is kept in the _calibDataManager, because it is the data manager
            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                undistHelper->UndistortToScan(_calibDataManager, topic, scanUndistortionOption);
                LOG_PLAINTEXT("undistort lidar scans for lidar '", topic, "' finished.")
            }
            LOG_ENDL()
            // -------
            // display
            // -------
            // _viewer("LiDAR Scan Undistort Compare");
            // // the raw scan is red, the undistort scan is green
            // _viewer.ShowLiDARFramesInMapWithColorWheel(
            //         {_calibDataManager->GetLidarRawFrames().at(0),
            //          _calibDataManager->GetLidarUndistFrames().at(0)}
            // );
            // _viewer.RunSingleThread();
            // ------------
            // save to file
            // ------------
            // _calibDataManager->GetLidarRawFrames().front()->SaveToFile(
            //         "/home/csl/ros_ws/LIC-Calib/src/lic_calib/output/debug/fir_raw_lidar_frame.txt"
            // );
            // _calibDataManager->GetLidarRawFrames().back()->SaveToFile(
            //         "/home/csl/ros_ws/LIC-Calib/src/lic_calib/output/debug/las_raw_lidar_frame.txt"
            // );
            // _calibDataManager->GetLidarUndistFrames().front()->SaveToFile(
            //         "/home/csl/ros_ws/LIC-Calib/src/lic_calib/output/debug/fir_undistort_lidar_frame.txt"
            // );
            // _calibDataManager->GetLidarUndistFrames().back()->SaveToFile(
            //         "/home/csl/ros_ws/LIC-Calib/src/lic_calib/output/debug/las_undistort_lidar_frame.txt"
            // );
        }

        // ------------------------------------------------------------
        // resolve the lidar odometer using the Undistorted lidar scans
        // ------------------------------------------------------------
        {
            LOG_INFO("ready to resolve the lidar odometer using undistorted lidar scans...")
            // here create a new clean lidar odometer
            for (const auto &[topic, undistFrames]: _calibDataManager->GetLidarUndistFrames()) {
                _lidarOdometer[topic] = LiDAROdometer::Create();
                for (int i = 0; i < undistFrames.size(); ++i) {
                    auto curUndistFrame = undistFrames.at(i);
                    Posed predCurToLast;
                    if (i == 0) {
                        predCurToLast = Posed();
                    } else {
                        auto lastUndistortedFrame = undistFrames.at(i - 1);
                        auto curLtoRef = _trajectory->LiDARToRef(
                                curUndistFrame->GetTimestamp(), topic, _calibParamManager
                        );
                        auto lastLtoRef = _trajectory->LiDARToRef(
                                lastUndistortedFrame->GetTimestamp(), topic, _calibParamManager
                        );
                        // is we query pose successfully
                        if (curLtoRef && lastLtoRef) {
                            // here, the translation has not been initialized, we can't use it
                            predCurToLast.t = Eigen::Vector3d::Zero();
                            predCurToLast.so3 = (*lastLtoRef).so3().inverse() * (*curLtoRef).so3();
                        } else {
                            predCurToLast = Posed();
                        }
                    }
                    // use the last lidar odometer result as the predict pose
                    _lidarOdometer[topic]->FeedFrame(
                            curUndistFrame, predCurToLast.T(), i < CalibConfig::LiDAROdometer::UpdateMapUntil
                    );
                }
            }
            LOG_PLAINTEXT("resolve lidar odometer finished.")
            LOG_ENDL()
            // ------------
            // save             
            // _viewer("LiDAR Odometer Using Scans Undistorted");
            // _viewer.ShowLiDARFramesInMapWithIntensity(
            //         _lidarOdometer["/livox_raw_frames"]->GetFramesVec(),
            //         PoseSeqDisplay(_lidarOdometer["/livox_raw_frames"]->GetPoseVec(), PoseSeqDisplay::Mode::COORD)
            // );
            // _viewer.RunSingleThread();
            // ------------
            // _lidarOdometer->SaveToFile("/home/csl/ros_ws/LIC-Calib/src/lic_calib/output/debug/lidar_odometer.txt");
            // -------
            // display
            // -------
            // _viewer("LiDAR Odometer Using Scans Undistorted");
            // _viewer.ShowLiDARFramesInMapWithIntensity(
            //         _lidarOdometer["/livox_raw_frames"]->GetFramesVec(),
            //         PoseSeqDisplay(_lidarOdometer["/livox_raw_frames"]->GetPoseVec(), PoseSeqDisplay::Mode::COORD)
            // );
            // _viewer.RunSingleThread();

            // _viewer("LiDAR Odometer Using Scans Undistorted");
            // _viewer.ShowLiDARFramesInMapWithIntensity(
            //         _lidarOdometer["/livox_raw_frames"]->GetFramesVec(),
            //         PoseSeqDisplay(_lidarOdometer["/livox_raw_frames"]->GetPoseVec(), PoseSeqDisplay::Mode::COORD)
            // );
            // _viewer.RunSingleThread();
        }

        // Need register maps to re-initialize the Livox-IMU rotational extrinisc!
        // -----------------------------------------------------------
        // init multi-lidar: POS_LmToLr
        // -----------------------------------------------------------
        {
            auto &L0Topic = *CalibConfig::CalibData::Topic::LiDARTopics.begin();
            auto tempLidarOdometer = LiDAROdometer::Create();
            const Sophus::SO3d SO3_LrToL0 = _calibParamManager->EXTRI.SO3_LmToLr.at(L0Topic).inverse();
            // [ topic, pose]
            aligned_map<std::string, OdomPosed> LmToL0;

            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                const auto &curLidarMap = _lidarOdometer.at(topic)->GetMap();
                const Sophus::SO3d SO3_LmToL0 = SO3_LrToL0 * _calibParamManager->EXTRI.SO3_LmToLr.at(topic);;
                auto pose = tempLidarOdometer->FeedFrame(
                        curLidarMap, Sophus::SE3d(SO3_LmToL0, Eigen::Vector3d::Zero()).matrix(), true
                );
                LmToL0.insert({topic, pose});
            }
            // center
            Eigen::Vector3d POS_LrInL0 = Eigen::Vector3d::Zero();
            for (const auto &[topic, pose]: LmToL0) { POS_LrInL0 += pose.Translation(); }
            POS_LrInL0 /= static_cast<double>(LmToL0.size());
            for (const auto &[topic, pose]: LmToL0) {
                _calibParamManager->EXTRI.POS_LmInLr.at(topic) =
                        SO3_LrToL0.inverse() * (pose.Translation() - POS_LrInL0);
            }
            for (const auto &[topic, pose]: LmToL0) {
                if(topic == L0Topic || (topic.find("livox") == topic.npos)) continue;
                // Only livox LiDAR need to re-initialize rotation
                Sophus::SO3d refine_Rot(Eigen::Quaterniond(pose.Rotation()));
                _calibParamManager->EXTRI.SO3_LmToLr.at(topic) = _calibParamManager->EXTRI.SO3_LmToLr.at(L0Topic) * refine_Rot;
                LOG_VAR(refine_Rot.matrix())
            }
        }
        _calibParamManager->ShowParamStatus();
        LOG_ENDL()
        // ---------------------------------
        // init the reference lidar odometer
        // ---------------------------------
        {
            aligned_vector<LiDAROdometerPack> lidarPacks;
            for (const auto &[topic, odom]: _lidarOdometer) {
                if((topic.find("livox") != topic.npos)) continue;
                lidarPacks.emplace_back(
                        odom->GetPoseVec(), odom->GetMap(), _calibParamManager->EXTRI.SE3_LmToLr(topic)
                );
            }
            _refLiDarOdometer = RefLiDAROdometer::Create(
                    lidarPacks, _calibParamManager, _trajectory->minTime(), _trajectory->maxTime()
            );
        }
        // if(topic == "/sim_livox_0/frame")
        // if(1)
        // {
        //     _viewer("Livox NDT ODO");
        //     _viewer.ShowLiDARFrameWithIntensity(_lidarOdometer.at("/sim_livox_0/frame")->GetMap(),"livox");
        //     _viewer.RunSingleThread();
        // }

        // -------------------------------------------
        // init the imu pose b-spline and struct scale
        // -------------------------------------------
        // here we create a temp b-spline for init the odometer trajectory, the knots dist 0.2
        // we first fit the lidar odometer trajectory, than using this trajectory to init the imu b-spline trajectory [mainly for pos]
        {
            LOG_INFO("ready to optimize the pose b-spline for imu using odometer pose b-spline...")
            // sample odometer trajectory to add constant to imu b-spline
            double sampleTime = _trajectory->getDt() * 0.43;
            auto estimator = TrajectoryEstimator::Create(_trajectory, _calibParamManager);
            auto trajLrToMr = _refLiDarOdometer->GetTrajLrToMr();
            for (double t = _trajectory->minTime() + sampleTime; t < _trajectory->maxTime();) {
                auto IrToLr = trajLrToMr->pose(t) * _calibParamManager->EXTRI.SE3_LrToIr().inverse();
                estimator->AddSE3Measurement(
                        Posed::FromSE3(IrToLr, t),
                        OptimizationOption::OPT_SO3 | OptimizationOption::OPT_POS,
                        CalibConfig::Optimization::OptWeight::SO3Weight,
                        CalibConfig::Optimization::OptWeight::POSWeight
                );
                t += sampleTime;
            }

            // solve
            ceres::Solver::Summary trajSummary = estimator->Solve();
            LOG_PLAINTEXT("solve finished, info:")
            LOG_PLAINTEXT(trajSummary.BriefReport())
            _calibParamManager->ShowParamStatus();
            LOG_ENDL()
            // ------------
            // save to file
            // ------------
            // _trajectory->SamplingToFile(
            //         "/home/csl/ros_ws/LIC-Calib/src/lic_calib/output/debug/traj_i_to_m.txt", 0.05
            // );
            // -------
            // display
            // -------
            // _viewer("Trajectory IMU to Map");
            // _viewer.ShowPoseSequence(
            //         {PoseSeqDisplay(_trajectory->Sampling(0.05), PoseSeqDisplay::Mode::COORD)}
            // );
            // _viewer.RunSingleThread();
        }

        // ---------------------------
        // create the data association
        // ---------------------------
        {
            LOG_INFO("ready to construct the association for lidar scans...")
            // ndt
            auto ndt = pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr(
                    new pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>()
            );
            ndt->setResolution(static_cast<float>(CalibConfig::LiDAROdometer::NDTResolution));
            ndt->setNumThreads(CalibConfig::LiDAROdometer::ThreadNum);
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            ndt->setTransformationEpsilon(1E-3);
            ndt->setStepSize(0.01);
            ndt->setMaximumIterations(50);
            ndt->setInputTarget(_refLiDarOdometer->GetMap());

            _surfelConstructor = SurfelConstructor::Create(ndt);

            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                // this function trans the local lidar frame to the map, according to the lidar odometer's pose seq
                // the result is saved in the _calibDataManager
                _calibDataManager->ComputeLidarUndistFramesInRef(
                        _lidarOdometer.at(topic)->GetOdomPoseVec(), _calibParamManager, topic
                );
                // -------
                // display
                // -------
                // _viewer("Lidar Undistorted Scans In Reference");
                // _viewer.ShowLiDARFramesInMapWithIntensity(
                //         _calibDataManager->GetLidarUndistFramesInRef().at(topic)
                // );
                // _viewer.RunSingleThread();

                // because we use all lidar frames to build the odometer, so the surfelRefTime is the map time
                _surfelAssociation.at(topic)->SetUp(
                        _surfelConstructor, _calibDataManager->GetLidarRawFrames().at(topic),
                        _calibDataManager->GetLidarUndistFramesInRef().at(topic), (topic.find("sim") != topic.npos)
                );
            }

            LOG_PLAINTEXT("construct the association finished.")
            LOG_ENDL()
            // -------
            // display
            // -------
            // _viewer("Surfel Planes With Surfel Points");
            // _viewer.ShowSurfelPlanePoints(
            //         _surfelAssociation->GetSurfelPlanes(), _surfelAssociation->GetSpointPerSurfel()
            // );
            // _viewer.RunSingleThread();
        }
    }

    void
    LIsCalibSolver::DataAssociationInRefine(std::uint32_t scanUndistortionOption, std::uint32_t reprojectAssocOption) {
        LOG_INFO("start 'DataAssociationInRefine' of the 'LIsCalibSolver' with 'scanUndistortionOption' = ",
                 ScanUndistortion::Option(scanUndistortionOption), ".")
        // ---------------------
        // Un distort lidar Scan
        // ---------------------
        {
            LOG_PLAINTEXT("ready to undistort the lidar scans using imu B-Spline...")
            auto undistHelper = ScanUndistortion::Create(_trajectory, _calibParamManager);
            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                // using option: "UNDIST_SO3 | UNDIST_POS" is not good, just undistort the rotation
                // the result is keep in the _calibDataManager, because it is the data manager
                undistHelper->UndistortToRef(_calibDataManager, topic, scanUndistortionOption);
            }
            LOG_PLAINTEXT("undistort lidar scans to reference finished.")
            LOG_ENDL()
            // -------
            // display
            // -------
            // _viewer("LiDAR Scan Undistort Compare");
            // the raw scan is red, the undistort scan is green
            // _viewer.ShowLiDARFramesInMapWithColorWheel(
            //         {_calibDataManager->GetLidarRawFrames().at(0),
            //          _calibDataManager->GetLidarUndistFrames().at(0)}
            // );
            // _viewer.RunSingleThread();
        }

        {
            LOG_PLAINTEXT("construct the lidar ndt mapCloud...")
            const auto &undistortFramesInRef = _calibDataManager->GetLidarUndistFramesInRef();
            PosTPointCloud::Ptr mapCloud(new PosTPointCloud);
            for (const auto &[topic, frames]: undistortFramesInRef) {
                for (const auto &frame: frames) {
                    PosTPointCloud::Ptr scanDownSampled(new PosTPointCloud);
                    // down sample
                    for (int k = 0; k < frame->GetScan()->size(); k += 3) {
                        const auto &p = frame->GetScan()->points[k];
                        if (IS_POINT_NAN(p)) {
                            continue;
                        }
                        scanDownSampled->push_back(p);
                    }
                    // add
                    *mapCloud += *scanDownSampled;
                }
            }
            // -------
            // display
            // -------
            // _viewer("Lidar Undistorted Scans In Reference");
            // _viewer.ShowLiDARFrameWithColorWheel(
            //         LiDARFrame::Create(_calibDataManager->GetMapTimestamp(), mapCloud)
            // );
            // _viewer.RunSingleThread();

            // construct the ndt object
            auto ndt = pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>::Ptr(
                    new pclomp::NormalDistributionsTransform<PosTPoint, PosTPoint>()
            );
            ndt->setResolution(static_cast<float>(CalibConfig::LiDAROdometer::NDTResolution));
            ndt->setNumThreads(CalibConfig::LiDAROdometer::ThreadNum);
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            ndt->setTransformationEpsilon(1E-3);
            ndt->setStepSize(0.01);
            ndt->setMaximumIterations(50);
            ndt->setInputTarget(mapCloud);

            _surfelConstructor = SurfelConstructor::Create(ndt);

            // set up the surfel objects
            for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                _surfelAssociation[topic] = SurfelAssociation::Create();
                _surfelAssociation.at(topic)->SetUp(
                        _surfelConstructor, _calibDataManager->GetLidarRawFrames().at(topic),
                        _calibDataManager->GetLidarUndistFramesInRef().at(topic), (topic.find("sim") != topic.npos)
                );
            }

            LOG_ENDL()
            // -------
            // display
            // -------
            // _viewer("Surfel Planes With Surfel Points");
            // _viewer.ShowSurfelPlanePoints(
            //         _surfelAssociation->GetSurfelPlanes(), _surfelAssociation->GetSpointPerSurfel()
            // );
            // _viewer.RunSingleThread();
        }
    }

    // ------
    // global
    // ------

    void LIsCalibSolver::Process(std::uint32_t reprojectAssocOption) {
        CalibConfig::CheckConfigureStatus();

        this->Initialization();

        this->DataAssociation(ScanUndistortion::Option::UNDIST_SO3, reprojectAssocOption);

        // pointPlaneOptOption
        std::uint32_t pointPlaneOptOption = OptimizationOption::OPT_SO3 | OptimizationOption::OPT_POS |
                                            OptimizationOption::OPT_SO3_LmToLr | OptimizationOption::OPT_POS_LmInLr |
                                            OptimizationOption::OPT_SO3_LrToIr | OptimizationOption::OPT_POS_LrInIr;
        // imuMeasureOptOption
        std::uint32_t imuMeasureOptOption = OptimizationOption::OPT_SO3 | OptimizationOption::OPT_POS |
                                            OptimizationOption::OPT_GRAVITY_REFINE | OptimizationOption::OPT_GYRO_BIAS |
                                            OptimizationOption::OPT_ACCE_BIAS | OptimizationOption::OPT_SO3_IjToIr |
                                            OptimizationOption::OPT_POS_IjInIr;

        if (!CalibConfig::Optimization::LockTimeOffset) {
            pointPlaneOptOption |= OptimizationOption::OPT_TIME_OFFSET_LmToLr;
            pointPlaneOptOption |= OptimizationOption::OPT_TIME_OFFSET_LrToIr;

            // this is an issue to consider, whether optimize these parameters in first batch optimization
            // imuMeasureOptOption |= OptimizationOption::OPT_TIME_OFFSET_IjToIr;
        }

        std::uint32_t reprojectOptOption = OptimizationOption::NONE;


        reprojectOptOption |= OptimizationOption::NONE;
        

        this->BatchOptimization(
                pointPlaneOptOption, reprojectOptOption, imuMeasureOptOption, reprojectAssocOption
        );

        for (int i = 0; i < CalibConfig::Optimization::RefineIterations; ++i) {
            this->Refinement(i, reprojectAssocOption);
        }

        LOG_PROCESS("process finished, everything is fine!")
    }

    void LIsCalibSolver::BatchOptimization(std::uint32_t pointPlaneOptOption, std::uint32_t reprojectOptOption,
                                           std::uint32_t imuMeasureOptOption, std::uint32_t reprojectAssocOption) {
        LOG_PROCESS("start 'BatchOptimization' of the 'LIsCalibSolver' with:")
        LOG_PROCESS("'pointPlaneOptOption' = ", OptimizationOption::Option(pointPlaneOptOption), ".")
        LOG_PROCESS("'imuMeasureOptOption' = ", OptimizationOption::Option(imuMeasureOptOption), ".")

        auto estimator = TrajectoryEstimator::Create(_trajectory, _calibParamManager);

        LOG_PLAINTEXT("finding the correspondences...")

        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            auto surfelCorr = _surfelAssociation.at(topic)->ConstructCorrespondence();
            LOG_PLAINTEXT("total surfelCorr count: ", surfelCorr.size(), " for lidar: ", topic)
            LOG_PLAINTEXT("adding the surfelCorr to estimator...")
            // add surfelCorr
            for (const auto &corr: surfelCorr) {

                estimator->AddPointPlaneCorrespondence(
                        corr, topic, pointPlaneOptOption, CalibConfig::Optimization::OptWeight::LidarWeight
                );
                
            }
        }

        LOG_PLAINTEXT("adding the imu measurement to estimator...")
        // add imu measurement
        for (const auto &[topic, imuFrames]: _calibDataManager->GetImuRawFrames()) {
            for (const auto &frame: imuFrames) {
                // we optimize all params that imu measurement can optimize
                estimator->AddIMUMeasurement(
                        frame, topic, imuMeasureOptOption,
                        CalibConfig::Optimization::OptWeight::GyroWeight,
                        CalibConfig::Optimization::OptWeight::AcceWeight
                );
            }
        }
        estimator->AddSO3Centralization(
                _calibParamManager->EXTRI.SO3_IjToI_AddressVec(), 1E4, imuMeasureOptOption
        );
        estimator->AddPOSCentralization(
                _calibParamManager->EXTRI.POS_IjInI_AddressVec(), 1E4, imuMeasureOptOption
        );
        estimator->AddTimeOffsetCentralization(
                _calibParamManager->TEMPORAL.TIME_OFFSET_IjToI_AddressVec(), 1E4, imuMeasureOptOption
        );
        estimator->AddSO3Centralization(
                _calibParamManager->EXTRI.SO3_LmToLr_AddressVec(), 1E4, pointPlaneOptOption, false
        );
        estimator->AddPOSCentralization(
                _calibParamManager->EXTRI.POS_LmInLr_AddressVec(), 1E4, pointPlaneOptOption, false
        );
        estimator->AddTimeOffsetCentralization(
                _calibParamManager->TEMPORAL.TIME_OFFSET_LmToLr_AddressVec(), 1E4, pointPlaneOptOption, false
        );

        if (CalibConfig::CalibData::OutputData::OutputLMEquationGraph) {
            // save the lm equation
            SaveEquationGraph(estimator);
        }

        LOG_PLAINTEXT("start solving...")
        // solve
        auto option = TrajectoryEstimator::DefaultSolverOptions(CalibConfig::Optimization::UseCuda);

        if (CalibConfig::CalibData::OutputData::OutputParamInEachIter) {
            option.callbacks.push_back(new LIsCeresDebugCallBack(_calibParamManager));
            option.update_state_every_iteration = true;
        }

        ceres::Solver::Summary summary = estimator->Solve(option);
        LOG_PLAINTEXT("solve finished, info:")
        LOG_PLAINTEXT(summary.BriefReport())
        _calibParamManager->ShowParamStatus();
        LOG_ENDL()

        // ------------
        // save to file
        // ------------
        // _trajectory->SamplingToFile(
        //         "/home/csl/ros_ws/LIC-Calib/src/lic_calib/output/debug/opt_traj_i_to_m.txt", 0.05
        // );
        // -------
        // display : green(imu trajectory), red(lidar odometer trajectory)
        // -------
        // _viewer("Optimized Trajectory IMU to Map");
        // _viewer.ShowPoseSequence(
        //         {PoseSeqDisplay(_trajectory->Sampling(0.05), PoseSeqDisplay::Mode::COORD),
        //          PoseSeqDisplay(_lidarOdometer->GetPoseVec(), PoseSeqDisplay::Mode::COORD)}
        // );
    }

    void LIsCalibSolver::Refinement(int iterationIndex, std::uint32_t reprojectAssocOption) {
        LOG_PROCESS("start 'Refinement' of the 'LIsCalibSolver' with 'iterationIndex' = ", iterationIndex, ".")
        // attention: these values are static
        static std::uint32_t scanUndistortionOption, pointPlaneOptOption, reprojectOptOption, imuMeasureOptOption;

        if (iterationIndex == 0) {
            scanUndistortionOption = ScanUndistortion::Option::UNDIST_SO3 | ScanUndistortion::Option::UNDIST_POS;

            // pointPlaneOptOption
            pointPlaneOptOption = OptimizationOption::OPT_SO3 | OptimizationOption::OPT_POS |
                                  OptimizationOption::OPT_SO3_LmToLr | OptimizationOption::OPT_POS_LmInLr |
                                  OptimizationOption::OPT_SO3_LrToIr | OptimizationOption::OPT_POS_LrInIr;

            reprojectOptOption = OptimizationOption::NONE;

            // imuMeasureOptOption
            imuMeasureOptOption = OptimizationOption::OPT_SO3 | OptimizationOption::OPT_POS |
                                  OptimizationOption::OPT_GRAVITY_REFINE | OptimizationOption::OPT_GYRO_BIAS |
                                  OptimizationOption::OPT_ACCE_BIAS | OptimizationOption::OPT_SO3_IjToIr |
                                  OptimizationOption::OPT_POS_IjInIr;

        } else {

            if (!CalibConfig::Optimization::LockTimeOffset) {
                pointPlaneOptOption |= OptimizationOption::OPT_TIME_OFFSET_LmToLr;
                pointPlaneOptOption |= OptimizationOption::OPT_TIME_OFFSET_LrToIr;
                reprojectOptOption |= OptimizationOption::NONE;
                imuMeasureOptOption |= OptimizationOption::OPT_TIME_OFFSET_IjToIr;
            }


            reprojectOptOption |= OptimizationOption::NONE;
            

            if (!CalibConfig::Optimization::LockIMUIntrinsic) {
                imuMeasureOptOption |= OptimizationOption::OPT_ACCE_MAP_COEFF | OptimizationOption::OPT_GYRO_MAP_COEFF |
                                       OptimizationOption::OPT_SO3_AtoG;
            }

            reprojectOptOption |= OptimizationOption::NONE;
            
        }

        this->DataAssociationInRefine(scanUndistortionOption, reprojectAssocOption);

        this->BatchOptimization(
                pointPlaneOptOption, reprojectOptOption, imuMeasureOptOption, reprojectAssocOption
        );
    }

    // ------
    // visual
    // ------

    void LIsCalibSolver::VisualizationScene(bool surfelMode) {
        VisualizationSceneHelper(_surfelConstructor, surfelMode);
    }
}
