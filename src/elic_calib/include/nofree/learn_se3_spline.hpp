//
// Created by csl on 10/8/22.
//

#ifndef LIC_CALIB_LEARN_SE3_SPLINE_HPP
#define LIC_CALIB_LEARN_SE3_SPLINE_HPP

#include <utility>

#include "functors/se3_functor.hpp"
#include "core/pose.hpp"
#include "core/se3_spline.h"
#include "core/trajectory_estimator.h"
#include "viewer/pose_viewer.h"

namespace ns_elic {

    void LearnSE3Spline(const CalibParamManager::Ptr &calibParamManager) {
        // create the sim data
        const double sTime = 0.0, eTime = 10.0, deltaTime = 0.5;
        aligned_vector<Posed> poseSeq;
        for (double t = sTime; t < eTime;) {
            Eigen::Vector3d trans;
            trans(0) = std::sin(t) * 10.0;
            trans(1) = std::cos(t) * 10.0;
            trans(2) = 2.0 * t;

            Eigen::Vector3d zAxis = -trans.normalized();
            Eigen::Vector3d xAxis = Eigen::Vector3d(trans(1), -trans(0), 0.0).normalized();
            Eigen::Vector3d yAxis = zAxis.cross(xAxis);
            Eigen::Matrix3d rotMatrix;
            rotMatrix.col(0) = xAxis;
            rotMatrix.col(1) = yAxis;
            rotMatrix.col(2) = zAxis;

            poseSeq.emplace_back(Sophus::SO3d(rotMatrix), trans, t);
            t += deltaTime;
        }
        Viewer viewer;
        viewer.ShowPoseSequence({PoseSeqDisplay(poseSeq, PoseSeqDisplay::Mode::ARROW)});

        // estimate spline
        auto trajectory = Trajectory::Create(deltaTime * 2.3, sTime, eTime);
        TrajectoryEstimator estimator(trajectory, calibParamManager);

        for (const auto &item: poseSeq) {
            estimator.AddSE3Measurement(item, OptimizationOption::OPT_POS | OptimizationOption::OPT_SO3, 1.0, 1.0);
        }
        // solve
        ceres::Solver::Summary summary = estimator.Solve();
        LOG_PLAINTEXT("solve finished, info:")
        LOG_PLAINTEXT(summary.BriefReport());
        // display: the sampling pose sequence, knots sequence, origin data pose sequence
        viewer.ShowPoseSequence(
                {
                        PoseSeqDisplay(
                                trajectory->Sampling(0.1, poseSeq.front().timeStamp, poseSeq.back().timeStamp),
                                PoseSeqDisplay::Mode::COORD
                        ),
                        PoseSeqDisplay(
                                trajectory->Sampling(), PoseSeqDisplay::Mode::ARROW, ns_viewer::Colour::Black()
                        ),
                        PoseSeqDisplay(poseSeq, PoseSeqDisplay::Mode::ARROW)
                }, 1.0f
        );
        viewer.RunSingleThread();
    }

    void LearnSplineMeta() {
        auto traj = Trajectory::Create(0.5, 0.0, 20.0);
        using SplineMeta = basalt::SplineMeta<CalibConfig::BSpline::SplineOrder>;
        SplineMeta splineMeta;
        traj->CalculateSplineMeta({{6.0, 7.0},
                                   {2.0, 3.0}}, splineMeta);
        LOG_VAR(splineMeta.NumParameters())
        for (const auto &item: splineMeta.segments) {
            /**
             *   double t0; // First valid time
             *   double dt; // Knot spacing
             *   size_t n;  // Number of knots
             */
            LOG_VAR(item.NumParameters())
            LOG_VAR(item.t0)
            LOG_VAR(item.dt)
            LOG_VAR(item.n)
            LOG_VAR(item.MinTime())
            LOG_VAR(item.MaxTime())
            LOG_VAR(item.DEG)
            LOG_VAR(item.N)
            LOG_ENDL()
        }
        for (const auto &seg: splineMeta.segments) {
            // the factor '1E-9' is the treatment of numerical accuracy
            auto idxMaster = traj->computeTIndex(seg.t0 + 1E-9).second;
            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                LOG_PLAINTEXT(i)
            }
            LOG_ENDL()
        }
    }

    void LearnSplineControl() {
        // create the b-spline from 0.0 to 10.0, the dist is 0.5
        // ctrl knots  time: 0.0 0.5 1.0 1.5 2.0 2.5 3.0 3.5 4.0 4.5 5.0 5.5 6.0 6.5 7.0 7.5 8.0 8.5 9.0 9.5 10.0
        // ctrl knots index:  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20
        auto traj = Trajectory::Create(0.5, 0.0, 10.0);
        // here the order is 4, degree is 3
        using SplineMeta = basalt::SplineMeta<CalibConfig::BSpline::SplineOrder>;
        {
            SplineMeta splineMeta;
            traj->CalculateSplineMeta({{0.2, 0.2}}, splineMeta);
            // because we only query one time stamp, so the 'splineMeta.segments' sequence only has one segment
            auto idxMaster = traj->computeTIndex(splineMeta.segments.front().t0 + 1E-9).second;
            for (int i = idxMaster; i < splineMeta.segments.front().NumParameters() + idxMaster; ++i) {
                LOG_VAR(i)
            }
            // output: 0 1 2 3
        }
        {
            SplineMeta splineMeta;
            traj->CalculateSplineMeta({{1.2, 1.2}}, splineMeta);
            // because we only query one time stamp, so the 'splineMeta.segments' sequence only has one segment
            auto idxMaster = traj->computeTIndex(splineMeta.segments.front().t0 + 1E-9).second;
            for (int i = idxMaster; i < splineMeta.segments.front().NumParameters() + idxMaster; ++i) {
                LOG_VAR(i)

            }
            // output: 2 3 4 5
        }
    }

    void LearnIMUTrajectoryRecovery(const CalibParamManager::Ptr &calibParamManager) {
        // gravity in map: [+0.0, +0.0, -9.797], only using the SO(3) for vector transform
        const Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -CalibConfig::BSpline::GRefineNorm);

        // create the sim data
        const double sTime = 0.0, eTime = M_PI * 2.0, deltaTime = 0.001;

        aligned_vector<Posed> poseSeqItoG;
        for (double t = sTime; t < eTime;) {
            Eigen::Vector3d trans;
            trans(0) = std::cos(t) * 2.0;
            trans(1) = std::sin(t) * 2.0;
            trans(2) = std::sin(5.0 * t) * 0.5;

            Eigen::Vector3d yAxis = -trans.normalized();
            Eigen::Vector3d xAxis = Eigen::Vector3d(-trans(1), trans(0), 0.0).normalized();
            Eigen::Vector3d zAxis = xAxis.cross(yAxis);
            Eigen::Matrix3d rotMatrix;
            rotMatrix.col(0) = xAxis;
            rotMatrix.col(1) = yAxis;
            rotMatrix.col(2) = zAxis;

            poseSeqItoG.emplace_back(Sophus::SO3d(rotMatrix), trans, t);
            t += deltaTime;
        }

        auto trajItoG = Trajectory::Create(deltaTime * 2.0, sTime, eTime);
        auto estimator = TrajectoryEstimator::Create(trajItoG, calibParamManager);
        for (const auto &item: poseSeqItoG) {
            estimator->AddSE3Measurement(item, OptimizationOption::OPT_POS | OptimizationOption::OPT_SO3, 1.0, 1.0);
        }

        // solve
        ceres::Solver::Summary summary = estimator->Solve();
        LOG_PLAINTEXT("solve finished, info:")
        LOG_PLAINTEXT(summary.BriefReport())

        // simu imu output
        auto meaVec = trajItoG->ComputeIMUMeasurement(
                poseSeqItoG.front().so3.inverse() * gravity, 1.0 / ((eTime - sTime) * 100.0)
        );
        // recovery trajectory
        auto trajRecovery = Trajectory::Create(0.02, sTime, eTime);
        estimator = TrajectoryEstimator::Create(trajRecovery, calibParamManager);
        for (const auto &item: meaVec) {
            estimator->AddIMUMeasurement(
                    item, poseSeqItoG.front().so3.inverse() * gravity,
                    OptimizationOption::OPT_SO3 | OptimizationOption::OPT_POS, 1.0, 1.0
            );
        }
        // pose constraint
        for (int i = 0; i < 2; ++i) {
            static std::default_random_engine engine(
                    std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_int_distribution<int> rand(0, static_cast<int>(poseSeqItoG.size()) - 1);
            estimator->AddSE3Measurement(
                    poseSeqItoG.at(rand(engine)), OptimizationOption::OPT_SO3 | OptimizationOption::OPT_POS,
                    1.0, 1.0
            );
        }
        // solve
        summary = estimator->Solve();
        LOG_PLAINTEXT("solve finished, info:")
        LOG_PLAINTEXT(summary.BriefReport())

        Viewer viewer;
        viewer.ShowPoseSequence(
                {PoseSeqDisplay(trajRecovery->Sampling(0.01), PoseSeqDisplay::Mode::COORD),
                 PoseSeqDisplay(trajItoG->Sampling(0.05), PoseSeqDisplay::Mode::ARROW)}, 1.0f
        );
        viewer.RunSingleThread();
    }
}

#endif //LIC_CALIB_LEARN_SE3_SPLINE_HPP
