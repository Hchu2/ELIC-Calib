//
// Created by csl on 1/18/23.
//

#ifndef LIC_CALIB_MULTI_IMU_SPLINE_HPP
#define LIC_CALIB_MULTI_IMU_SPLINE_HPP

#include <utility>
#include <ostream>

#include "functors/se3_functor.hpp"
#include "core/pose.hpp"
#include "core/se3_spline.h"
#include "core/trajectory_estimator.h"
#include "viewer/pose_viewer.h"

namespace ns_elic {
    struct MultiIMUFunctorLearner {
    private:
        SplineMeta _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;
    public:
        explicit MultiIMUFunctorLearner(SplineMeta splineMeta, IMUFrame::Ptr imuFrame)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt) {}

        static auto
        Create(const SplineMeta &splineMeta, const IMUFrame::Ptr &imuFrame) {
            return new ceres::DynamicAutoDiffCostFunction<MultiIMUFunctorLearner>(
                    new MultiIMUFunctorLearner(splineMeta, imuFrame)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(MultiIMUFunctorLearner).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS | SO3_IjToIr ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            std::size_t R_offset;
            std::size_t P_offset;
            double u;
            _splineMeta.ComputeSplineIndex(_imuFrame->GetTimestamp(), R_offset, u);
            P_offset = R_offset + _splineMeta.NumParameters();

            std::size_t SO3_IjToIr_OFFSET = 2 * _splineMeta.NumParameters();

            Sophus::SO3<T> so3_IrToIr0;
            SO3Tangent<T> so3Vel_IrToIr0InBody, so3Acce_IrToIr0InBody;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
                    sKnots + R_offset, u, _dtInv, &so3_IrToIr0, &so3Vel_IrToIr0InBody, &so3Acce_IrToIr0InBody
            );

            Vector3<T> posAcce_IrToIr0InIr0;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate<T, 3, 2>(
                    sKnots + P_offset, u, _dtInv, &posAcce_IrToIr0InIr0
            );

            Eigen::Map<Sophus::SO3<T> const> const SO3_IjToIr(sKnots[SO3_IjToIr_OFFSET]);

            Sophus::SO3<T> so3_IjToIr0 = so3_IrToIr0 * SO3_IjToIr;
            SO3Tangent<T> so3Vel_IrToIr0InIr0 = so3_IrToIr0 * so3Vel_IrToIr0InBody;

            SO3Tangent<T> so3Vel_IjToIr0InIr0 = so3Vel_IrToIr0InIr0;

            Vector3<T> gyroPred = so3_IjToIr0.inverse() * so3Vel_IjToIr0InIr0;

            Vector3<T> gyroResiduals = gyroPred - _imuFrame->GetGyro().template cast<T>();

            residuals.template block<3, 1>(0, 0) = gyroResiduals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct NormalizedRefTrajectoryFunctorLearner {
    private:
        std::size_t _blockSize;

    public:
        explicit NormalizedRefTrajectoryFunctorLearner(std::size_t blockSize) : _blockSize(blockSize) {}

        static auto Create(std::size_t blockSize) {
            return new ceres::DynamicAutoDiffCostFunction<NormalizedRefTrajectoryFunctorLearner>(
                    new NormalizedRefTrajectoryFunctorLearner(blockSize)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(NormalizedRefTrajectoryFunctorLearner).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3_I0ToIr | ... | SO3_IjToIr | ... ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            Vector3<T> normSO3 = Vector3<T>::Zero();
            for (int SO3_IjToIr_OFFSET = 0; SO3_IjToIr_OFFSET < _blockSize; ++SO3_IjToIr_OFFSET) {
                Eigen::Map<Sophus::SO3<T> const> const SO3_IjToIr(sKnots[SO3_IjToIr_OFFSET]);
                normSO3 += SO3_IjToIr.log();
            }

            residuals.template block<3, 1>(0, 0) = normSO3;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class TrajectoryEstimatorLearner : public TrajectoryEstimator {
    public:
        using TrajectoryEstimator::TrajectoryEstimator;

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS | SO3_IjToIr ]
         */
        void
        AddMultiIMUMeasurement(const IMUFrame::Ptr &imuFrame, Sophus::SO3d &SO3_IjToIr) {
            // check time stamp
            if (!_trajectory->TimeStampInRange(imuFrame->GetTimestamp())) {
                return;
            }

            SplineMeta splineMeta;
            _trajectory->CalculateSplineMeta({{imuFrame->GetTimestamp(), imuFrame->GetTimestamp()}}, splineMeta);

            auto costFunc = MultiIMUFunctorLearner::Create(splineMeta, imuFrame);
            // so3 knots param block [each has four sub params]
            for (int i = 0; i < splineMeta.NumParameters(); i++) {
                costFunc->AddParameterBlock(4);
            }
            // pos knots param block [each has three sub params]
            for (int i = 0; i < splineMeta.NumParameters(); i++) {
                costFunc->AddParameterBlock(3);
            }
            // SO3_IjToIr
            costFunc->AddParameterBlock(4);

            costFunc->SetNumResiduals(3);

            std::vector<double *> paramBlockVec;

            AddCtrlPointsData(
                    paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS, splineMeta, false
            );

            AddCtrlPointsData(
                    paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS, splineMeta, false
            );

            paramBlockVec.emplace_back(SO3_IjToIr.data());
            this->AddParameterBlock(SO3_IjToIr.data(), 4, QUATER_MANIFOLD);

            this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        }

        /**
         * param blocks:
         * [ SO3_I0ToIr | ... | SO3_IjToIr | ... ]
         */
        void AddNormalizedRefTrajectory(const std::vector<Sophus::SO3d *> &SO3_IjToIr_VEC) {
            auto costFunc = NormalizedRefTrajectoryFunctorLearner::Create(SO3_IjToIr_VEC.size());

            std::vector<double *> paramBlockVec;

            for (auto &item: SO3_IjToIr_VEC) {
                // SO3_IjToIr
                costFunc->AddParameterBlock(4);
                paramBlockVec.emplace_back(item->data());
                this->AddParameterBlock(item->data(), 4, QUATER_MANIFOLD);
            }
            costFunc->SetNumResiduals(3);

            this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        }
    };

    struct MultiIMUSpline {
    public:
        struct TrajectoryPack {
            aligned_vector<Posed> poseSeq;
            aligned_vector<IMUFrame::Ptr> imuMeaSeq;
            Trajectory::Ptr trajectory;

            Sophus::SO3d SO3_IjToIr;
            Eigen::Vector3d POS_IjToIr;

            TrajectoryPack(const aligned_vector<Posed> &poseSeq, const aligned_vector<IMUFrame::Ptr> &imuMeaSeq,
                           Trajectory::Ptr trajectory, const Sophus::SE3d &se3IjToMain)
                    : poseSeq(poseSeq), imuMeaSeq(imuMeaSeq), trajectory(std::move(trajectory)),
                      SO3_IjToIr(se3IjToMain.so3()), POS_IjToIr(se3IjToMain.translation()) {}

            friend ostream &operator<<(ostream &os, const TrajectoryPack &pack) {
                os << "SO3_IjToMain: " << pack.SO3_IjToIr.log().transpose();
                return os;
            }
        };

    public:
        static aligned_vector<Posed> GetTrajectory(const double sTime, const double eTime, const double deltaTime) {
            aligned_vector<Posed> poseSeq;
            for (double t = sTime; t < eTime;) {
                Eigen::Vector3d trans;
                trans(0) = std::cos(t) * 2.0;
                trans(1) = std::sin(t) * 2.0;
                trans(2) = std::sin(5.0 * t) * 0.5;
                // trans(2) = 0.0;

                Eigen::Vector3d yAxis = -trans.normalized();
                Eigen::Vector3d xAxis = Eigen::Vector3d(-trans(1), trans(0), 0.0).normalized();
                Eigen::Vector3d zAxis = xAxis.cross(yAxis);
                Eigen::Matrix3d rotMatrix;
                rotMatrix.col(0) = xAxis;
                rotMatrix.col(1) = yAxis;
                rotMatrix.col(2) = zAxis;

                poseSeq.emplace_back(Sophus::SO3d(rotMatrix), trans, t);
                t += deltaTime;
            }
            return poseSeq;
        }

        static aligned_vector<Posed>
        UpdatePoseSequence(const aligned_vector<Posed> &refPoseSeq, const Sophus::SE3d &pose, bool atLeft = true) {
            aligned_vector<Posed> poseSeq;
            for (const auto &item: refPoseSeq) {
                if (atLeft) {
                    poseSeq.push_back(Posed::FromSE3(pose * item.se3(), item.timeStamp));
                } else {
                    poseSeq.push_back(Posed::FromSE3(item.se3() * pose, item.timeStamp));
                }
            }
            return poseSeq;
        }

        static void EstimateTrajectory(const aligned_vector<Posed> &poseSeq, const Trajectory::Ptr &trajectory,
                                       const CalibParamManager::Ptr &calibParamManager) {
            TrajectoryEstimator estimator(trajectory, calibParamManager);

            for (const auto &item: poseSeq) {
                estimator.AddSE3Measurement(item, OptimizationOption::OPT_POS | OptimizationOption::OPT_SO3, 1.0, 1.0);
            }
            // solve
            ceres::Solver::Summary summary = estimator.Solve();
            LOG_PLAINTEXT("solve finished, info:")
            LOG_PLAINTEXT(summary.BriefReport())
        }

        static void DisplayTrajectory(Viewer &viewer, const Trajectory::Ptr &trajectory, const double timeDis,
                                      const double sTime, const double eTime) {
            viewer.ShowPoseSequence(
                    {PoseSeqDisplay(trajectory->Sampling(timeDis, sTime, eTime), PoseSeqDisplay::Mode::COORD)}, 0.5f
            );
        }

        static void DisplayTrajectory(Viewer &viewer, const aligned_vector<Posed> &poseSeq) {
            viewer.ShowPoseSequence(
                    {PoseSeqDisplay(poseSeq, PoseSeqDisplay::Mode::COORD)}, 1.0f
            );
        }

        static Sophus::SE3d GeneratePoseBias() {
            constexpr double DEG_TO_RAD = M_PI / 180.0;
            static std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> aRand(-180, 180), pRand(-0.5, 0.5);
            auto a1 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 0, 1));
            auto a2 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 1, 0));
            auto a3 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(1, 0, 0));
            return {(a1 * a2 * a3).toRotationMatrix(), Eigen::Vector3d(pRand(engine), pRand(engine), pRand(engine))};
        }

    public:
        static void LearnSplineTransform(const CalibParamManager::Ptr &calibParamManager) {
            // create the sim data
            const double sTime = 0.0, eTime = M_PI * 2.0, deltaTime = 0.0001;

            aligned_vector<Posed> poseSeqIjToG = GetTrajectory(sTime, eTime, deltaTime);

            Sophus::SE3d SE3_IrToIj = GeneratePoseBias();
            Sophus::SE3d SE3_IjToIr = SE3_IrToIj.inverse();

            aligned_vector<Posed> poseSeqIrToG = UpdatePoseSequence(poseSeqIjToG, SE3_IrToIj, false);

            Sophus::SE3d GtoIr0 = poseSeqIrToG.front().se3().inverse();

            aligned_vector<Posed> poseSeqIjToIr0 = UpdatePoseSequence(poseSeqIjToG, GtoIr0, true);
            aligned_vector<Posed> poseSeqIrToIr0 = UpdatePoseSequence(poseSeqIrToG, GtoIr0, true);

            // estimate spline
            // auto trajIjToG = Trajectory::Create(deltaTime * 2.0, sTime, eTime);
            // auto trajIrToG = Trajectory::Create(deltaTime * 2.0, sTime, eTime);
            // EstimateTrajectory(poseSeqIjToG, trajIjToG, calibParamManager);
            // EstimateTrajectory(poseSeqIrToG, trajIrToG, calibParamManager);

            auto trajIjToIr0 = Trajectory::Create(deltaTime * 2.0, sTime, eTime);
            auto trajIrToIr0 = Trajectory::Create(deltaTime * 2.0, sTime, eTime);
            EstimateTrajectory(poseSeqIjToIr0, trajIjToIr0, calibParamManager);
            EstimateTrajectory(poseSeqIrToIr0, trajIrToIr0, calibParamManager);


            // test
            for (double t = sTime; t < eTime;) {
                auto curIjToTr0 = trajIjToIr0->getSo3Spline().evaluate(t);
                Eigen::Vector3d angVelIjInIr0 = curIjToTr0 * trajIjToIr0->getSo3Spline().velocityBody(t);

                auto curIrToTr0 = trajIrToIr0->getSo3Spline().evaluate(t);
                Eigen::Vector3d angVelIrInIr0 = curIrToTr0 * trajIrToIr0->getSo3Spline().velocityBody(t);
                Eigen::Vector3d angVelIjInIr0Pred = angVelIrInIr0;
                // LOG_VAR(angVelIrInIr0.transpose())
                // LOG_VAR(angVelIjInIr0Pred.transpose())
                // LOG_ENDL()

                Eigen::Vector3d angAccIrInIr0 = curIrToTr0 * trajIrToIr0->getSo3Spline().accelerationBody(t);
                Eigen::Vector3d POS_IjInIr = SE3_IjToIr.translation();

                Eigen::Vector3d linAccIjInIr0 = trajIjToIr0->getPosSpline().acceleration(t);
                Eigen::Vector3d linVelIjInIr0 = trajIjToIr0->getPosSpline().velocity(t);
                Eigen::Vector3d linAccIrInIr0 = trajIrToIr0->getPosSpline().acceleration(t);
                Eigen::Vector3d linVelIrInIr0 = trajIrToIr0->getPosSpline().velocity(t);

                Eigen::Vector3d linVelIjInIr0Pred =
                        -Sophus::SO3d::hat(curIrToTr0 * POS_IjInIr) * angVelIrInIr0 + linVelIrInIr0;
                // LOG_VAR(linVelIjInIr0.transpose())
                // LOG_VAR(linVelIjInIr0Pred.transpose())
                // LOG_ENDL()

                Eigen::Matrix3d tmpHat = Sophus::SO3d::hat(curIrToTr0 * POS_IjInIr);
                // Eigen::Vector3d linAccIjInIr0Pred =
                //         -tmpHat * angAccIrInIr0 + linAccIrInIr0
                //         + Sophus::SO3d::hat(tmpHat * angVelIrInIr0) * (Eigen::Matrix3d::Identity() - tmpHat) *
                //           angVelIrInIr0;
                Eigen::Vector3d linAccIjInIr0Pred = -tmpHat * angAccIrInIr0 + linAccIrInIr0 -
                                                    Sophus::SO3d::hat(angVelIrInIr0) * tmpHat * angVelIrInIr0;
                LOG_VAR(linAccIjInIr0.transpose())
                LOG_VAR(linAccIjInIr0Pred.transpose())
                LOG_ENDL()
                t += 0.1;
            }


            // display: the sampling pose sequence, knots sequence, origin data pose sequence
            Viewer viewer;
            // DisplayTrajectory(viewer, trajIjToG, sTime, eTime - 0.1);
            // DisplayTrajectory(viewer, trajIrToG, sTime, eTime - 0.1);
            DisplayTrajectory(viewer, trajIjToIr0, 0.05, sTime, eTime - 0.1);
            DisplayTrajectory(viewer, trajIrToIr0, 0.05, sTime, eTime - 0.1);
            viewer.RunSingleThread();
        }

        static void LearnMultiIMU(const CalibParamManager::Ptr &calibParamManager) {
            ns_log::ns_priv::stdLogger.setPrecision(10);
            // gravity in map: [+0.0, +0.0, -9.797], only using the SO(3) for vector transform
            const Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -CalibConfig::BSpline::GRefineNorm);

            // create the sim data
            const double sTime = 0.0, eTime = M_PI * 2.0, deltaTime = 0.001;

            aligned_vector<Posed> poseSeqMainToG = GetTrajectory(sTime, eTime, deltaTime);
            auto trajIrToIr0GroundTruth = Trajectory::Create(deltaTime * 2.0, sTime, eTime);
            EstimateTrajectory(poseSeqMainToG, trajIrToIr0GroundTruth, calibParamManager);

            std::vector<TrajectoryPack> imuVec;

            // simu multi imu data
            for (int i = 0; i != 5; ++i) {
                Sophus::SE3d SE3_IjToMain = GeneratePoseBias();
                aligned_vector<Posed> poseSeqIjToG = UpdatePoseSequence(poseSeqMainToG, SE3_IjToMain, false);
                auto trajIjToG = Trajectory::Create(deltaTime * 2.0, sTime, eTime);
                EstimateTrajectory(poseSeqIjToG, trajIjToG, calibParamManager);
                // 400 HZ imu measurements
                imuVec.emplace_back(
                        poseSeqIjToG, trajIjToG->ComputeIMUMeasurement(gravity, 1.0 / ((eTime - sTime) * 100.0)),
                        trajIjToG, SE3_IjToMain
                );
            }
            LOG_VAR(imuVec)

            // print ground truth
            for (int i = 1; i < imuVec.size(); ++i) {
                auto IjToI0 = imuVec.front().SO3_IjToIr.inverse() * imuVec.at(i).SO3_IjToIr;
                LOG_VAR(IjToI0.log().transpose())
            }

            // estimation
            // TODO: how to handle the rotation norm constraint
            auto normTraj = Trajectory::Create(0.02, sTime, eTime);
            auto estimator = TrajectoryEstimatorLearner(normTraj, calibParamManager);
            std::vector<Sophus::SO3d *> SO3_IjToIr_VEC;
            for (auto &item: imuVec) {
                for (const auto &frame: item.imuMeaSeq) {
                    estimator.AddMultiIMUMeasurement(frame, item.SO3_IjToIr);
                }
                // init as identity
                item.SO3_IjToIr = Sophus::SO3d();
                SO3_IjToIr_VEC.push_back(&item.SO3_IjToIr);
            }
            estimator.AddNormalizedRefTrajectory(SO3_IjToIr_VEC);
            // pose constraint
            Posed origin(normTraj->minTime());
            estimator.AddSO3Measurement(
                    origin, OptimizationOption::OPT_SO3, CalibConfig::Optimization::OptWeight::SO3Weight
            );
            LOG_VAR(imuVec)
            // solve
            ceres::Solver::Summary summary = estimator.Solve();
            LOG_PLAINTEXT("solve finished, info:")
            LOG_PLAINTEXT(summary.BriefReport())

            // print estimation result
            LOG_VAR(imuVec)
            for (int i = 1; i < imuVec.size(); ++i) {
                auto IjToI0 = imuVec.front().SO3_IjToIr.inverse() * imuVec.at(i).SO3_IjToIr;
                LOG_VAR(IjToI0.log().transpose())
            }

            // display
            // view
            Viewer viewer;
            for (const auto &item: imuVec) {
                DisplayTrajectory(viewer, item.trajectory, 0.05, sTime, eTime - 0.1);
            }
            auto normTrajPoseSeq = normTraj->Sampling(0.01, sTime, eTime - 0.1);
            for (auto &item: normTrajPoseSeq) {
                item.t = Eigen::Vector3d(item.timeStamp, 0.0, 0.0);
            }
            viewer.ShowPoseSequence(
                    {PoseSeqDisplay(normTrajPoseSeq, PoseSeqDisplay::Mode::COORD)}, 1.0f
            );
            viewer.RunSingleThread();
        }
    };

}

#endif //LIC_CALIB_MULTI_IMU_SPLINE_HPP
