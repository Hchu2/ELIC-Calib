//
// Created by csl on 10/4/22.
//

#include "core/rotation_estimator.h"
#include "util/utils.hpp"

namespace ns_elic {

    RotationEstimator::RotationEstimator() : _solveFlag(false), _targetToTraj() {}

    RotationEstimator::Ptr RotationEstimator::Create() {
        return std::make_shared<RotationEstimator>();
    }

    aligned_vector<Eigen::Matrix4d> RotationEstimator::OrganizeCoeffMatSeq(
            const Trajectory::Ptr &trajectory, const aligned_vector<OdomPosed> &toTargetPoseSeq) {
        aligned_vector<Eigen::Matrix4d> AMatSeq;

        for (int i = 1; i < toTargetPoseSeq.size(); ++i) {
            int curIdx = i, lastIdx = i - 1;
            auto curTimeStamp = toTargetPoseSeq[curIdx].timeStamp;
            auto lastTimeStamp = toTargetPoseSeq[lastIdx].timeStamp;

            // check time stamp
            if (!trajectory->TimeStampInRange(curTimeStamp) || !trajectory->TimeStampInRange(lastTimeStamp)) {
                continue;
            }

            Eigen::Quaterniond sensorCurToLast, trajCurToLast;
            double factor;

            {
                // sensor
                auto curToTarget = toTargetPoseSeq[curIdx].pose;
                auto lastToTarget = toTargetPoseSeq[lastIdx].pose;

                sensorCurToLast = Eigen::Quaterniond((lastToTarget.inverse() * curToTarget).topLeftCorner<3, 3>());
            }
            {
                // trajectory
                auto curToTarget = trajectory->pose(curTimeStamp).so3();
                auto lastToTarget = trajectory->pose(lastTimeStamp).so3();

                trajCurToLast = (lastToTarget.inverse() * curToTarget).unit_quaternion();
            }
            {
                Eigen::AngleAxisd sensorAngleAxis(sensorCurToLast.toRotationMatrix());
                Eigen::AngleAxisd trajAngleAxis(trajCurToLast.toRotationMatrix());
                // compute weight factor
                const static double radianToDegree = 180 / M_PI;
                double deltaAngle = radianToDegree * std::fabs(sensorAngleAxis.angle() - trajAngleAxis.angle());
                factor = deltaAngle > 1.0 ? 1.0 / deltaAngle : 1.0;
            }

            Eigen::Matrix4d lqMat = LeftQuatMatrix(sensorCurToLast);
            Eigen::Matrix4d rqMat = RightQuatMatrix(trajCurToLast);
            AMatSeq.push_back(factor * (lqMat - rqMat));
        }
        return AMatSeq;
    }

    void
    RotationEstimator::Estimate(const Trajectory::Ptr &trajectory, const aligned_vector<OdomPosed> &toTargetPoseSeq) {
        _solveFlag = false;

        aligned_vector<Eigen::Matrix4d> AMatSeq = OrganizeCoeffMatSeq(
                trajectory, toTargetPoseSeq
        );

        if (AMatSeq.size() < 15) {
            return;
        }

        Eigen::MatrixXd AMat(AMatSeq.size() * 4, 4);
        for (int i = 0; i < AMatSeq.size(); ++i) {
            AMat.block<4, 4>(i * 4, 0) = AMatSeq.at(i);
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(AMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector4d cov = svd.singularValues();

        if (cov(2) > 0.25) {
            // get result
            Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
            Eigen::Quaterniond quat(x);
            Sophus::SO3d trajToTarget(quat);

            _solveFlag = true;
            _targetToTraj = trajToTarget.inverse();
        }
    }

    bool RotationEstimator::SolveStatus() const {
        return _solveFlag;
    }

    const Sophus::SO3d &RotationEstimator::GetSO3TargetToTraj() const {
        return _targetToTraj;
    }

}