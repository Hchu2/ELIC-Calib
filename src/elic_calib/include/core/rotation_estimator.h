//
// Created by csl on 10/4/22.
//

#ifndef LIC_CALIB_ROTATION_ESTIMATOR_H
#define LIC_CALIB_ROTATION_ESTIMATOR_H

#include "core/pose.hpp"
#include "core/se3_spline.h"

namespace ns_elic {
    class RotationEstimator {
        using Ptr = std::shared_ptr<RotationEstimator>;

    private:
        bool _solveFlag;
        Sophus::SO3d _targetToTraj;

    public:
        RotationEstimator();

        static RotationEstimator::Ptr Create();

        void Estimate(const Trajectory::Ptr &trajectory, const aligned_vector<OdomPosed> &toTargetPoseSeq);

        [[nodiscard]] bool SolveStatus() const;

        [[nodiscard]] const Sophus::SO3d &GetSO3TargetToTraj() const;

    protected:
        static aligned_vector<Eigen::Matrix4d>
        OrganizeCoeffMatSeq(const Trajectory::Ptr &trajectory, const aligned_vector<OdomPosed> &toTargetPoseSeq);
    };
}


#endif //LIC_CALIB_ROTATION_ESTIMATOR_H
