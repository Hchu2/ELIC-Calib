//
// Created by csl on 12/14/22.
//

#ifndef LIC_CALIB_SYSTEM_OBSERVABILITY_H
#define LIC_CALIB_SYSTEM_OBSERVABILITY_H

#include "util/type_define.hpp"
#include "ceres/ceres.h"
#include "opencv2/imgproc.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"

namespace ns_elic {
    struct CeresFactor {
    private:
        ceres::CostFunction *_costFunction;
        std::vector<double *> _paramBlocks;
        std::size_t _costFunctorHashCode;

        // param address, jacobian matrix
        std::map<const double *, Eigen::MatrixXd> jacobians;
        Eigen::VectorXd residuals;

    public:
        CeresFactor(ceres::CostFunction *costFunction, const std::vector<double *> &paramBlocks,
                    std::size_t costFunctorHashCode);

        void Evaluate(const std::map<const double *, std::string> &targetParams);

        [[nodiscard]] const std::map<const double *, Eigen::MatrixXd> &GetJacobians() const;

        [[nodiscard]] const Eigen::VectorXd &GetResiduals() const;

        [[nodiscard]] size_t GetCostFunctorHashCode() const;
    };

    struct LMEquation {
    private:
        Eigen::MatrixXd _hMat;
        Eigen::VectorXd _bVec;

        // param name, param, dime
        std::vector<std::pair<std::string, std::size_t>> _paramDesc;

        Eigen::MatrixXd _hMatEchelonForm;
        Eigen::VectorXd _bVecEchelonForm;

        std::map<std::size_t, aligned_vector<Eigen::VectorXd>> _residualsMap;

    public:
        LMEquation(
                Eigen::MatrixXd hMat, Eigen::VectorXd bVec,
                const std::vector<std::pair<std::string, std::size_t>> &paramDesc,
                const std::map<std::size_t, aligned_vector<Eigen::VectorXd>> &residualsMap
        );

        [[nodiscard]] const LMEquation &
        SaveEquationToDisk(const std::string &filepath, bool echelonForm = false) const;

        [[nodiscard]] cv::Mat EquationGraph(bool echelonForm = false) const;

        [[nodiscard]] Eigen::VectorXd ZeroSpace() const;

        [[nodiscard]] const std::map<std::size_t, aligned_vector<Eigen::VectorXd>> &GetResidualsMap() const;

    protected:
        [[nodiscard]] const LMEquation &
        SaveEquationToDisk(const std::string &filepath, const Eigen::MatrixXd &hMat, const Eigen::VectorXd &bVec) const;

        [[nodiscard]] cv::Mat EquationGraph(const Eigen::MatrixXd &hMat, const Eigen::VectorXd &bVec) const;
    };
}


#endif //LIC_CALIB_SYSTEM_OBSERVABILITY_H
