//
// Created by csl on 12/14/22.
//

#include "core/system_observability.h"
#include "fstream"
#include "iomanip"
#include "slam-scene-viewer/colour.hpp"
#include "opencv2/highgui.hpp"
#include "thirdparty/logger/src/include/logger.h"
#include "util/utils.hpp"

namespace ns_elic {

    CeresFactor::CeresFactor(ceres::CostFunction *costFunction, const std::vector<double *> &paramBlocks,
                             std::size_t costFunctorHashCode)
            : _costFunction(costFunction), _paramBlocks(paramBlocks), _costFunctorHashCode(costFunctorHashCode) {}

    void CeresFactor::Evaluate(const std::map<const double *, std::string> &targetParams) {
        const auto &parameterBlockSizes = _costFunction->parameter_block_sizes();
        const int numParameterBlocks = static_cast<int>(parameterBlockSizes.size());
        const int numResiduals = _costFunction->num_residuals();

        std::vector<double> rawResiduals(numResiduals);
        std::vector<double *> rawJacobians(numParameterBlocks);

        // allocate
        for (int i = 0; i < numParameterBlocks; ++i) {
            rawJacobians.at(i) = new double[numResiduals * parameterBlockSizes.at(i)];
        }

        // evaluate
        _costFunction->Evaluate(_paramBlocks.data(), rawResiduals.data(), rawJacobians.data());

        // save rawResiduals
        residuals.resize(numResiduals);
        for (int i = 0; i < numResiduals; ++i) {
            residuals(i) = rawResiduals.at(i);
        }

        // save rawJacobians
        jacobians.clear();
        for (int i = 0; i < numParameterBlocks; ++i) {
            // the param block that we don't interest
            if (targetParams.find(_paramBlocks.at(i)) == targetParams.end()) {
                continue;
            }
            Eigen::MatrixXd jMat(numResiduals, parameterBlockSizes.at(i));
            for (int r = 0; r < numResiduals; ++r) {
                for (int c = 0; c < parameterBlockSizes.at(i); ++c) {
                    jMat(r, c) = rawJacobians.at(i)[r * parameterBlockSizes.at(i) + c];
                }
            }
            jacobians.insert(std::make_pair(_paramBlocks.at(i), jMat));
        }

        // deallocate
        for (int i = 0; i < numParameterBlocks; ++i) {
            delete[] rawJacobians.at(i);
        }
    }

    const std::map<const double *, Eigen::MatrixXd> &CeresFactor::GetJacobians() const {
        return jacobians;
    }

    const Eigen::VectorXd &CeresFactor::GetResiduals() const {
        return residuals;
    }

    size_t CeresFactor::GetCostFunctorHashCode() const {
        return _costFunctorHashCode;
    }

    LMEquation::LMEquation(Eigen::MatrixXd hMat, Eigen::VectorXd bVec,
                           const std::vector<std::pair<std::string, std::size_t>> &paramDesc,
                           const std::map<std::size_t, aligned_vector<Eigen::VectorXd>> &residualsMap)
            : _hMat(std::move(hMat)), _bVec(std::move(bVec)), _paramDesc(paramDesc), _residualsMap(residualsMap) {
        Eigen::MatrixXd hMatBar(_hMat.rows(), _hMat.cols() + 1);
        hMatBar.topLeftCorner(_hMat.rows(), _hMat.cols()) = _hMat;
        hMatBar.topRightCorner(_hMat.rows(), 1) = _bVec;
        hMatBar = ReducedRowEchelonForm(hMatBar);
        _hMatEchelonForm = hMatBar.topLeftCorner(_hMat.rows(), _hMat.cols());
        _bVecEchelonForm = hMatBar.topRightCorner(_hMat.rows(), 1);
    }

    cv::Mat LMEquation::EquationGraph(const Eigen::MatrixXd &hMat, const Eigen::VectorXd &bVec) const {
        LOG_INFO(_paramDesc)
        constexpr static int GRID_SIZE = 50;

        const int rows = static_cast<int>(hMat.rows()), cols = static_cast<int>(hMat.cols() + 4);
        cv::Mat img(rows * GRID_SIZE, cols * GRID_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));

        double maxVal = std::max(hMat.maxCoeff(), bVec.maxCoeff());
        double minVal = std::min(hMat.minCoeff(), bVec.minCoeff());

        // hessian matrix
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < rows; ++c) {
                const double val = hMat(r, c);
                ns_viewer::Colour color;
                if (val > 0.0) {
                    color = ns_viewer::Colour::Hsv(
                            0.0f, static_cast<float>(std::log2(val + 1) / std::log2(maxVal + 1)), 1.0f);
                } else {
                    color = ns_viewer::Colour::Hsv(
                            0.6f, static_cast<float>(std::log2(-val + 1) / std::log2(-minVal + 1)), 1.0f
                    );
                }

                img(cv::Range(r * GRID_SIZE, (r + 1) * GRID_SIZE), cv::Range(c * GRID_SIZE, (c + 1) * GRID_SIZE))
                        .setTo(cv::Scalar(color.b * 255.0, color.g * 255.0, color.r * 255.0));

            }
            // bVec
            const double val = bVec(r);
            ns_viewer::Colour color;
            if (val > 0.0) {
                color = ns_viewer::Colour::Hsv(
                        0.0f, static_cast<float>(std::log2(val + 1) / std::log2(maxVal + 1)), 1.0f
                );
            } else {
                color = ns_viewer::Colour::Hsv(
                        0.6f, static_cast<float>(std::log2(-val + 1) / std::log2(-minVal + 1)), 1.0f
                );
            }
            img(cv::Range(r * GRID_SIZE, (r + 1) * GRID_SIZE),
                cv::Range(static_cast<int>(hMat.cols() + 3) * GRID_SIZE,
                          static_cast<int>(hMat.cols() + 4) * GRID_SIZE))
                    .setTo(cv::Scalar(color.b * 255.0, color.g * 255.0, color.r * 255.0));
        }
        // x
        int cr = 0;
        for (const auto &[pName, pDime]: _paramDesc) {
            ns_viewer::Colour color = ns_viewer::Colour::Hsv(
                    0.0f, 0.0f, static_cast<float>(cr) / static_cast<float>(cols) + 0.25f
            );
            img(cv::Range(cr * GRID_SIZE, static_cast<int>((cr + pDime)) * GRID_SIZE),
                cv::Range(static_cast<int>(hMat.cols() + 1) * GRID_SIZE,
                          static_cast<int>(hMat.cols() + 2) * GRID_SIZE)
            ).setTo(cv::Scalar(color.b * 255.0, color.g * 255.0, color.r * 255.0));
            cr += static_cast<int>(pDime);
        }

        // light line
        for (int r = 0; r < rows; ++r) {
            // hMat
            img(cv::Range(r * GRID_SIZE, r * GRID_SIZE + 1), cv::Range(0, static_cast<int>(hMat.cols()) * GRID_SIZE))
                    .setTo(cv::Scalar(0, 0, 0));
            // x
            img(cv::Range(r * GRID_SIZE, r * GRID_SIZE + 1),
                cv::Range(static_cast<int>(hMat.cols() + 1) * GRID_SIZE,
                          static_cast<int>(hMat.cols() + 2) * GRID_SIZE)
            ).setTo(cv::Scalar(0, 0, 0));
            // bVec
            img(cv::Range(r * GRID_SIZE, r * GRID_SIZE + 1),
                cv::Range(static_cast<int>(hMat.cols() + 3) * GRID_SIZE,
                          static_cast<int>(hMat.cols() + 4) * GRID_SIZE)
            ).setTo(cv::Scalar(0, 0, 0));
        }
        {
            // hMat
            img.col(cols * GRID_SIZE - 1).setTo(cv::Scalar(0, 0, 0));
            // hMat
            img(cv::Range(rows * GRID_SIZE - 1, rows * GRID_SIZE),
                cv::Range(0, static_cast<int>(hMat.cols()) * GRID_SIZE)).setTo(cv::Scalar(0, 0, 0));
            // x
            img(cv::Range(rows * GRID_SIZE - 1, rows * GRID_SIZE), cv::Range(
                    static_cast<int>(hMat.cols() + 1) * GRID_SIZE, static_cast<int>(hMat.cols() + 2) * GRID_SIZE
            )).setTo(cv::Scalar(0, 0, 0));
            // bVec
            img(cv::Range(rows * GRID_SIZE - 1, rows * GRID_SIZE), cv::Range(
                    static_cast<int>(hMat.cols() + 3) * GRID_SIZE, static_cast<int>(hMat.cols() + 4) * GRID_SIZE
            )).setTo(cv::Scalar(0, 0, 0));
        }
        for (int c = 0; c < cols; ++c) {
            img(cv::Range(0, rows * GRID_SIZE), cv::Range(c * GRID_SIZE, c * GRID_SIZE + 1))
                    .setTo(cv::Scalar(0, 0, 0));
        }
        // weight line
        cr = 0;
        for (const auto &[pName, pDime]: _paramDesc) {
            if (cr == 0) {
                cr += static_cast<int>(pDime);
                continue;
            }
            // hMat
            img(cv::Range(cr * GRID_SIZE, cr * GRID_SIZE + 3), cv::Range(0, static_cast<int>(hMat.cols()) * GRID_SIZE))
                    .setTo(cv::Scalar(0, 0, 0));
            // x
            img(cv::Range(cr * GRID_SIZE, cr * GRID_SIZE + 3),
                cv::Range(static_cast<int>(hMat.cols() + 1) * GRID_SIZE,
                          static_cast<int>(hMat.cols() + 2) * GRID_SIZE)
            ).setTo(cv::Scalar(0, 0, 0));
            // bVec
            img(cv::Range(cr * GRID_SIZE, cr * GRID_SIZE + 3),
                cv::Range(static_cast<int>(hMat.cols() + 3) * GRID_SIZE,
                          static_cast<int>(hMat.cols() + 4) * GRID_SIZE)
            ).setTo(cv::Scalar(0, 0, 0));
            img(cv::Range(0, rows * GRID_SIZE), cv::Range(cr * GRID_SIZE, cr * GRID_SIZE + 3))
                    .setTo(cv::Scalar(0, 0, 0));

            cr += static_cast<int>(pDime);
        }

        return img;
    }

    cv::Mat LMEquation::EquationGraph(bool echelonForm) const {
        if (echelonForm) {
            return EquationGraph(_hMatEchelonForm, _bVecEchelonForm);
        } else {
            return EquationGraph(_hMat, _bVec);
        }
    }

    Eigen::VectorXd LMEquation::ZeroSpace() const {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(_hMat);
        return lu.kernel();
    }

    const std::map<std::size_t, aligned_vector<Eigen::VectorXd>> &LMEquation::GetResidualsMap() const {
        return _residualsMap;
    }

    const LMEquation &
    LMEquation::SaveEquationToDisk(const std::string &filepath, const Eigen::MatrixXd &hMat,
                                   const Eigen::VectorXd &bVec) const {
        std::ofstream file(filepath, std::ios::out);
        cereal::JSONOutputArchive ar(file);
        ar.setNextName("param_blocks");
        ar.startNode();
        int count = 0;
        for (const auto &item: _paramDesc) {
            ar.setNextName(("blocks_" + std::to_string(count++)).c_str());
            ar.startNode();
            ar(cereal::make_nvp("name", item.first), cereal::make_nvp("dime", item.second));
            ar.finishNode();
        }
        ar.finishNode();
        ar(
                cereal::make_nvp("h_matrix", EigenMatXToVector(hMat)),
                cereal::make_nvp("b_vector", EigenVecXToVector(bVec))
        );
        return *this;
    }

    const LMEquation &
    LMEquation::SaveEquationToDisk(const std::string &filepath, bool echelonForm) const {
        if (echelonForm) {
            return SaveEquationToDisk(filepath, _hMatEchelonForm, _bVecEchelonForm);
        } else {
            return SaveEquationToDisk(filepath, _hMat, _bVec);
        }
    }
}