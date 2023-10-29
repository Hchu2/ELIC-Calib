//
// Created by csl on 10/1/22.
//

#ifndef LIC_CALIB_CALIB_PARAM_MANAGER_H
#define LIC_CALIB_CALIB_PARAM_MANAGER_H

#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "memory"
#include "sophus/se3.hpp"
#include "util/type_define.hpp"
#include "util/utils.hpp"
#include "slam-scene-viewer/scene_viewer.h"
#include "config/calib_config.h"
#include "cereal/types/map.hpp"

namespace ns_elic {
    struct CalibParamManager {
    public:
        using Ptr = std::shared_ptr<CalibParamManager>;

    public:
        // trans radian angle to degree angle
        constexpr static double RAD_TO_DEG = 180.0 / M_PI;
        // trans degree angle to radian angle
        constexpr static double DEG_TO_RAD = M_PI / 180.0;

        // extrinsic
        struct {
            // imus to reference imu
            aligned_map<std::string, Sophus::SO3d> SO3_IjToIr;
            aligned_map<std::string, Eigen::Vector3d> POS_IjInIr;

            // lidars to reference lidar
            aligned_map<std::string, Sophus::SO3d> SO3_LmToLr;
            aligned_map<std::string, Eigen::Vector3d> POS_LmInLr;

            // reference lidar to reference imu
            Sophus::SO3d SO3_LrToIr;
            Eigen::Vector3d POS_LrInIr;

            // cameras to reference imu
            aligned_map<std::string, Sophus::SO3d> SO3_CkToIr;
            aligned_map<std::string, Eigen::Vector3d> POS_CkInIr;

            // S2Manifold
            Eigen::Vector3d GRAVITY;

            // lie algebra vector space se3
            [[nodiscard]] Sophus::SE3d SE3_IjToIr(const std::string &topic) const {
                return {SO3_IjToIr.at(topic), POS_IjInIr.at(topic)};
            }

            [[nodiscard]] Sophus::SE3d SE3_LmToLr(const std::string &topic) const {
                return {SO3_LmToLr.at(topic), POS_LmInLr.at(topic)};
            }

            [[nodiscard]] Sophus::SE3d SE3_LrToIr() const {
                return {SO3_LrToIr, POS_LrInIr};
            }

            [[nodiscard]] Sophus::SE3d SE3_LmToIr(const std::string &topic) const {
                return SE3_LrToIr() * SE3_LmToLr(topic);
            }

            [[nodiscard]] Sophus::SE3d SE3_CkToIr(const std::string &topic) const {
                return {SO3_CkToIr.at(topic), POS_CkInIr.at(topic)};
            }

            // quaternion
            [[nodiscard]] Eigen::Quaterniond Q_IjToIr(const std::string &topic) const {
                return SO3_IjToIr.at(topic).unit_quaternion();
            }

            [[nodiscard]] Eigen::Quaterniond Q_LmToLr(const std::string &topic) const {
                return SO3_LmToLr.at(topic).unit_quaternion();
            }

            [[nodiscard]] Eigen::Quaterniond Q_LrToLr() const {
                return SO3_LrToIr.unit_quaternion();
            }

            [[nodiscard]] Eigen::Quaterniond Q_LmToIr(const std::string &topic) const {
                return Q_LrToLr() * Q_LmToLr(topic);
            }

            [[nodiscard]] Eigen::Quaterniond Q_CkToIr(const std::string &topic) const {
                return SO3_CkToIr.at(topic).unit_quaternion();
            }

            // transform matrix
            [[nodiscard]] Eigen::Matrix4d T_IjToIr(const std::string &topic) const {
                return SE3_IjToIr(topic).matrix();
            }

            [[nodiscard]] Eigen::Matrix4d T_LmToLr(const std::string &topic) const {
                return SE3_LmToLr(topic).matrix();
            }

            [[nodiscard]] Eigen::Matrix4d T_LrToIr() const {
                return SE3_LrToIr().matrix();
            }

            [[nodiscard]] Eigen::Matrix4d T_LmToIr(const std::string &topic) const {
                return T_LrToIr() * T_LmToLr(topic);
            }

            [[nodiscard]] Eigen::Matrix4d T_CkToIr(const std::string &topic) const {
                return SE3_CkToIr(topic).matrix();
            }

            // rotation matrix
            [[nodiscard]] Eigen::Matrix3d R_IjToIr(const std::string &topic) const {
                return SO3_IjToIr.at(topic).matrix();
            }

            [[nodiscard]] Eigen::Matrix3d R_LmToLr(const std::string &topic) const {
                return SO3_LmToLr.at(topic).matrix();
            }

            [[nodiscard]] Eigen::Matrix3d R_LrToIr() const {
                return SO3_LrToIr.matrix();
            }

            [[nodiscard]] Eigen::Matrix3d R_LmToIr(const std::string &topic) const {
                return R_LrToIr() * R_LmToLr(topic);
            }

            [[nodiscard]] Eigen::Matrix3d R_CkToIr(const std::string &topic) const {
                return SO3_CkToIr.at(topic).matrix();
            }

            // the euler angles [radian and degree format]
            [[nodiscard]] Eigen::Vector3d EULER_IjToIr_RAD(const std::string &topic) const {
                return Q_IjToIr(topic).toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] Eigen::Vector3d EULER_LmToLr_RAD(const std::string &topic) const {
                return Q_LmToLr(topic).toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] Eigen::Vector3d EULER_LrToIr_RAD() const {
                return Q_LrToLr().toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] Eigen::Vector3d EULER_LmToIr_RAD(const std::string &topic) const {
                return Q_LmToIr(topic).toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] Eigen::Vector3d EULER_CkToIr_RAD(const std::string &topic) const {
                return Q_CkToIr(topic).toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] Eigen::Vector3d EULER_IjToIr_DEG(const std::string &topic) const {
                auto euler = EULER_IjToIr_RAD(topic);
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

            [[nodiscard]] Eigen::Vector3d EULER_LmToLr_DEG(const std::string &topic) const {
                auto euler = EULER_LmToLr_RAD(topic);
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

            [[nodiscard]] Eigen::Vector3d EULER_LrToIr_DEG() const {
                auto euler = EULER_LrToIr_RAD();
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

            [[nodiscard]] Eigen::Vector3d EULER_LmToIr_DEG(const std::string &topic) const {
                auto euler = EULER_LmToIr_RAD(topic);
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

            [[nodiscard]] Eigen::Vector3d EULER_CkToIr_DEG(const std::string &topic) const {
                auto euler = EULER_CkToIr_RAD(topic);
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

            // Serialization
            template<class Archive>
            void save(Archive &archive) const {
                std::map<std::string, std::vector<double>> MAP_Q_IjToIr, MAP_EULER_IjToIr, MAP_POS_IjInIr;
                std::map<std::string, std::vector<double>> MAP_Q_CkToIr, MAP_EULER_CkToIr, MAP_POS_CkInIr;
                std::map<std::string, std::vector<double>> MAP_Q_LmToLr, MAP_EULER_LmToLr, MAP_POS_LmInLr;

                for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
                    MAP_Q_IjToIr.insert({topic, EigenQuaternionToVector(Q_IjToIr(topic))});
                    MAP_EULER_IjToIr.insert({topic, EigenVecToVector(EULER_IjToIr_DEG(topic))});
                    MAP_POS_IjInIr.insert({topic, EigenVecToVector(POS_IjInIr.at(topic))});
                }

                for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
                    MAP_Q_LmToLr.insert({topic, EigenQuaternionToVector(Q_LmToLr(topic))});
                    MAP_EULER_LmToLr.insert({topic, EigenVecToVector(EULER_LmToLr_DEG(topic))});
                    MAP_POS_LmInLr.insert({topic, EigenVecToVector(POS_LmInLr.at(topic))});
                }

                archive(
                        cereal::make_nvp("Q_IjToIr", MAP_Q_IjToIr),
                        cereal::make_nvp("EULER_IjToIr", MAP_EULER_IjToIr),
                        cereal::make_nvp("POS_IjInIr", MAP_POS_IjInIr),

                        cereal::make_nvp("Q_LmToLr", MAP_Q_LmToLr),
                        cereal::make_nvp("EULER_LmToLr", MAP_EULER_LmToLr),
                        cereal::make_nvp("POS_LmInLr", MAP_POS_LmInLr),

                        cereal::make_nvp("Q_CkToIr", MAP_Q_CkToIr),
                        cereal::make_nvp("EULER_CkToIr", MAP_EULER_CkToIr),
                        cereal::make_nvp("POS_CkInIr", MAP_POS_CkInIr),

                        cereal::make_nvp("Q_LrToIr", EigenQuaternionToVector(Q_LrToLr())),
                        cereal::make_nvp("EULER_LrToIr", EigenVecToVector(EULER_LrToIr_DEG())),
                        cereal::make_nvp("POS_LrInIr", EigenVecToVector(POS_LrInIr)),

                        cereal::make_nvp("GRAVITY", EigenVecToVector(GRAVITY))
                );
            }

            template<class Archive>
            void load(Archive &archive) {
                // important!!
                SO3_IjToIr.clear(), POS_IjInIr.clear();
                SO3_CkToIr.clear(), POS_CkInIr.clear();
                SO3_LmToLr.clear(), POS_LmInLr.clear();
                std::map<std::string, std::vector<double>> MAP_Q_IjToIr, MAP_POS_IjInIr;
                std::map<std::string, std::vector<double>> MAP_Q_CkToIr, MAP_POS_CkInIr;
                std::map<std::string, std::vector<double>> MAP_Q_LmToLr, MAP_POS_LmInLr;
                std::vector<double> Q_LrToIr_VEC, POS_LrInIr_VEC, GRAVITY_REFINE_VEC;

                archive(
                        cereal::make_nvp("Q_IjToIr", MAP_Q_IjToIr),
                        cereal::make_nvp("POS_IjInIr", MAP_POS_IjInIr),

                        cereal::make_nvp("Q_LmToLr", MAP_Q_LmToLr),
                        cereal::make_nvp("POS_LmInLr", MAP_POS_LmInLr),

                        cereal::make_nvp("Q_CkToIr", MAP_Q_CkToIr),
                        cereal::make_nvp("POS_CkInIr", MAP_POS_CkInIr),

                        cereal::make_nvp("Q_LrToIr", Q_LrToIr_VEC),
                        cereal::make_nvp("POS_LrInIr", POS_LrInIr_VEC),

                        cereal::make_nvp("GRAVITY", EigenVecToVector(GRAVITY))
                );
                // imu
                for (const auto &[topic, data]: MAP_Q_IjToIr) {
                    SO3_IjToIr.insert({topic, Sophus::SO3d(VectorToEigenQuaternion(data))});
                }
                for (const auto &[topic, data]: MAP_POS_IjInIr) {
                    POS_IjInIr.insert({topic, VectorToEigenVec<double, 3>(data)});
                }
                // lidar
                for (const auto &[topic, data]: MAP_Q_LmToLr) {
                    SO3_LmToLr.insert({topic, Sophus::SO3d(VectorToEigenQuaternion(data))});
                }
                for (const auto &[topic, data]: MAP_POS_LmInLr) {
                    POS_LmInLr.insert({topic, VectorToEigenVec<double, 3>(data)});
                }
                // camera
                for (const auto &[topic, data]: MAP_Q_CkToIr) {
                    SO3_CkToIr.insert({topic, Sophus::SO3d(VectorToEigenQuaternion(data))});
                }
                for (const auto &[topic, data]: MAP_POS_CkInIr) {
                    POS_CkInIr.insert({topic, VectorToEigenVec<double, 3>(data)});
                }

                SO3_LrToIr = Sophus::SO3d(VectorToEigenQuaternion(Q_LrToIr_VEC));
                POS_LrInIr = VectorToEigenVec<double, 3>(POS_LrInIr_VEC);

                GRAVITY = VectorToEigenVec<double, 3>(GRAVITY_REFINE_VEC);
            }

            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;

#define SAVE_EXTRI_INFO(param) infoMap.insert(std::make_pair(param.data(), #param));

                // imu
                for (const auto &[topic, data]: SO3_IjToIr) {
                    infoMap.insert({data.data(), "SO3_IjToIr[" + topic + "]"});
                }
                for (const auto &[topic, data]: POS_IjInIr) {
                    infoMap.insert({data.data(), "POS_IjInIr[" + topic + "]"});
                }

                // lidar
                for (const auto &[topic, data]: SO3_LmToLr) {
                    infoMap.insert({data.data(), "SO3_LmToLr[" + topic + "]"});
                }
                for (const auto &[topic, data]: POS_LmInLr) {
                    infoMap.insert({data.data(), "POS_LmInLr[" + topic + "]"});
                }

                // camera
                for (const auto &[topic, data]: SO3_CkToIr) {
                    infoMap.insert({data.data(), "SO3_CkToIr[" + topic + "]"});
                }
                for (const auto &[topic, data]: POS_CkInIr) {
                    infoMap.insert({data.data(), "POS_CkInIr[" + topic + "]"});
                }

                SAVE_EXTRI_INFO(SO3_LrToIr)
                SAVE_EXTRI_INFO(POS_LrInIr)

#undef SAVE_EXTRI_INFO

                return infoMap;
            }

            std::vector<Sophus::SO3d *> SO3_IjToI_AddressVec() {
                std::vector<Sophus::SO3d *> addressVec(SO3_IjToIr.size());
                std::transform(SO3_IjToIr.begin(), SO3_IjToIr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

            std::vector<Eigen::Vector3d *> POS_IjInI_AddressVec() {
                std::vector<Eigen::Vector3d *> addressVec(POS_IjInIr.size());
                std::transform(POS_IjInIr.begin(), POS_IjInIr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

            std::vector<Sophus::SO3d *> SO3_LmToLr_AddressVec() {
                std::vector<Sophus::SO3d *> addressVec(SO3_LmToLr.size());
                std::transform(SO3_LmToLr.begin(), SO3_LmToLr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

            std::vector<Eigen::Vector3d *> POS_LmInLr_AddressVec() {
                std::vector<Eigen::Vector3d *> addressVec(POS_LmInLr.size());
                std::transform(POS_LmInLr.begin(), POS_LmInLr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

        public:
            [[nodiscard]] static Eigen::Vector3d EULER_RAD(const Sophus::SO3d &so3) {
                return so3.unit_quaternion().toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] static Eigen::Vector3d EULER_DEG(const Sophus::SO3d &so3) {
                auto euler = EULER_RAD(so3);
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } EXTRI{};

        // temporal
        struct {
            // [ topic, time offset ]
            std::map<std::string, double> TIME_OFFSET_IjToIr;

            // [ topic, time offset ]
            std::map<std::string, double> TIME_OFFSET_LmToLr;

            // [ topic, time offset ]
            std::map<std::string, double> TIME_OFFSET_CkToIr;

            double TIME_OFFSET_LrToIr;

            // [ topic, readout time ]
            std::map<std::string, double> TIME_READOUT;

            // Serialization
            template<class Archive>
            void save(Archive &archive) const {
                archive(
                        cereal::make_nvp("TIME_OFFSET_IjToIr", TIME_OFFSET_IjToIr),
                        cereal::make_nvp("TIME_OFFSET_LmToLr", TIME_OFFSET_LmToLr),
                        cereal::make_nvp("TIME_OFFSET_CkToIr", TIME_OFFSET_CkToIr),
                        cereal::make_nvp("TIME_OFFSET_LrToIr", TIME_OFFSET_LrToIr),
                        cereal::make_nvp("TIME_READOUT", TIME_READOUT)
                );
            }

            template<class Archive>
            void load(Archive &archive) {
                TIME_OFFSET_IjToIr.clear();
                archive(
                        cereal::make_nvp("TIME_OFFSET_IjToIr", TIME_OFFSET_IjToIr),
                        cereal::make_nvp("TIME_OFFSET_LmToLr", TIME_OFFSET_LmToLr),
                        cereal::make_nvp("TIME_OFFSET_CkToIr", TIME_OFFSET_CkToIr),
                        cereal::make_nvp("TIME_OFFSET_LrToIr", TIME_OFFSET_LrToIr),
                        cereal::make_nvp("TIME_READOUT", TIME_READOUT)
                );
            }

            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;

#define SAVE_TEMPORAL_INFO(param) infoMap.insert(std::make_pair(&param, #param));

                for (const auto &[topic, data]: TIME_OFFSET_IjToIr) {
                    infoMap.insert({&data, "TIME_OFFSET_IjToIr[" + topic + "]"});
                }

                for (const auto &[topic, data]: TIME_OFFSET_LmToLr) {
                    infoMap.insert({&data, "TIME_OFFSET_LmToLr[" + topic + "]"});
                }

                for (const auto &[topic, data]: TIME_OFFSET_CkToIr) {
                    infoMap.insert({&data, "TIME_OFFSET_CkToIr[" + topic + "]"});
                    infoMap.insert({&TIME_READOUT.at(topic), "TIME_READOUT[" + topic + "]"});
                }

                SAVE_TEMPORAL_INFO(TIME_OFFSET_LrToIr)


#undef SAVE_TEMPORAL_INFO

                return infoMap;
            }

            std::vector<double *> TIME_OFFSET_IjToI_AddressVec() {
                std::vector<double *> addressVec(TIME_OFFSET_IjToIr.size());
                std::transform(TIME_OFFSET_IjToIr.begin(), TIME_OFFSET_IjToIr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

            std::vector<double *> TIME_OFFSET_LmToLr_AddressVec() {
                std::vector<double *> addressVec(TIME_OFFSET_LmToLr.size());
                std::transform(TIME_OFFSET_LmToLr.begin(), TIME_OFFSET_LmToLr.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }
        } TEMPORAL{};

        // intrinsic
        struct {
            struct IMUParamPack {
                struct {
                    Eigen::Vector3d BIAS;
                    /**
                     * MAP_COEFF: [v1, v2, v3, v4, v5, v6]^T
                     * mapMatrix:
                     *   v1 & v4 & v5
                     *    0 & v2 & v6
                     *    0 &  0 & v3
                     * f(measure) = mapMat * f(real) + bias
                     */
                    Eigen::Vector6d MAP_COEFF;

                    // organize the vector to a matrix
                    [[nodiscard]] Eigen::Matrix3d MapMatrix() const {
                        Eigen::Matrix3d mat;
                        mat.diagonal() = Eigen::Map<const Eigen::Vector3d>(MAP_COEFF.data(), 3);
                        mat(0, 1) = *(MAP_COEFF.data() + 3);
                        mat(0, 2) = *(MAP_COEFF.data() + 4);
                        mat(1, 2) = *(MAP_COEFF.data() + 5);
                        return Eigen::Matrix3d(MAP_COEFF.data());
                    }

                    // Serialization
                    template<class Archive>
                    void save(Archive &archive) const {
                        archive(
                                cereal::make_nvp("BIAS", EigenVecToVector(BIAS)),
                                cereal::make_nvp("MAP_COEFF", EigenVecToVector(MAP_COEFF))
                        );
                    }


                    template<class Archive>
                    void load(Archive &archive) {
                        std::vector<double> BIAS_VEC, MAP_COEFF_VEC;
                        archive(
                                cereal::make_nvp("BIAS", BIAS_VEC),
                                cereal::make_nvp("MAP_COEFF", MAP_COEFF_VEC)
                        );
                        BIAS = VectorToEigenVec<double, 3>(BIAS_VEC);
                        MAP_COEFF = VectorToEigenVec<double, 6>(MAP_COEFF_VEC);
                    }

                public:
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                } ACCE;

                struct {
                    Eigen::Vector3d BIAS;
                    /**
                     * MAP_COEFF: [v1, v2, v3, v4, v5, v6]^T
                     * mapMatrix:
                     *   v1 & v4 & v5
                     *    0 & v2 & v6
                     *    0 &  0 & v3
                     * g(measure) = mapMat * g(real) + bias
                     */
                    Eigen::Vector6d MAP_COEFF;

                    // organize the vector to a matrix
                    [[nodiscard]] Eigen::Matrix3d MapMatrix() const {
                        Eigen::Matrix3d mat;
                        mat.diagonal() = Eigen::Map<const Eigen::Vector3d>(MAP_COEFF.data(), 3);
                        mat(0, 1) = *(MAP_COEFF.data() + 3);
                        mat(0, 2) = *(MAP_COEFF.data() + 4);
                        mat(1, 2) = *(MAP_COEFF.data() + 5);
                        return Eigen::Matrix3d(MAP_COEFF.data());
                    }

                    // Serialization
                    template<class Archive>
                    void save(Archive &archive) const {
                        archive(
                                cereal::make_nvp("BIAS", EigenVecToVector(BIAS)),
                                cereal::make_nvp("MAP_COEFF", EigenVecToVector(MAP_COEFF))
                        );
                    }


                    template<class Archive>
                    void load(Archive &archive) {
                        std::vector<double> BIAS_VEC, MAP_COEFF_VEC;
                        archive(
                                cereal::make_nvp("BIAS", BIAS_VEC),
                                cereal::make_nvp("MAP_COEFF", MAP_COEFF_VEC)
                        );
                        BIAS = VectorToEigenVec<double, 3>(BIAS_VEC);
                        MAP_COEFF = VectorToEigenVec<double, 6>(MAP_COEFF_VEC);
                    }

                public:
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                } GYRO;

                Sophus::SO3d SO3_AtoG;


                // quaternion
                [[nodiscard]] Eigen::Quaterniond Q_AtoG() const {
                    return SO3_AtoG.unit_quaternion();
                }

                // euler angles
                [[nodiscard]] Eigen::Vector3d EULER_AtoG_RAD() const {
                    return Q_AtoG().toRotationMatrix().eulerAngles(0, 1, 2);
                }

                [[nodiscard]] Eigen::Vector3d EULER_AtoG_DEG() const {
                    auto euler = EULER_AtoG_RAD();
                    for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                    return euler;
                }

                // Serialization
                template<class Archive>
                void save(Archive &archive) const {
                    archive(
                            cereal::make_nvp("ACCE", ACCE),
                            cereal::make_nvp("GYRO", GYRO),
                            cereal::make_nvp("Q_AtoG", EigenQuaternionToVector(Q_AtoG())),
                            cereal::make_nvp("EULER_AtoG", EigenVecToVector(EULER_AtoG_DEG()))
                    );
                }


                template<class Archive>
                void load(Archive &archive) {
                    std::vector<double> Q_AtoG_VEC, EULER_AtoG_VEC;
                    archive(
                            cereal::make_nvp("ACCE", ACCE),
                            cereal::make_nvp("GYRO", GYRO),
                            cereal::make_nvp("Q_AtoG", Q_AtoG_VEC)
                    );
                    SO3_AtoG = Sophus::SO3d(VectorToEigenQuaternion(Q_AtoG_VEC));
                }
            };

            // [ topic, param pack ]
            std::map<std::string, IMUParamPack> IMU;

            struct CameraParamPack {
                // current version just supports the pinhole camera
                Eigen::Vector2d FOCAL_LENGTH;
                Eigen::Vector2d FOCAL;
                // pinhole: [k1, k2, k3, t1, t2]
                aligned_vector<double> DISTO_PARAMS;

                double &Fx() { return FOCAL_LENGTH(0); }

                double &Fy() { return FOCAL_LENGTH(1); }

                double &Cx() { return FOCAL(0); }

                double &Cy() { return FOCAL(1); }

                [[nodiscard]] double Fx() const { return FOCAL_LENGTH(0); }

                [[nodiscard]] double Fy() const { return FOCAL_LENGTH(1); }

                [[nodiscard]] double Cx() const { return FOCAL(0); }

                [[nodiscard]] double Cy() const { return FOCAL(1); }

                [[nodiscard]] double Fxy() const { return (Fx() + Fy()) * 0.5; }

                // the intri matrix
                [[nodiscard]] Eigen::Matrix3d KMat() const {
                    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
                    K(0, 0) = Fx();
                    K(1, 1) = Fy();
                    K(0, 2) = Cx();
                    K(1, 2) = Cy();
                    return K;
                }

                // Serialization
                template<class Archive>
                void save(Archive &archive) const {
                    archive(
                            cereal::make_nvp("K", EigenMatToVector(KMat())),
                            cereal::make_nvp("DISTO_PARAMS", DISTO_PARAMS)
                    );
                }


                template<class Archive>
                void load(Archive &archive) {
                    std::vector<std::vector<double>> KMat_VEC;
                    archive(
                            cereal::make_nvp("K", KMat_VEC),
                            cereal::make_nvp("DISTO_PARAMS", DISTO_PARAMS)
                    );
                    auto KMat = VectorToEigenMat<double, 3, 3>(KMat_VEC);
                    FOCAL_LENGTH = Eigen::Vector2d(KMat(0, 0), KMat(1, 1));
                    FOCAL = Eigen::Vector2d(KMat(0, 2), KMat(1, 2));
                }

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

            // [ topic, param pack ]
            std::map<std::string, CameraParamPack> CAMERA;

            // Serialization
            template<class Archive>
            void save(Archive &archive) const {
                archive(
                        cereal::make_nvp("IMU", IMU),
                        cereal::make_nvp("CAMERA", CAMERA)
                );
            }

            template<class Archive>
            void load(Archive &archive) {
                archive(
                        cereal::make_nvp("IMU", IMU),
                        cereal::make_nvp("CAMERA", CAMERA)
                );
            }

            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;
                // IMU
                for (const auto &[topic, pack]: IMU) {
                    infoMap.insert({pack.ACCE.MAP_COEFF.data(), "IMU.ACCE.MAP_COEFF[" + topic + "]"});
                    infoMap.insert({pack.ACCE.BIAS.data(), "IMU.ACCE.BIAS[" + topic + "]"});
                    infoMap.insert({pack.GYRO.MAP_COEFF.data(), "IMU.GYRO.MAP_COEFF[" + topic + "]"});
                    infoMap.insert({pack.GYRO.BIAS.data(), "IMU.GYRO.BIAS[" + topic + "]"});
                    infoMap.insert({pack.SO3_AtoG.data(), "IMU.SO3_AtoG[" + topic + "]"});
                }

                // CAMERA
                for (const auto &[topic, pack]: CAMERA) {
                    infoMap.insert({pack.FOCAL_LENGTH.data(), "CAMERA.FOCAL_LENGTH[" + topic + "]"});
                    infoMap.insert({pack.FOCAL.data(), "CAMERA.FOCAL[" + topic + "]"});
                    infoMap.insert({pack.DISTO_PARAMS.data(), "CAMERA.DISTO_PARAMS[" + topic + "]"});
                }
                return infoMap;
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } INTRI{};

    public:

        // the constructor
        CalibParamManager();

        // the creator
        static CalibParamManager::Ptr Create();

        // save the parameters to file using cereal library(json file format)
        void Save(const std::string &filename) const;

        // load the parameters from file using cereal library(json file format)
        static CalibParamManager::Ptr Load(const std::string &filename);

        // print the parameters in the console
        void ShowParamStatus();

        void VisualizationSensors(ns_viewer::SceneViewer &viewer) const;

        // Serialization, make sure this function be public
        template<class Archive>
        void save(Archive &archive) const {
            archive(
                    cereal::make_nvp("EXTRI", EXTRI),
                    cereal::make_nvp("TEMPORAL", TEMPORAL),
                    cereal::make_nvp("INTRI", INTRI)
            );
        }


        template<class Archive>
        void load(Archive &archive) {
            archive(
                    cereal::make_nvp("EXTRI", EXTRI),
                    cereal::make_nvp("TEMPORAL", TEMPORAL),
                    cereal::make_nvp("INTRI", INTRI)
            );
        }


    protected:

        // set the params to the init values, the intrinsic coeff of camera will load from the config file
        // make sure load and check config before initialize the parameters
        void InitializeParameters();

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}


#endif //LIC_CALIB_CALIB_PARAM_MANAGER_H
