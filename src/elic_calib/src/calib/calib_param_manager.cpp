//
// Created by csl on 10/1/22.
//

#include "calib/calib_param_manager.h"
#include "config/calib_config.h"
#include "filesystem"
#include "fmt/color.h"
#include "functors/functor_typedef.hpp"

namespace ns_elic {

    CalibParamManager::CalibParamManager() {
        this->InitializeParameters();
    }

    CalibParamManager::Ptr CalibParamManager::Create() {
        return std::make_shared<CalibParamManager>();
    }

    void CalibParamManager::InitializeParameters() {
        LOG_INFO("initialize the calibration parameters...")
        CalibConfig::CheckConfigureStatus();

        // extrinsic
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            // allocate memory
            EXTRI.SO3_IjToIr[topic] = Sophus::SO3d();
            EXTRI.POS_IjInIr[topic] = Eigen::Vector3d::Zero();
        }

        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            // allocate memory
            EXTRI.SO3_LmToLr[topic] = Sophus::SO3d();
            EXTRI.POS_LmInLr[topic] = Eigen::Vector3d::Zero();
        }

        EXTRI.POS_LrInIr = Eigen::Vector3d::Zero();
        EXTRI.SO3_LrToIr = Sophus::SO3d();

        // align to the 'z' axis
        EXTRI.GRAVITY = Eigen::Vector3d(0.0, 0.0, -CalibConfig::BSpline::GRefineNorm);

        // temporal
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            // allocate memory
            TEMPORAL.TIME_OFFSET_IjToIr[topic] = 0.0;
        }

        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            TEMPORAL.TIME_OFFSET_LmToLr[topic] = 0.0;
        }

        TEMPORAL.TIME_OFFSET_LrToIr = 0.0;

        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            // initialize intrinsic
            auto &IMU = INTRI.IMU[topic];

            IMU.ACCE.BIAS = Eigen::Vector3d::Zero();
            IMU.ACCE.MAP_COEFF = Eigen::Vector6d::Zero();
            IMU.ACCE.MAP_COEFF(0) = 1.0;
            IMU.ACCE.MAP_COEFF(1) = 1.0;
            IMU.ACCE.MAP_COEFF(2) = 1.0;

            IMU.GYRO.BIAS = Eigen::Vector3d::Zero();
            IMU.GYRO.MAP_COEFF = Eigen::Vector6d::Zero();
            IMU.GYRO.MAP_COEFF(0) = 1.0;
            IMU.GYRO.MAP_COEFF(1) = 1.0;
            IMU.GYRO.MAP_COEFF(2) = 1.0;

            IMU.SO3_AtoG = Sophus::SO3d();
        }

        LOG_PLAINTEXT("initialize the parameters finished.")
        LOG_ENDL()
    }

    void CalibParamManager::Save(const std::string &filename) const {
        std::ofstream file(filename, std::ios::out);
        cereal::JSONOutputArchive ar(file);

        ar(cereal::make_nvp("CalibParam", *this));
    }

    CalibParamManager::Ptr CalibParamManager::Load(const std::string &filename) {
        auto calibParamManager = CalibParamManager::Create();
        std::ifstream file(filename, std::ios::in);
        cereal::JSONInputArchive ar(file);

        ar(cereal::make_nvp("CalibParam", *calibParamManager));
        return calibParamManager;
    }

    void CalibParamManager::ShowParamStatus() {

#define LOG_STYLE_NONE std::string("\033[0m")
#define LOG_STYLE_BOLD std::string("\033[1m")
#define LOG_STYLE_GREEN std::string("\033[92m")

#define ITEM(name) LOG_STYLE_BOLD, LOG_STYLE_GREEN, name, LOG_STYLE_NONE
#define PARAM(name) LOG_STYLE_BOLD, name, LOG_STYLE_NONE

        constexpr std::size_t n = 87;

        LOG_PLAINTEXT(std::string(25, '-'))
        LOG_PLAINTEXT(ITEM("calibration parameters"), " --")
        LOG_PLAINTEXT(std::string(n, '-'))

        Sophus::SE3d SE3_ToRef;

        if (CalibConfig::IMUIntegrated()) {
            SE3_ToRef = Sophus::SE3d(
                    EXTRI.SO3_IjToIr.at(*CalibConfig::CalibData::Topic::IMUTopics.cbegin()),
                    EXTRI.POS_IjInIr.at(*CalibConfig::CalibData::Topic::IMUTopics.cbegin())
            ).inverse();
        } else {
            // for lidars suite
            SE3_ToRef = Sophus::SE3d(
                    EXTRI.SO3_LmToLr.at(*CalibConfig::CalibData::Topic::LiDARTopics.cbegin()),
                    EXTRI.POS_LmInLr.at(*CalibConfig::CalibData::Topic::LiDARTopics.cbegin())
            ).inverse();
        }

        // -------------------------
        if (CalibConfig::IMUIntegrated()) {
            LOG_PLAINTEXT(ITEM("EXTRI(ALIGNED TO IMU: " + *CalibConfig::CalibData::Topic::IMUTopics.cbegin() + ")"))
        } else {
            LOG_PLAINTEXT(ITEM("EXTRI(ALIGNED TO LiDAR: " + *CalibConfig::CalibData::Topic::LiDARTopics.cbegin() + ")"))
        }
        // -------------------------
        LOG_PLAINTEXT("")

        // imu
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            LOG_PLAINTEXT("IMU: ", topic)

            auto SE3_curToRef = SE3_ToRef * Sophus::SE3d(EXTRI.SO3_IjToIr.at(topic), EXTRI.POS_IjInIr.at(topic));

            const auto EULER_IjToRef = EXTRI.EULER_DEG(SE3_curToRef.so3());
            LOG_PLAINTEXT(PARAM("EUR_IjToRef: "), FormatValueVector<double>(
                    {"Xr", "Yp", "Zy"}, {EULER_IjToRef(0), EULER_IjToRef(1), EULER_IjToRef(2)}))
            const auto POS_IjInRef = SE3_curToRef.translation();
            LOG_PLAINTEXT(PARAM("POS_IjInRef: "), FormatValueVector<double>(
                    {"Px", "Py", "Pz"}, {POS_IjInRef(0), POS_IjInRef(1), POS_IjInRef(2)}))
            LOG_PLAINTEXT("")
        }

        auto SE3_LrToRef = SE3_ToRef * Sophus::SE3d(EXTRI.SO3_LrToIr, EXTRI.POS_LrInIr);

        // lidar
        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            LOG_PLAINTEXT("LiDAR: ", topic)

            auto SE3_curToRef = SE3_LrToRef * Sophus::SE3d(EXTRI.SO3_LmToLr.at(topic), EXTRI.POS_LmInLr.at(topic));

            const auto EULER_LmToRef = EXTRI.EULER_DEG(SE3_curToRef.so3());
            LOG_PLAINTEXT(PARAM("EUR_LmToRef: "), FormatValueVector<double>(
                    {"Xr", "Yp", "Zy"}, {EULER_LmToRef(0), EULER_LmToRef(1), EULER_LmToRef(2)}))
            const auto POS_LmInRef = SE3_curToRef.translation();
            LOG_PLAINTEXT(PARAM("POS_LmInRef: "), FormatValueVector<double>(
                    {"Px", "Py", "Pz"}, {POS_LmInRef(0), POS_LmInRef(1), POS_LmInRef(2)}))
            LOG_PLAINTEXT("")
        }

        if(CalibConfig::CalibData::Topic::LiDARTopics.size() >= 2) {
            auto topic_velo = CalibConfig::CalibData::Topic::LiDARTopics.at(0);
            auto topic_livox = CalibConfig::CalibData::Topic::LiDARTopics.at(1);
            auto SE3_velToRef = SE3_LrToRef * Sophus::SE3d(EXTRI.SO3_LmToLr.at(topic_velo), EXTRI.POS_LmInLr.at(topic_velo));
            auto SE3_livToRef = SE3_LrToRef * Sophus::SE3d(EXTRI.SO3_LmToLr.at(topic_livox), EXTRI.POS_LmInLr.at(topic_livox));
            auto SE_livTovel = SE3_velToRef.inverse() * SE3_livToRef;
            Eigen::Quaterniond tmp = Eigen::Quaterniond(SE_livTovel.so3().matrix());
            LOG_PLAINTEXT(PARAM("Qua_LivoxToVelodyne: "), FormatValueVector<double>(
                    {"Qw", "Qx", "Qy", "Qz"}, {tmp.w(), tmp.x(), tmp.y(), tmp.z()}))
            const auto EULER_LivToVel = EXTRI.EULER_DEG(SE_livTovel.so3());
            LOG_PLAINTEXT(PARAM("EUR_LivoxToVelodyne: "), FormatValueVector<double>(
                    {"Xr", "Yp", "Zy"}, {EULER_LivToVel(0), EULER_LivToVel(1), EULER_LivToVel(2)}))
            LOG_PLAINTEXT(PARAM("POS_LivoxInVelodyne: "), FormatValueVector<double>(
                    {"Px", "Py", "Pz"}, {SE_livTovel.translation()(0), SE_livTovel.translation()(1), SE_livTovel.translation()(2)}))
            LOG_PLAINTEXT("")
        }

        LOG_PLAINTEXT(PARAM("    GRAVITY: "), FormatValueVector<double>(
                {"Gx", "Gy", "Gz"}, {EXTRI.GRAVITY(0), EXTRI.GRAVITY(1), EXTRI.GRAVITY(2)}))
        LOG_PLAINTEXT(std::string(n, '-'))
        // ----------------------------
        if (CalibConfig::IMUIntegrated()) {
            LOG_PLAINTEXT(ITEM("TEMPORAL(ALIGNED TO IMU: " + *CalibConfig::CalibData::Topic::IMUTopics.cbegin() + ")"))
        } else {
            LOG_PLAINTEXT(
                    ITEM("TEMPORAL(ALIGNED TO LiDAR: " + *CalibConfig::CalibData::Topic::LiDARTopics.cbegin() + ")"))
        }
        // ----------------------------
        double TIME_OFFSET_ToRef;
        if (CalibConfig::IMUIntegrated()) {
            TIME_OFFSET_ToRef = -TEMPORAL.TIME_OFFSET_IjToIr.at(*CalibConfig::CalibData::Topic::IMUTopics.cbegin());
        } else {
            TIME_OFFSET_ToRef = -TEMPORAL.TIME_OFFSET_LmToLr.at(*CalibConfig::CalibData::Topic::LiDARTopics.cbegin());
        }
        LOG_PLAINTEXT("")
        // imu
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            LOG_PLAINTEXT("IMU: ", topic)
            LOG_PLAINTEXT(PARAM("TIME_OFFSET_IjToRef: "),
                          fmt::format(fmt::emphasis::italic, "{:+010.6f}",
                                      TEMPORAL.TIME_OFFSET_IjToIr.at(topic) + TIME_OFFSET_ToRef))
            LOG_PLAINTEXT("")
        }

        // lidar
        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            LOG_PLAINTEXT("LiDAR: ", topic)
            LOG_PLAINTEXT(PARAM("TIME_OFFSET_LmToRef:  "),
                          fmt::format(fmt::emphasis::italic, "{:+010.6f}",
                                      TEMPORAL.TIME_OFFSET_LmToLr.at(topic) + TEMPORAL.TIME_OFFSET_LrToIr +
                                      TIME_OFFSET_ToRef))
            LOG_PLAINTEXT("")
        }

        LOG_PLAINTEXT(std::string(n, '-'))
        // -------------------------
        LOG_PLAINTEXT(ITEM("INTRI"))
        // -------------------------
        LOG_PLAINTEXT("")
        for (const auto &[topic, IMU]: INTRI.IMU) {
            LOG_PLAINTEXT("IMU: ", topic)

            LOG_PLAINTEXT(PARAM("ACCE      BIAS: "), FormatValueVector<double>(
                    {"Bx", "By", "Bz"}, {IMU.ACCE.BIAS(0), IMU.ACCE.BIAS(1), IMU.ACCE.BIAS(2)}))
            LOG_PLAINTEXT(PARAM("ACCE_MAP_COEFF: "), FormatValueVector<double>(
                    {"00", "11", "22"},
                    {IMU.ACCE.MAP_COEFF(0), IMU.ACCE.MAP_COEFF(1), IMU.ACCE.MAP_COEFF(2)}))
            LOG_PLAINTEXT(PARAM("                "), FormatValueVector<double>(
                    {"01", "02", "12"},
                    {IMU.ACCE.MAP_COEFF(3), IMU.ACCE.MAP_COEFF(4), IMU.ACCE.MAP_COEFF(5)}))
            LOG_PLAINTEXT("")

            LOG_PLAINTEXT(PARAM("GYRO      BIAS: "), FormatValueVector<double>(
                    {"Bx", "By", "Bz"}, {IMU.GYRO.BIAS(0), IMU.GYRO.BIAS(1), IMU.GYRO.BIAS(2)}))
            LOG_PLAINTEXT(PARAM("GYRO_MAP_COEFF: "), FormatValueVector<double>(
                    {"00", "11", "22"},
                    {IMU.GYRO.MAP_COEFF(0), IMU.GYRO.MAP_COEFF(1), IMU.GYRO.MAP_COEFF(2)}))
            LOG_PLAINTEXT(PARAM("                "), FormatValueVector<double>(
                    {"01", "02", "12"},
                    {IMU.GYRO.MAP_COEFF(3), IMU.GYRO.MAP_COEFF(4), IMU.GYRO.MAP_COEFF(5)}))
            LOG_PLAINTEXT("")

            const auto Q_AtoG = IMU.Q_AtoG();
            LOG_PLAINTEXT(PARAM("Qua_AtoG: "), FormatValueVector<double>(
                    {"Qx", "Qy", "Qz", "Qw"}, {Q_AtoG.x(), Q_AtoG.y(), Q_AtoG.z(), Q_AtoG.w()}));
            LOG_PLAINTEXT("")
        }
        LOG_PLAINTEXT(std::string(n, '-'))

#undef ITEM
#undef PARAM
#undef LOG_STYLE_NONE
#undef LOG_STYLE_BOLD
#undef LOG_STYLE_GREEN

    }

    void CalibParamManager::VisualizationSensors(ns_viewer::SceneViewer &viewer) const {
        auto ItoI = Sophus::SE3d().cast<float>();

        // lidar
        for (const auto &topic: CalibConfig::CalibData::Topic::LiDARTopics) {
            auto SE3_LmToIr = EXTRI.SE3_LmToIr(topic).cast<float>();
            auto SE3_LrToIr = EXTRI.SE3_LrToIr().cast<float>();
            viewer.AddLiDAR(
                    ns_viewer::Posef(SE3_LmToIr.so3().matrix(), SE3_LmToIr.translation()),
                    ns_viewer::Colour::Blue(), 0.03f
            );
            viewer.AddArrow(
                    Eigen::Vector3f{SE3_LmToIr.translation()(0), SE3_LmToIr.translation()(1),
                                    SE3_LmToIr.translation()(2)},
                    Eigen::Vector3f{SE3_LrToIr.translation()(0), SE3_LrToIr.translation()(1),
                                    SE3_LrToIr.translation()(2)},
                    ns_viewer::Colour(0.3f, 0.3f, 0.3f, 0.2f), 0.6f
            );
        }
        // imu
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            auto SO3_IjToI = EXTRI.SO3_IjToIr.at(topic);
            auto POS_IjInI = EXTRI.POS_IjInIr.at(topic);
            viewer.AddIMU(
                    ns_viewer::Posed(SO3_IjToI.matrix(), POS_IjInI).cast<float>(), ns_viewer::Colour::Red(), 0.05f
            );
            viewer.AddArrow(
                    POS_IjInI.cast<float>(),
                    Eigen::Vector3f{ItoI.translation()(0), ItoI.translation()(1), ItoI.translation()(2)},
                    ns_viewer::Colour(0.3f, 0.3f, 0.3f, 0.2f), 0.6f
            );
        }
        if (CalibConfig::IMUIntegrated()) {
            viewer.AddIMU(
                    ns_viewer::Posef(ItoI.so3().matrix(), ItoI.translation()),
                    ns_viewer::Colour::Black().WithAlpha(0.2f), 0.05f
            );
        }

        if (CalibConfig::LiDARIntegrated()) {
            auto SE3_LrToIr = EXTRI.SE3_LrToIr().cast<float>();
            viewer.AddLiDAR(
                    ns_viewer::Posef(SE3_LrToIr.so3().matrix(), SE3_LrToIr.translation()),
                    ns_viewer::Colour::Black().WithAlpha(0.2f), 0.03f
            );
            viewer.AddArrow(
                    Eigen::Vector3f{SE3_LrToIr.translation()(0), SE3_LrToIr.translation()(1),
                                    SE3_LrToIr.translation()(2)},
                    Eigen::Vector3f{ItoI.translation()(0), ItoI.translation()(1), ItoI.translation()(2)},
                    ns_viewer::Colour(0.3f, 0.3f, 0.3f, 0.2f), 0.6f
            );
        }


    }
}
