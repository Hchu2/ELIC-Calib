//
// Created by csl on 10/1/22.
//

#include "config/calib_config.h"
#include "filesystem"
#include "thread"
#include "util/enum_cast.hpp"
#include "util/status.hpp"
#include "util/utils.hpp"
#include "yaml-cpp/yaml.h"

namespace ns_elic {

    double CalibConfig::LiDAROdometer::NDTResolution = {};
    double CalibConfig::LiDAROdometer::NDTKeyFrameDownSample = {};
    int CalibConfig::LiDAROdometer::ThreadNum = {};
    int CalibConfig::LiDAROdometer::UpdateMapUntil = {};

    bool CalibConfig::Optimization::UseCuda = {};
    bool CalibConfig::Optimization::LockTimeOffset = {};
    bool CalibConfig::Optimization::LockIMUIntrinsic = {};
    double CalibConfig::Optimization::OptWeight::GyroWeight = {};
    double CalibConfig::Optimization::OptWeight::AcceWeight = {};
    double CalibConfig::Optimization::OptWeight::LidarWeight = {};
    double CalibConfig::Optimization::OptWeight::SO3Weight = {};
    double CalibConfig::Optimization::OptWeight::POSWeight = {};
    bool CalibConfig::Optimization::ProgressToStdout = {};
    int CalibConfig::Optimization::ThreadNum = {};
    int CalibConfig::Optimization::CeresIterations = {};
    double CalibConfig::Optimization::TimeOffsetPadding = {};
    double CalibConfig::Optimization::TimeReadoutPadding = {};
    int CalibConfig::Optimization::RefineIterations = {};

    double CalibConfig::BSpline::KnotTimeDistance = {};

    double CalibConfig::DataAssociate::AssociateRadius = {};
    double CalibConfig::DataAssociate::PlaneLambdaThd = {};

    std::set<std::string> CalibConfig::CalibData::Topic::IMUTopics = {};
    std::vector<std::string> CalibConfig::CalibData::Topic::LiDARTopics = {};
    std::string CalibConfig::CalibData::BagPath = {};
    std::string CalibConfig::CalibData::ParamSavePath = {};
    double CalibConfig::CalibData::BeginTime = {};
    double CalibConfig::CalibData::Duration = {};
    const std::set<std::string> CalibConfig::CalibData::LiDARSupportTypes = {
            "VLP_16_PACKET", "VLP_16_SIMU", "VLP_16_POINTS", "VLP_32E_POINTS", "OUSTER_16_POINTS",
            "LIVOX_HORIZON", "LIVOX_MID40", "LIVOX_MID70", "LIVOX_MID70_SIMU",
            "OUSTER_32_POINTS", "OUSTER_64_POINTS", "OUSTER_128_POINTS"
    };
    std::map<std::string, std::string> CalibConfig::CalibData::LiDARModels = {};

    std::string CalibConfig::CalibData::OutputData::OutputDataDir = {};
    bool CalibConfig::CalibData::OutputData::OutputIMUFrame = {};
    bool CalibConfig::CalibData::OutputData::OutputLiDARFrame = {};
    bool CalibConfig::CalibData::OutputData::OutputLMEquationGraph = {};
    bool CalibConfig::CalibData::OutputData::OutputParamInEachIter = {};
    std::string CalibConfig::CalibData::OutputData::OutputIMUFrameDir = {};
    std::string CalibConfig::CalibData::OutputData::OutputLiDARFrameDir = {};
    std::map<std::string, std::string> CalibConfig::CalibData::OutputData::OutputIMUFrameDirs = {};
    std::map<std::string, std::string> CalibConfig::CalibData::OutputData::OutputLiDARFrameDirs = {};

    SolveModeConfig::Type CalibConfig::SolveMode = {};

    bool CalibConfig::_loadFinished = false;
    bool CalibConfig::_checkFinished = false;

    bool CalibConfig::LoadConfigure(const std::string &filename) {
        LOG_INFO("ready to load calibration configure from file '", filename, "'...")

        if (!std::filesystem::exists(filename)) {
            throw Status(Status::Flag::FETAL, "can't load configure from file: " + filename);
        }

        // load yaml file and analysis it
        auto doc = YAML::LoadFile(filename);
        try {

            SolveMode = EnumCast::stringToEnum<SolveModeConfig::Type>(doc["SolveMode"].as<std::string>());

            // calib data
            auto CalibData = doc["CalibData"];
            auto OutputData = CalibData["OutputData"];
            // imu topics
            if (IMUIntegrated()) {
                auto IMUs = CalibData["IMUs"];
                for (auto &&IMU: IMUs) {
                    CalibData::Topic::IMUTopics.insert(IMU["Topic"].as<std::string>());
                }
            }
            // lidar topic
            if (LiDARIntegrated()) {
                auto LiDARs = CalibData["LiDARs"];
                for (auto &&LiDAR: LiDARs) {
                    auto topic = LiDAR["Topic"].as<std::string>();
                    auto type = LiDAR["Type"].as<std::string>();
                    CalibData::Topic::LiDARTopics.emplace_back(topic);
                    CalibData::LiDARModels.insert({topic, type});
                }
            }

            CalibData::BagPath = CalibData["BagPath"].as<std::string>();
            CalibData::ParamSavePath = CalibData["ParamSavePath"].as<std::string>();

            CalibData::BeginTime = CalibData["BeginTime"].as<double>();
            CalibData::Duration = CalibData["Duration"].as<double>();

            CalibData::OutputData::OutputDataDir = OutputData["OutputDataDir"].as<std::string>();
            CalibData::OutputData::OutputIMUFrame = OutputData["OutputIMUFrame"].as<bool>();
            CalibData::OutputData::OutputLiDARFrame = OutputData["OutputLiDARFrame"].as<bool>();
            CalibData::OutputData::OutputLMEquationGraph = OutputData["OutputLMEquationGraph"].as<bool>();
            CalibData::OutputData::OutputParamInEachIter = OutputData["OutputParamInEachIter"].as<bool>();

            // lidar odometer
            auto LiDAROdometer = doc["LiDAROdometer"];
            LiDAROdometer::NDTResolution = LiDAROdometer["NDTResolution"].as<double>();
            LiDAROdometer::NDTKeyFrameDownSample = LiDAROdometer["NDTKeyFrameDownSample"].as<double>();
            LiDAROdometer::ThreadNum = LiDAROdometer["ThreadNum"].as<int>();
            LiDAROdometer::UpdateMapUntil = LiDAROdometer["UpdateMapUntil"].as<int>();

            // optimization options
            auto Optimization = doc["Optimization"];
            Optimization::UseCuda = Optimization["UseCuda"].as<bool>();
            Optimization::LockTimeOffset = Optimization["LockTimeOffset"].as<bool>();
            Optimization::LockIMUIntrinsic = Optimization["LockIMUIntrinsic"].as<bool>();


            auto OptWeight = Optimization["OptWeight"];
            Optimization::OptWeight::GyroWeight = OptWeight["GyroWeight"].as<double>();
            Optimization::OptWeight::AcceWeight = OptWeight["AcceWeight"].as<double>();
            Optimization::OptWeight::LidarWeight = OptWeight["LidarWeight"].as<double>();
            Optimization::OptWeight::SO3Weight = OptWeight["SO3Weight"].as<double>();
            Optimization::OptWeight::POSWeight = OptWeight["POSWeight"].as<double>();

            Optimization::ThreadNum = Optimization["ThreadNum"].as<int>();
            Optimization::CeresIterations = Optimization["CeresIterations"].as<int>();
            Optimization::TimeOffsetPadding = Optimization["TimeOffsetPadding"].as<double>();
            Optimization::TimeReadoutPadding = Optimization["TimeReadoutPadding"].as<double>();
            Optimization::ProgressToStdout = Optimization["ProgressToStdout"].as<bool>();
            Optimization::RefineIterations = Optimization["RefineIterations"].as<int>();

            // imu b-spline
            auto BSpline = doc["BSpline"];
            BSpline::KnotTimeDistance = BSpline["KnotTimeDistance"].as<double>();

            // data associate
            auto DataAssociate = doc["DataAssociate"];
            DataAssociate::AssociateRadius = DataAssociate["AssociateRadius"].as<double>();
            DataAssociate::PlaneLambdaThd = DataAssociate["PlaneLambdaThd"].as<double>();

            _loadFinished = true;

        } catch (const std::exception &e) {
            throw Status(Status::Flag::FETAL, std::string("loading the config file failed, message: ") + e.what());
        }
        CheckConfigure();
        _checkFinished = true;

        LOG_PLAINTEXT("loading calibration configure finished.")
        LOG_ENDL()
        // print the main fields for the configure information
        PrintMainFields();
        LOG_ENDL()
        return true;
    }

    void CalibConfig::CheckConfigureStatus() {
        if (!_loadFinished) {
            throw Status(Status::Flag::FETAL, "please load the 'CalibConfig' first.");
        }
        if (!_checkFinished) {
            throw Status(Status::Flag::FETAL, "please check the 'CalibConfig' first.");
        }
    }

    void CalibConfig::CheckConfigure() {
        LOG_PLAINTEXT("start checking the configure information...")

        unsigned int hardwareConcurrency = std::thread::hardware_concurrency();

        // SolveMode
        if (SolveMode != SolveModeConfig::Type::MULTI_LiDAR_IMU) {
            throw Status(
                    Status::Flag::FETAL,
                    "invalid solve mode, this field should be "
                    "'MULTI_LiDAR_IMU'."
            );
        }

        // CalibData
        if (IMUIntegrated() && CalibData::Topic::IMUTopics.empty()) {
            // imu data is required
            throw Status(Status::Flag::WARNING, "the param 'CalibData::Topic::IMUTopics' should not be empty.");
        }
        if (LiDARIntegrated() && CalibData::Topic::LiDARTopics.empty()) {
            // lidar data is optional
            throw Status(Status::Flag::WARNING, "the param 'CalibData::Topic::LiDARTopics' should not be empty.");
        }
        if (!std::filesystem::exists(CalibData::BagPath)) {
            throw Status(Status::Flag::WARNING, "the param 'CalibData::BagPath' not exists.");
        }

        for (const auto &[topic, type]: CalibData::LiDARModels) {
            if (CalibData::LiDARSupportTypes.find(type) == CalibData::LiDARSupportTypes.end()) {
                throw Status(Status::Flag::ERROR, "unknown LiDAR type: '" + type + "'.");
            }
        }

        CalibData::OutputData::OutputIMUFrameDir = CalibData::OutputData::OutputDataDir + "/imus";
        CalibData::OutputData::OutputLiDARFrameDir = CalibData::OutputData::OutputDataDir + "/lidars";

        // check the other directories
        for (const auto &topic: CalibData::Topic::IMUTopics) {
            CalibData::OutputData::OutputIMUFrameDirs[topic] =
                    CalibData::OutputData::OutputIMUFrameDir + "/" + topic;
            if (CalibData::OutputData::OutputIMUFrame &&
                !std::filesystem::exists(CalibData::OutputData::OutputIMUFrameDirs[topic])) {
                std::filesystem::create_directories(CalibData::OutputData::OutputIMUFrameDirs[topic]);
            }
        }
        for (const auto &topic: CalibData::Topic::LiDARTopics) {
            CalibData::OutputData::OutputLiDARFrameDirs[topic] =
                    CalibData::OutputData::OutputLiDARFrameDir + "/" + topic;
            if (CalibData::OutputData::OutputLiDARFrame &&
                !std::filesystem::exists(CalibData::OutputData::OutputLiDARFrameDirs[topic])) {
                std::filesystem::create_directories(CalibData::OutputData::OutputLiDARFrameDirs[topic]);
            }
        }

        // LiDAROdometer
        if (LiDAROdometer::NDTResolution <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'LiDAROdometer::NDTResolution' should be positive.");
        }
        if (LiDAROdometer::NDTKeyFrameDownSample <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'LiDAROdometer::NDTKeyFrameDownSample' should be positive.");
        }
        if (LiDAROdometer::ThreadNum <= 0 || LiDAROdometer::ThreadNum > hardwareConcurrency) {
            LOG_WARNING("'LiDAROdometer::ThreadNum' is set from '", LiDAROdometer::ThreadNum, "' to '",
                        hardwareConcurrency, "'.")
            LiDAROdometer::ThreadNum = static_cast<int>(hardwareConcurrency);
        }
        if (LiDAROdometer::UpdateMapUntil <= 0) {
            throw Status(Status::Flag::WARNING, "the param 'LiDAROdometer::UpdateMapUntil' should be positive.");
        }

        // Optimization
        if (Optimization::OptWeight::GyroWeight <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::OptWeight::GyroWeight' should be positive.");
        }
        if (Optimization::OptWeight::AcceWeight <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::OptWeight::AcceWeight' should be positive.");
        }
        if (Optimization::OptWeight::LidarWeight <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::OptWeight::LidarWeight' should be positive.");
        }
        if (Optimization::OptWeight::SO3Weight <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::OptWeight::SO3Weight' should be positive.");
        }
        if (Optimization::OptWeight::POSWeight <= 0.0) {
            throw Status(Status::Flag::WARNING,
                         "the param 'Optimization::OptWeight::POSWeight' should be positive.");
        }
        if (Optimization::TimeOffsetPadding <= 0.0) {
            throw Status(Status::Flag::WARNING,
                         "the param 'Optimization::TimeOffsetPadding' should be positive.");
        }
        if (Optimization::TimeReadoutPadding <= 0.0) {
            throw Status(Status::Flag::WARNING,
                         "the param 'Optimization::TimeReadoutPadding' should be positive.");
        }
        if (Optimization::CeresIterations <= 0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::CeresIterations' should be positive");
        }
        if (Optimization::RefineIterations <= 0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::RefineIterations' should be positive");
        }
        if (Optimization::ThreadNum <= 0 || Optimization::ThreadNum > hardwareConcurrency) {
            LOG_WARNING("'Optimization::ThreadNum' is set from '", Optimization::ThreadNum, "' to '",
                        hardwareConcurrency, "'.")
            Optimization::ThreadNum = static_cast<int>(hardwareConcurrency);
        }

        // BSpline
        if (BSpline::KnotTimeDistance <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'BSpline::KnotTimeDistance' should be positive.");
        }

        // DataAssociate
        if (DataAssociate::AssociateRadius <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'DataAssociate::AssociateRadius' should be positive.");
        }
        if (DataAssociate::PlaneLambdaThd <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'DataAssociate::PlaneLambdaThd' should be positive.");
        }

        LOG_PLAINTEXT("checking the configure information finished, everything is ok!")
    }

    void CalibConfig::PrintMainFields() {
        LOG_PLAINTEXT("Main Fields for Config:")
        std::cout << std::boolalpha;
        // SolveMode
        LOG_PLAINTEXT("SolveMode: ", SolveMode)
        LOG_PLAINTEXT(" LiDAR Integrated: ", LiDARIntegrated())
        LOG_PLAINTEXT("   IMU Integrated: ", IMUIntegrated())
        LOG_ENDL()
        // CalibData::Topic
        LOG_VAR(CalibData::Topic::IMUTopics)
        LOG_VAR(CalibData::Topic::LiDARTopics)
        LOG_VAR(CalibData::LiDARModels)

        LOG_ENDL()
        LOG_VAR(CalibData::OutputData::OutputLiDARFrame)
        LOG_VAR(CalibData::OutputData::OutputIMUFrame)
        LOG_VAR(CalibData::OutputData::OutputLMEquationGraph)
        LOG_VAR(CalibData::OutputData::OutputParamInEachIter)
        LOG_ENDL()
        // CalibData
        LOG_VAR(CalibData::BagPath)
        LOG_VAR(CalibData::BeginTime)
        LOG_VAR(CalibData::Duration)
        LOG_ENDL()
        // Optimization
        LOG_VAR(Optimization::UseCuda)
        LOG_VAR(Optimization::LockTimeOffset)
        LOG_VAR(Optimization::LockIMUIntrinsic)
        LOG_VAR(Optimization::RefineIterations)
        LOG_ENDL()
        // BSpline
        LOG_VAR(BSpline::KnotTimeDistance)
        LOG_VAR(BSpline::SplineOrder)
        LOG_ENDL()
        // DataAssociate
        LOG_VAR(DataAssociate::AssociateRadius)
        LOG_VAR(DataAssociate::PlaneLambdaThd)
        LOG_ENDL()
    }

    bool CalibConfig::IMUIntegrated() {
        return SolveModeConfig::IsSolveModeTypeWith(SolveModeConfig::Type::IMU, SolveMode);
    }

    bool CalibConfig::LiDARIntegrated() {
        return SolveModeConfig::IsSolveModeTypeWith(SolveModeConfig::Type::LiDAR, SolveMode);
    }
}