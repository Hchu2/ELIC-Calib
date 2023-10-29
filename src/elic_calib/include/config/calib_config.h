//
// Created by csl on 10/1/22.
//

#ifndef LIC_CALIB_CALIB_CONFIG_H
#define LIC_CALIB_CALIB_CONFIG_H

#include "fmt/format.h"
#include "sophus/se3.hpp"
#include "string"
#include "thirdparty/logger/src/include/logger.h"
#include "util/macros.hpp"
#include "util/type_define.hpp"

namespace ns_elic {

    using namespace magic_enum::bitwise_operators;

    struct SolveModeConfig {

        enum class Type : std::uint32_t {
            /**
             * @brief options
             */
            IMU = 1 << 1,
            LiDAR = 1 << 2,
            MULTI_LiDAR_IMU = IMU | LiDAR,
        };

        static bool IsSolveModeTypeWith(Type desired, Type curSolveModeType) {
            return (desired == (desired & curSolveModeType));
        }

        /**
         * @brief override operator '<<' for type 'SolveModeType'
         */
        friend std::ostream &operator<<(std::ostream &os, const Type &curSolveModeType) {
            std::stringstream stream;
            if (IsSolveModeTypeWith(Type::MULTI_LiDAR_IMU, curSolveModeType)) {
                stream << "MULTI_LiDAR_IMU";
            } else {
                stream << "unknown solve mode";
            }
            os << stream.str();
            return os;
        }
    };


    struct CalibConfig {
    public:

    public:
        struct LiDAROdometer {
            static double NDTResolution;
            static double NDTKeyFrameDownSample;
            static int ThreadNum;
            static int UpdateMapUntil;
        };

        struct Optimization {
            static bool UseCuda;
            static bool LockTimeOffset;
            static bool LockIMUIntrinsic;
            struct OptWeight {
                static double GyroWeight;
                static double AcceWeight;
                static double LidarWeight;

                static double SO3Weight;
                static double POSWeight;

            };
            static int ThreadNum;
            static int CeresIterations;
            static double TimeOffsetPadding;
            static double TimeReadoutPadding;
            static bool ProgressToStdout;
            static int RefineIterations;
        };

        struct BSpline {
            static double KnotTimeDistance;
            // the order(degree + 1) of the b-spline
            // for simulate
            // static constexpr int SplineOrder = 10;
            // for solving
            static constexpr int SplineOrder = 4;
            static constexpr double GRefineNorm = 9.797;
        };

        struct DataAssociate {
            static double AssociateRadius;
            static double PlaneLambdaThd;
            static std::map<std::string, int> LandmarkCulling;
        };

        struct CalibData {
            struct Topic {
                static std::set<std::string> IMUTopics;
                static std::vector<std::string> LiDARTopics;
            };
            static std::string BagPath;
            static std::string ParamSavePath;

            static double BeginTime;
            static double Duration;

            static const std::set<std::string> LiDARSupportTypes;
            static std::map<std::string, std::string> LiDARModels;

            struct OutputData {
                static std::string OutputDataDir;
                static bool OutputIMUFrame;
                static bool OutputLiDARFrame;

                static bool OutputLMEquationGraph;
                static bool OutputParamInEachIter;

                static std::string OutputIMUFrameDir;
                static std::string OutputLiDARFrameDir;

                // [ topic, directory ]
                static std::map<std::string, std::string> OutputIMUFrameDirs;
                static std::map<std::string, std::string> OutputLiDARFrameDirs;

                // the precision for float data output
                static constexpr int Precision = 12;
            };

        };

        static SolveModeConfig::Type SolveMode;

    private:
        static bool _loadFinished;
        static bool _checkFinished;

    public:
        // load configure information from the yaml file
        static bool LoadConfigure(const std::string &filename);

        // check the config loading status, make sure call it  every time before using CalibConfig info
        static void CheckConfigureStatus();

        static bool IMUIntegrated();

        static bool LiDARIntegrated();

    protected:
        // check the parameters, if there is any invalid parameter, it will throw an exception
        static void CheckConfigure();

        // print the main fields for configure check
        static void PrintMainFields();

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //LIC_CALIB_CALIB_CONFIG_H