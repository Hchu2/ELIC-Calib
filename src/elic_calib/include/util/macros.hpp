//
// Created by csl on 10/5/22.
//

#ifndef LIC_CALIB_MACROS_HPP
#define LIC_CALIB_MACROS_HPP

#include "icecream.hpp"
#include "util/status.hpp"

namespace ns_elic {
    // macros
#define INVALID_TIME_STAMP (-1.0)

#define IS_INVALID_TIME_STAMP(t) ((t) < 0.0)

#define ASSERT_TIME_STAMP(t)                                                                            \
  if (IS_INVALID_TIME_STAMP(t)) {                                                                       \
    icecream::ic.prefix("");                                                                            \
    auto str = std::string{};                                                                           \
    icecream::ic.output(str);                                                                           \
    IC();                                                                                               \
    throw ns_elic::Status(ns_elic::Status::Flag::FETAL, "the time stamp is invalid, check here: " + str); \
  }

#define IS_POINT_NAN(p) (std::isnan((p).x) || std::isnan((p).y) || std::isnan((p).z))
//zero point would appear in LivoxPoint
#define IS_POINT_ZERO(p) ((abs((p).x) < 0.01) && (abs((p).y) < 0.01) && (abs((p).z) < 0.01))
#define SET_POST_POINT_NAN(p)           \
  {                                     \
    (p).timestamp = INVALID_TIME_STAMP; \
    (p).x = (p).y = (p).z = NAN;        \
  }

#define LOAD_STR_ROS_PARAM(ns, paramName)                         \
    std::string paramName = []() {                                \
        std::string paramName;                                    \
        ros::param::get(std::string(ns) + #paramName, paramName); \
        LOG_PLAINTEXT(#paramName, ": ", paramName)                \
        return paramName;                                         \
    }();
}

#endif // LIC_CALIB_MACROS_HPP
