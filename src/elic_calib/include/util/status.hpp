//
// Created by csl on 9/22/22.
//

#ifndef ALG_SIM_STATUS_HPP
#define ALG_SIM_STATUS_HPP

#include "exception"
#include "string"
#include "util/enum_cast.hpp"

namespace ns_elic {
    struct Status : std::exception {
        enum class Flag {
            FINE, WARNING, ERROR, FETAL
        };
    public:
        Flag flag;
        std::string what;

        Status(Flag flag, std::string what) : flag(flag), what(std::move(what)) {}

        Status() : flag(Flag::FINE), what() {}

        friend std::ostream &operator<<(std::ostream &os, const Status &status) {
            os << "[" << EnumCast::enumToString(status.flag) << "]-[" << status.what << "]";
            return os;
        }
    };

}

#endif //ALG_SIM_STATUS_HPP
