//
// Created by csl on 11/1/22.
//

#ifndef LIC_CALIB_SFM_ASSOCIATION_H
#define LIC_CALIB_SFM_ASSOCIATION_H

#include <utility>

#include "calib/calib_data_manager.h"
#include "ostream"
#include "utility"

namespace ns_elic {
    struct ReprojectCorrespondence {
    public:
        // trans from pi to pj
        double piImgTimestamp{};
        Eigen::Vector2d pi;
        // tri = vi/h - 0.5
        double tri{};
        // to get thr inverse depth of pi
        std::uint32_t trackId{};

        double pjImgTimestamp{};
        Eigen::Vector2d pj;
        // trj = vj/h - 0.5
        double trj{};

        double confidence;

        ReprojectCorrespondence(double piImgTimestamp, Eigen::Vector2d pi, double tri, uint32_t trackId,
                                double pjImgTimestamp, Eigen::Vector2d pj, double trj, double confidence)
                : piImgTimestamp(piImgTimestamp), pi(std::move(pi)), tri(tri), trackId(trackId),
                  pjImgTimestamp(pjImgTimestamp), pj(std::move(pj)), trj(trj), confidence(confidence) {}

        ReprojectCorrespondence() = default;

        friend ostream &operator<<(ostream &os, const ReprojectCorrespondence &correspondence) {
            os << "piImgTimestamp: " << correspondence.piImgTimestamp << " pi: " << correspondence.pi << " tri: "
               << correspondence.tri << " trackId: " << correspondence.trackId << " pjImgTimestamp: "
               << correspondence.pjImgTimestamp << " pj: " << correspondence.pj << " trj: " << correspondence.trj
               << " confidence: " << correspondence.confidence;
            return os;
        }
    };

    class ReprojectAssociation {
    public:
        using Ptr = std::shared_ptr<ReprojectAssociation>;

        enum Option : std::uint32_t {
            /**
             * @brief options
             */
            NONE = 1 << 0,
            CAM_CAM = 1 << 1,
            CAM_IMG = 1 << 2,
            IMG_IMG = 1 << 3,
            IMG_CAM = 1 << 4,
            ALL = CAM_CAM | CAM_IMG | IMG_IMG | IMG_CAM
        };

        static bool IsOptionWith(std::uint32_t desired, std::uint32_t curOption) {
            return (desired == (desired & curOption));
        }

        /**
         * @brief override operator '<<' for type 'Option'
         */
        friend std::ostream &operator<<(std::ostream &os, const Option &curOption) {
            std::stringstream stream;
            int count = 0;
            if (IsOptionWith(CAM_CAM, curOption)) {
                stream << "CAM_CAM";
                ++count;
            }
            if (IsOptionWith(CAM_IMG, curOption)) {
                stream << " | CAM_IMG";
                ++count;
            }
            if (IsOptionWith(IMG_IMG, curOption)) {
                stream << " | IMG_IMG";
                ++count;
            }
            if (IsOptionWith(IMG_CAM, curOption)) {
                stream << " | IMG_CAM";
                ++count;
            }
            if (count == 0) {
                os << "NONE";
            } else if (count == 4) {
                os << "ALL";
            } else {
                std::string str = stream.str();
                if (str.at(1) == '|') {
                    str = str.substr(3, str.size() - 3);
                }
                os << str;
            }
            return os;
        };

    private:
        aligned_vector<ReprojectCorrespondence> _corrs;

    public:
        ReprojectAssociation() = default;

        static ReprojectAssociation::Ptr Create();

    };
}

#endif //LIC_CALIB_SFM_ASSOCIATION_H
