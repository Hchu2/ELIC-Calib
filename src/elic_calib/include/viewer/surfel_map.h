//
// Created by csl on 5/24/23.
//

#ifndef ELIC_CALIB_SURFEL_MAP_H
#define ELIC_CALIB_SURFEL_MAP_H

#include "core/surfel_association.h"
#include "viewer/pose_viewer.h"
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/polymorphic.hpp"
#include "util/utils.hpp"

namespace ns_elic {
    struct SurfelMap {
    public:
        using Ptr = std::shared_ptr<SurfelMap>;

    protected:
        aligned_vector<SurfelPlane> _surfels;

    public:
        explicit SurfelMap(const aligned_vector<SurfelPlane> &surfels);

        SurfelMap() = default;

        static Ptr Create(const aligned_vector<SurfelPlane> &surfels);

        void ShowSurfelMap(const std::string &saveShotDir = "") const;

        void ShowSurfelMap(Viewer &viewer) const;

        void Save(const std::string &filename, bool binaryMode = true) const;

        static Ptr Load(const std::string &filename, bool binaryMode = true);

    public:
        template<class Archive>
        void serialize(Archive &archive) {
            archive(cereal::make_nvp("surfels", _surfels));
        }
    };
}

#endif //ELIC_CALIB_SURFEL_MAP_H
