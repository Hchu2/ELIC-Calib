//
// Created by csl on 5/24/23.
//

#include "viewer/surfel_map.h"

namespace ns_elic {

    SurfelMap::SurfelMap(const aligned_vector<SurfelPlane> &surfels) : _surfels(surfels) {}

    SurfelMap::Ptr SurfelMap::Create(const aligned_vector<SurfelPlane> &surfels) {
        return std::make_shared<SurfelMap>(surfels);
    }

    void SurfelMap::ShowSurfelMap(const std::string &saveShotDir) const {
        Viewer viewer(saveShotDir, "surfel map");
        ShowSurfelMap(viewer);
        viewer.RunSingleThread();
    }

    void SurfelMap::Save(const std::string &filename, bool binaryMode) const {
        std::ofstream file(filename);
        if (binaryMode) {
            cereal::BinaryOutputArchive ar(file);
            ar(cereal::make_nvp("surfel_map", *this));
        } else {
            cereal::JSONOutputArchive ar(file);
            ar(cereal::make_nvp("surfel_map", *this));
        }
    }

    SurfelMap::Ptr SurfelMap::Load(const std::string &filename, bool binaryMode) {
        SurfelMap::Ptr surfelMap(new SurfelMap());
        std::ifstream file(filename);
        if (binaryMode) {
            cereal::BinaryInputArchive ar(file);
            ar(cereal::make_nvp("surfel_map", *surfelMap));
        } else {
            cereal::JSONInputArchive ar(file);
            ar(cereal::make_nvp("surfel_map", *surfelMap));
        }
        return surfelMap;
    }

    void SurfelMap::ShowSurfelMap(Viewer &viewer) const {
        viewer.ShowSurfelPlanes(_surfels);
    }
}