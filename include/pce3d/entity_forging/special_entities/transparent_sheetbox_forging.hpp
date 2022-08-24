#ifndef transparent_sheetbox_forging_hpp
#define transparent_sheetbox_forging_hpp

extern ControlPanel control;

namespace pce3d {
namespace forge {

std::vector<uint32_t> forgeSheetBoxEntity(
    const double h
  , const double w
  , const double l
  , const glm::dvec3& center_point
  , const std::vector<int>& color
);

}
}


#endif /* transparent_sheetbox_forging_hpp */
