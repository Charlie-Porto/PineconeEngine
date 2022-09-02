#ifndef panel_forging_hpp
#define panel_forging_hpp

#include <vector>

#include "sheet_forging.hpp"

extern ControlPanel control;

namespace pce3d {
namespace forge {

std::vector<uint32_t> forgeFloorPanelByDimensionsAndCenterPoint(
    const double tile_w
  , const double tile_l
  , const double panel_w
  , const double panel_l
  , const glm::dvec3& center_point
  , const std::vector<int>& color
  , const bool is_transparent = false
);


}
}




#endif /* panel_forging_hpp */
