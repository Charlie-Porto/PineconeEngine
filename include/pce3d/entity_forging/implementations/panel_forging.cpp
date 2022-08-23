#ifndef panel_forging_cpp
#define panel_forging_cpp

#include "../panel_forging.hpp"


namespace pce3d {
namespace forge {

std::vector<uint32_t> forgeFloorPanelByDimensionsAndCenterPoint(
    const double tile_w
  , const double tile_l
  , const double panel_w
  , const double panel_l
  , const glm::dvec3& center_point
  , const std::vector<int>& color
)
{
  const glm::dvec3 sheet_creation_start_point = glm::dvec3(
    center_point.x - panel_w/2.0,
    center_point.y,
    center_point.z - panel_l/2.0);

  std::vector<uint32_t> entities{};

  double current_x_coord = sheet_creation_start_point.x;  
  for (double x_crawl = 0; x_crawl <= panel_w; x_crawl += tile_w)
  {
    double current_z_coord = sheet_creation_start_point.z;
    for (double z_crawl = 0; z_crawl <= panel_l; z_crawl += tile_l)
    {
      std::cout << "creating panel" << z_crawl <<'\n';
      const uint32_t new_sheet_entity = forgeRectSheetEntity(
        tile_w,
        tile_l,
        glm::dvec3(
          (current_x_coord + x_crawl) + tile_w/2.0,
          center_point.y,
          (current_z_coord + z_crawl) + tile_l/2.0),
        0.0,
        glm::dvec3(1.0, 0, 0),
        color
      );
      entities.push_back(new_sheet_entity);
    }
  }
  
  return entities;

}


}
}


#endif /* panel_forging_cpp */
