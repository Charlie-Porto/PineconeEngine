#ifndef transparent_sheetbox_forging_cpp
#define transparent_sheetbox_forging_cpp

#include "../transparent_sheetbox_forging.hpp"
#include "../../sheet_forging.hpp"

namespace pce3d {
namespace forge {

std::vector<uint32_t> forgeSheetBoxEntity(
    const double h
  , const double w
  , const double l
  , const glm::dvec3& center_point
  , const std::vector<int>& color
)
{
  auto top_sheet = forgeRectSheetEntity(
    w, l,
    glm::dvec3(center_point.x, center_point.y + h/2.0, center_point.z),
    0.0,
    glm::dvec3(0, 0, 1),
    color,
    true
  );
  auto bottom_sheet = forgeRectSheetEntity(
    w, l,
    glm::dvec3(center_point.x, center_point.y - h/2.0, center_point.z),
    0.0,
    glm::dvec3(0, 0, 1),
    color,
    true
  );

  auto side_front_sheet = forgeRectSheetEntity(
    w, h,
    glm::dvec3(center_point.x, center_point.y, center_point.z + l/2.0),
    90.0,
    glm::dvec3(1, 0, 0),
    color,
    true
  );
  auto side_back_sheet = forgeRectSheetEntity(
    w, h,
    glm::dvec3(center_point.x, center_point.y, center_point.z - l/2.0),
    90.0,
    glm::dvec3(1, 0, 0),
    color,
    true
  );
  auto side_left_sheet = forgeRectSheetEntity(
    l, h,
    glm::dvec3(center_point.x + w/2.0, center_point.y, center_point.z),
    90.0,
    glm::dvec3(0, 0, 1),
    color,
    true
  );
  auto side_right_sheet = forgeRectSheetEntity(
    l, h,
    glm::dvec3(center_point.x - w/2.0, center_point.y, center_point.z),
    90.0,
    glm::dvec3(0, 0, 1),
    color,
    true
  );

  return {
    top_sheet,
    bottom_sheet,
    side_front_sheet,
    side_back_sheet,
    side_left_sheet,
    side_right_sheet
  };
}

}
}



#endif /* transparent_sheetbox_forging_cpp */
