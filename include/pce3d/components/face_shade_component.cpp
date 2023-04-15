#ifndef face_shade_component_cpp
#define face_shade_component_cpp

#include <unordered_map>
#include "../utilities/functions/glm_hash.hpp"

namespace pce {

using FaceShadeMap = std::unordered_map<uint32_t, double>;
using PixelShadeMap = std::unordered_map<glm::dvec2, double>;
using PixelMap = std::unordered_map<glm::dvec2, glm::dvec2>;
using SurfaceZoneBrightnessMap = std::unordered_map<glm::dvec3, double>;

struct FaceShade {
  FaceShadeMap face_shade_map;
  double virtual_pixel_ratio = 1.0;
  PixelShadeMap pixel_shade_map;
  PixelMap outline_pixels{};
  double center_point_shade;
  SurfaceZoneBrightnessMap surface_zone_brightness_map;
  SurfaceZoneBrightnessMap camera_transformed_surface_zone_brightness_map;
};

}

#endif /* face_shade_component_cpp */
