#ifndef face_shade_component_cpp
#define face_shade_component_cpp

#include <unordered_map>
#include <glm_hash.hpp>

namespace pce {

using FaceShadeMap = std::unordered_map<uint32_t, double>;
using PixelShadeMap = std::unordered_map<glm::dvec2, double>;

struct FaceShade {
  FaceShadeMap face_shade_map;
  PixelShadeMap pixel_shade_map;
};

}

#endif /* face_shade_component_cpp */
