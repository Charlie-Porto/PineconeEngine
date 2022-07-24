#ifndef face_shade_component_cpp
#define face_shade_component_cpp

#include <unordered_map>

namespace pce {

using FaceShadeMap = std::unordered_map<uint32_t, double>;

struct FaceShade {
  FaceShadeMap face_shade_map;
};

}

#endif /* face_shade_component_cpp */
