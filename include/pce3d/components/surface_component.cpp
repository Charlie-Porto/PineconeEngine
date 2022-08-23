#ifndef surface_component_cpp
#define surface_component_cpp

#include <vector>
#include <unordered_map>

namespace pce {

struct Surface {
  std::vector<int> color;
  std::unordered_map<uint32_t, std::vector<int>> face_color_map;
  double opacity;
  double collision_elasticity_index;
  bool is_transparent = false;
};

}

#endif /* surface_component_cpp */
