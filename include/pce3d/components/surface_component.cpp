#ifndef surface_component_cpp
#define surface_component_cpp

#include <vector>
#include <unordered_map>

namespace pce {

struct Surface {
  std::vector<int> color;
  std::unordered_map<uint32_t, std::vector<int>> face_color_map;
  double opacity;
};

}

#endif /* surface_component_cpp */
