#ifndef rigid_object_component_cpp
#define rigid_object_component_cpp

#include <utility>
#include <vector>
#include <glm/vec3.hpp>

namespace pce {

using VertexMap = std::vector<std::pair<uint32_t, glm::dvec3>>;
using EdgeMap = std::vector<std::pair<std::pair<uint32_t, glm::dvec3>, std::pair<uint32_t, glm::dvec3>>>;

struct RigidObject {
  double radius;  // if == 0, then not a sphere
  VertexMap vertices;
  EdgeMap edges;
};

}



#endif /* rigid_object_component_cpp */
