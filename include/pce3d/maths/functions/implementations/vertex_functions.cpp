#ifndef vertex_functions_cpp
#define vertex_functions_cpp

#include "../vertex_functions.hpp"

namespace pce3d {
namespace maths {

uint32_t calculateClosestVertexToPoint(const glm::dvec3& point, const VertexMap& vertices) {
  double minimum = 100000.0;
  uint32_t closest_vertex = 1;

  for (auto const& [id, vertex] : vertices) {
    const double distance = calculateDistanceBetweenVectors(point, vertex);
    if (distance < minimum) {
      minimum = distance;
      closest_vertex = id;
    }
  }
  return closest_vertex;
}

}
}





#endif /* vertex_functions_cpp */
