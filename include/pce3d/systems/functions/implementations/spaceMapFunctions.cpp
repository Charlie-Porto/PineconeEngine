#ifndef spaceMapFunctions_cpp
#define spaceMapFunctions_cpp

#include "../spaceMapFunctions.hpp"

namespace pce3d {
namespace space_map {

glm::ivec3 findIndexOfPoint(const glm::dvec3& point, const double meter_index_ratio) {
  const glm::dvec3 p = point / meter_index_ratio;
  return glm::ivec3(p.x, p.y, p.z);
}

glm::dvec3 findPointOfIndex(const glm::ivec3& index, const double meter_index_ratio) {
  const glm::ivec3 p = index * int(meter_index_ratio);
  return glm::dvec3(p.x, p.y, p.z);
}

std::vector<glm::ivec3> findIndicesGivenVertices(VertexMap vertices, const double meter_index_ratio) {
  std::vector<glm::ivec3> indices{};
  for (auto const& [id, vertex] : vertices) {
    indices.push_back(findIndexOfPoint(vertex, meter_index_ratio));
  }
  return indices;
}



}
}


#endif /* spaceMapFunctions_cpp */
