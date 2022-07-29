#ifndef spaceMapFunctions_cpp
#define spaceMapFunctions_cpp

#include "../spaceMapFunctions.hpp"

namespace pce3d {
namespace space_map {


glm::ivec3 findIndexOfPoint(const glm::dvec3& point, const glm::ivec3& mdim, const double mir) {
  glm::dvec3 p = point / mir;
  p = p + ((glm::dvec3(mdim.x, mdim.y, mdim.z) / mir) / 2.0);
  return glm::ivec3(p.x, p.y, p.z);
}


glm::dvec3 findPointOfIndex(const glm::ivec3& index, const glm::ivec3& mdim, const double mir) {
  glm::ivec3 p = index - (mdim / int(mir)) / 2;
  p = p * int(mir);
  return glm::dvec3(p.x, p.y, p.z);
}


std::vector<glm::ivec3> findIndicesGivenVertices(const VertexMap& vertices, const glm::dvec3& mdim, const double mir) {
  std::vector<glm::ivec3> indices{};
  for (auto const& [id, vertex] : vertices) {
    indices.push_back(findIndexOfPoint(vertex, mdim, mir));
  }
  return indices;
}


std::vector<glm::ivec3> findIndicesOfFaceMidpoints(const std::vector<uint32_t>& face,
                                                   const VertexMap& vertices, const glm::dvec3& mdim, 
                                                   const double mir) {
  
}
  


}
}


#endif /* spaceMapFunctions_cpp */
