#ifndef spaceMapFunctions_hpp
#define spaceMapFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
free functions to assist the space map system

mir = meter index ratio
mdim = map dimensions
-----------------------------------------------------------------*/

#include <unordered_map>
#include <vector>
#include <glm/vec3.hpp>

namespace pce3d {
namespace space_map {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;

glm::ivec3 findIndexOfPoint(const glm::dvec3& point, const glm::ivec3& mdim, const double mir);

glm::dvec3 findPointOfIndex(const glm::ivec3& index, const glm::ivec3& mdim, const double mir);

std::vector<glm::ivec3> findIndicesGivenVertices(const VertexMap& vertices, const glm::dvec3& mdim, const double mir);

std::vector<glm::ivec3> findIndicesOfFaceMidpoints(const std::vector<uint32_t>& face,
                                                   const VertexMap& vertices, const glm::dvec3& mdim, 
                                                   const double mir);

}
}





#endif /* spaceMapFunctions_hpp */
