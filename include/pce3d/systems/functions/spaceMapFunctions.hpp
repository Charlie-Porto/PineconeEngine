#ifndef spaceMapFunctions_hpp
#define spaceMapFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
free functions to assist the space map system
-----------------------------------------------------------------*/

#include <unordered_map>
#include <vector>
#include <glm/vec3.hpp>

namespace pce3d {
namespace space_map {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;

glm::ivec3 findIndexOfPoint(const glm::dvec3& point, const double meter_index_ratio);

glm::dvec3 findPointOfIndex(const glm::ivec3& index, const double meter_index_ratio);

std::vector<glm::ivec3> findIndicesGivenVertices(VertexMap vertices, const double meter_index_ratio);

std::vector<glm::ivec3> findIndexOfMidPointsRecursive(const glm::ivec3& A, const double meter_index_ratio);

}
}





#endif /* spaceMapFunctions_hpp */
