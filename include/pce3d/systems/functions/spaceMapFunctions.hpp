#ifndef spaceMapFunctions_hpp
#define spaceMapFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
free functions to assist the space map system

mir = meter index ratio
mdim = map dimensions
-----------------------------------------------------------------*/

#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "../../maths/functions/triangle_functions.hpp"
#include "../../maths/functions/vector_functions.hpp"

namespace pce3d {
namespace space_map {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;


glm::ivec3 findIndexOfPoint(const glm::dvec3& point, const glm::ivec3& mdim, const double mir);

glm::dvec3 findPointOfIndex(const glm::ivec3& index, const glm::ivec3& mdim, const double mir);

std::vector<glm::ivec3> findIndicesGivenVertices(const VertexMap& vertices, const glm::dvec3& mdim, const double mir);

std::unordered_map<uint32_t, glm::ivec3> findIndicesGivenVerticesLabeled(const VertexMap& vertices, const glm::dvec3& mdim, const double mir);

std::vector<glm::dvec3> orderVerticesByDistanceFromFirst(const std::vector<glm::dvec3>& vertices);

std::vector<glm::ivec3> findFaceIndices(const std::vector<uint32_t>& face,
                                        const VertexMap& vertices, const glm::dvec3& mdim,
                                        const double mir);

std::vector<glm::ivec3> findTriangleFaceTopIndices(const std::vector<glm::dvec3>& vertices,
                                                      const glm::dvec3& mdim,
                                                      const double mir);

std::vector<glm::ivec3> findTriangleFaceBottomIndices(const std::vector<glm::dvec3>& vertices,
                                                      const glm::dvec3& mdim,
                                                      const double mir);

  
std::vector<glm::ivec3> findRectFaceIndices(const std::vector<uint32_t>& face,
                                            const VertexMap& unordered_vertices, const glm::dvec3& mdim,
                                            const double mir);

std::vector<glm::ivec3> findTriangleFaceIndices(const std::vector<uint32_t>& face,
                                                const VertexMap& vertices, const glm::dvec3& mdim,
                                                const double mir);

}
}





#endif /* spaceMapFunctions_hpp */
