#ifndef vertex_functions_hpp
#define vertex_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to relating to polyhedron vertices
-----------------------------------------------------------------*/

#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "vector_functions.hpp"
#include <glm_hash.hpp>

namespace pce3d {
namespace maths {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using EdgeMap = std::vector<std::pair<uint32_t, uint32_t>>;

uint32_t calculateClosestVertexToPoint(const glm::dvec3& point, const VertexMap& vertices);

std::vector<uint32_t> calculateClosestPolyhedronFaceToPoint(
    const VertexMap& vertices, const EdgeMap& edges,  const FaceVertexMap& faces, const glm::dvec3& point);

}
}





#endif /* vertex_functions_hpp */
