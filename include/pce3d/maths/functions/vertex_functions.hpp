#ifndef vertex_functions_hpp
#define vertex_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to relating to polyhedron vertices
-----------------------------------------------------------------*/

#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include "vector_functions.hpp"
#include "plane_functions.hpp"
#include "../../utilities/functions/glm_hash.hpp"

namespace pce3d {
namespace maths {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using VertexDistanceMap = std::unordered_map<uint32_t, double>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using EdgeMap = std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>;
using VertexFaceCornerMap = std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>;

uint32_t calculateClosestVertexToPoint(const glm::dvec3& point, const VertexMap& vertices);

std::vector<uint32_t> calculateClosestPolyhedronFaceToPoint(
    const VertexMap& vertices, const EdgeMap& edges,  const FaceVertexMap& faces, const glm::dvec3& point);

uint32_t calculateClosestVertexOfFaceToOrigin(const std::vector<uint32_t>& face_vertex_ids, 
                                              const VertexDistanceMap& vertex_distance_map);

std::pair<bool, uint32_t> determineIfVertexTouchesObjectEdges(
    const glm::dvec3& vertex
  , const VertexMap& vertices
  , const EdgeMap& edges
  , const double distance_threshold
);


std::pair<bool, uint32_t> determineIfVertexTouchesObjectFaces(
    const glm::dvec3& vertex
  , const uint32_t closest_object_vertex_id
  , const VertexMap& vertices
  , const FaceVertexMap& faces
  , const VertexFaceCornerMap& vertex_face_corner_map
  , const double distance_threshold
);



}
}





#endif /* vertex_functions_hpp */
