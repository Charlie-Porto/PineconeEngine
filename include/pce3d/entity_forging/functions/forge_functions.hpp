#ifndef forge_functions_hpp
#define forge_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist the forge process
-----------------------------------------------------------------*/

#include <iostream>
#include <unordered_map>
#include <glm/vec3.hpp>
#include "../../maths/functions/quaternion_functions.hpp"

namespace pce3d {
namespace forge {


using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using VertexVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using VertexPixelMap = std::unordered_map<uint32_t, glm::dvec2>;
using VertexDistanceMap = std::unordered_map<uint32_t, double>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using EdgeMap = std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>;
using FaceEdgeMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using IndexFaceMap = std::unordered_map<glm::ivec3, uint32_t>;
using FaceIndexMap = std::unordered_map<uint32_t, glm::ivec3>;
using IndexVertexMap = std::unordered_map<glm::ivec3, uint32_t>;
using VertexIndexMap = std::unordered_map<uint32_t, glm::ivec3>;
using EntityFaceCollisionMap = std::unordered_map<uint32_t, uint32_t>;
using EntityVertexCollisionMap = std::unordered_map<uint32_t, uint32_t>;
using EntityEdgeCollisionMap = std::unordered_map<uint32_t, uint32_t>;
using EntityIndexCollisionMap = std::unordered_map<uint32_t, std::vector<glm::ivec3>>;
using EntityTimeCollisionMap = std::unordered_map<uint32_t, double>;
using FaceCornerMap = std::unordered_map<uint32_t, glm::dvec3>;
using FaceVertexCornerMap = std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>;
using VertexFaceCornerMap = std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>;



void rotateVertices(VertexMap& vertices, const double angle, const glm::dvec3& axis,
                    const glm::dvec3& center_of_gravity);


void createFaceVertexCornerMaps(const VertexMap& vertices, const FaceVertexMap& faces,
                                FaceCornerMap& face_corner_map,
                                FaceVertexCornerMap& face_vertex_corner_map,
                                VertexFaceCornerMap& vertex_face_corner_map,
                                const glm::dvec3& center_point,
                                bool vertices_normalized_to_object_center = false);

void createFaceEdgeMap(
    const FaceVertexMap& face_vertex_map
  , const EdgeMap& edge_map
  , FaceEdgeMap& face_edge_map
);

}
}




#endif /* forge_functions_hpp */
