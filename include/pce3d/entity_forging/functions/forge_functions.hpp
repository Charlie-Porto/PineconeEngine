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

using VertexVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using VertexPixelMap = std::unordered_map<uint32_t, glm::dvec2>;
using VertexDistanceMap = std::unordered_map<uint32_t, double>;
using EdgeMap = std::vector<std::pair<uint32_t, uint32_t>>;
using IndexFaceMap = std::unordered_map<glm::ivec3, uint32_t>;
using FaceIndexMap = std::unordered_map<uint32_t, glm::ivec3>;
using EntityFaceCollisionMap = std::unordered_map<uint32_t, uint32_t>;

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using FaceCornerMap = std::unordered_map<uint32_t, glm::dvec3>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using FaceVertexCornerMap = std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>;
using VertexFaceCornerMap = std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>;  


void rotateVertices(VertexMap& vertices, const double angle, const glm::dvec3& axis,
                    const glm::dvec3& center_of_gravity);


void createFaceVertexCornerMaps(const VertexMap& vertices, const FaceVertexMap& faces,
                                FaceCornerMap& face_corner_map,
                                FaceVertexCornerMap& face_vertex_corner_map,
                                VertexFaceCornerMap& vertex_face_corner_map,
                                const glm::dvec3& center_point);



}
}




#endif /* forge_functions_hpp */
