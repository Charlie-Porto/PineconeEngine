#ifndef renderFunctions_hpp
#define renderFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist with rendering
-----------------------------------------------------------------*/

#include <utility>
#include <vector>
#include <unordered_map>
#include "../../maths/functions/vector_functions.hpp"

extern pce3d::DevRenderSystem dev_render_system;

namespace pce3d {
namespace render {

using VertexDistanceMap = std::unordered_map<uint32_t, double>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using FaceCornerMap = std::unordered_map<uint32_t, glm::dvec3>;
using FaceVertexCornerMap = std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>;
using VertexFaceCornerMap = std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>;

std::vector<std::pair<uint32_t, double>> orderFacesByCameraProximity(const FaceVertexMap& face_vertex_map,
                                                  const VertexDistanceMap& vertex_distance_map);

std::vector<uint32_t> getFacesOrderedForRender(const uint32_t closest_vertex_id, 
                                               const VertexFaceCornerMap& vertex_face_corner_map,
                                               const FaceCornerMap& face_corner_map);
}
}





#endif /* renderFunctions_hpp */
