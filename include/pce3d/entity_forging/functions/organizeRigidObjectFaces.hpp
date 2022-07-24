#ifndef organizeRigidObjectFaces_hpp
#define organizeRigidObjectFaces_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
function to assign and organize a rigid object's faces
-----------------------------------------------------------------*/

#include <unordered_map>
#include <utility>
#include <vector>
#include <glm/vec3.hpp>
#include "../../maths/objects/pceVec3.hpp"


namespace pce3d {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using EdgeMap = std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>>;
using FaceMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using FaceNormalVectorMap = std::unordered_map<uint32_t, pce3d::pceVec3>;
using VertexEdgeMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;

VertexEdgeMap defineVertexEdgeMap(const VertexMap& vertex_map, const EdgeMap& edge_map);

void groupRigidBodyEdgesIntoFaces(const VertexMap& vertex_map, 
                                  const EdgeMap& edge_map, FaceMap& face_map);

}





#endif /* organizeRigidObjectFaces_hpp */
