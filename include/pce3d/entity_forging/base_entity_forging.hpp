#ifndef base_entity_forging_hpp
#define base_entity_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
base entity forging. 
-----------------------------------------------------------------*/

#include <vector>

#include <glm/vec3.hpp>

extern ControlPanel control;

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


namespace pce3d {
namespace forge {
  
  uint32_t forgeBaseEntity(
      const glm::dvec3 center_location
    , const std::vector<int>& color = {255, 255, 255, 255}
    , const double collision_elasticity = 0.7
  );

}
}




#endif /* base_entity_forging_hpp */
