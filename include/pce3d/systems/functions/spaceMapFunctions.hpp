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

extern ControlPanel control;

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

std::vector<glm::ivec3> findFaceIndicesGeneral(
    const uint32_t entity
  , const uint32_t face
  , pce::RigidObject& rigid_object
  , const glm::ivec3& mdim
  , const double mir
);

std::unordered_map<uint32_t, glm::ivec3> updateBodVertexMap(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& bod_vertex_map_ 
  , const glm::ivec3 mdim
  , const double mir
);

std::unordered_map<uint32_t, std::vector<glm::ivec3>> updateBodEdgeMap(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& bod_edge_map_ 
  , const glm::ivec3 mdim
  , const double mir
);

void doPreLoopMapUpdate(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , std::unordered_map<glm::ivec3, std::vector<uint32_t>>& bod_map_
  , const glm::ivec3 mdim
  , const double mir
);


void updateLiveBodIndicesAndCheckForLiveBodCollision(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , const glm::ivec3 mdim
  , const double mir
  , std::unordered_map<glm::ivec3, std::vector<uint32_t>>& livebod_map
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& livebod_vertex_map
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& livebod_edge_map
  , std::unordered_map<glm::ivec3, std::unordered_map<uint32_t, uint32_t>>& livebod_index_face_map
  , std::unordered_map<uint32_t, uint32_t>& potential_colliding_entities
);

void checkForCollisionWithNonLiveBods(
    const uint32_t entity
  , pce::RigidObject& rigid_object
  , std::unordered_map<uint32_t, glm::ivec3>& vertex_indices
  , const glm::ivec3 mdim
  , const double mir
  , std::unordered_map<glm::ivec3, std::vector<uint32_t>>& deadbod_map
  , std::unordered_map<glm::ivec3, std::vector<uint32_t>>& restingbod_map
  , std::unordered_map<uint32_t, uint32_t>& potential_colliding_entities
);









}
}





#endif /* spaceMapFunctions_hpp */
