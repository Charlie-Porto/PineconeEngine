#ifndef rigid_object_component_cpp
#define rigid_object_component_cpp

#include <unordered_map>
#include <utility>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include "../utilities/functions/glm_hash.hpp"

namespace pce {

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

struct RigidObject {
  double radius;  // if == 0, then not a sphere

  double mass;
  bool is_deadbod;
  bool is_restingbod;
  bool is_complex_livebod;

  VertexMap vertices;
  VertexVertexMap vertex_vertex_map;
  EdgeMap edges;
  FaceVertexMap face_vertex_map;
  FaceEdgeMap face_edge_map;
  uint32_t base_face_id;
  int face_count;

  VertexMap camera_transformed_vertices;
  VertexPixelMap vertex_pixels;
  VertexDistanceMap vertex_distance_map;

  FaceCornerMap face_corner_map;
  FaceCornerMap camera_rotated_face_corner_map;
  VertexFaceCornerMap vertex_face_corner_map;
  FaceVertexCornerMap face_vertex_corner_map;

  IndexFaceMap index_face_map;
  FaceIndexMap face_index_map;  
  IndexVertexMap index_vertex_map;
  VertexIndexMap vertex_index_map;

  EntityFaceCollisionMap entity_face_collision_map;
  EntityVertexCollisionMap entity_vertex_collision_map;
  EntityEdgeCollisionMap entity_edge_collision_map;
  EntityIndexCollisionMap entity_index_collision_map;
  EntityTimeCollisionMap entity_time_collision_map;
};

}



#endif /* rigid_object_component_cpp */
