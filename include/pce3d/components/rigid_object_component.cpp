#ifndef rigid_object_component_cpp
#define rigid_object_component_cpp

#include <unordered_map>
#include <utility>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

namespace pce {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using VertexPixelMap = std::unordered_map<uint32_t, glm::dvec2>;
using VertexDistanceMap = std::unordered_map<uint32_t, double>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using EdgeMap = std::vector<std::pair<uint32_t, uint32_t>>;

struct RigidObject {
  double radius;  // if == 0, then not a sphere
  VertexMap vertices;
  EdgeMap edges;
  FaceVertexMap face_vertex_map;
  VertexMap camera_transformed_vertices;
  VertexPixelMap vertex_pixels;
  VertexDistanceMap vertex_distance_map;
};

}



#endif /* rigid_object_component_cpp */
