#ifndef vertex_functions_cpp
#define vertex_functions_cpp

#include "../vertex_functions.hpp"

namespace pce3d {
namespace maths {

uint32_t calculateClosestVertexToPoint(const glm::dvec3& point, const VertexMap& vertices) {
  double minimum = 100000.0;
  uint32_t closest_vertex = 1;

  for (auto const& [id, vertex] : vertices) {
    const double distance = calculateDistanceBetweenVectors(point, vertex);
    if (distance < minimum) {
      minimum = distance;
      closest_vertex = id;
    }
  }
  return closest_vertex;
}

std::vector<uint32_t> calculateClosestPolyhedronFaceToPoint(
    const VertexMap& vertices, const EdgeMap& edges, const FaceVertexMap& faces, const glm::dvec3& point) {

  const uint32_t closest_vertex = calculateClosestVertexToPoint(point, vertices);
  std::vector<uint32_t> connected_vertices{};
  std::vector<uint32_t> connected_faces{};
  std::vector<std::pair<uint32_t, uint32_t>> edge_vertices_to_bisect{};
  std::unordered_map<glm::dvec3, std::pair<uint32_t, uint32_t>> edge_bisector_vertex_map{};
  std::unordered_map<double, std::pair<uint32_t, uint32_t>> distance_edge_bisector_map{};
  double min_face_distance = 100000.0;

  /* pick up here */ 
  for (auto const& edge : edges) {
    if (edge.first == closest_vertex) { connected_vertices.push_back(edge.second); }
    if (edge.second == closest_vertex) { connected_vertices.push_back(edge.first); }
  }
  // std::cout << "edges vertices obtained" << '\n';
  // std::cout << "face count: " << faces.size() << '\n';
  for (auto const& [face, vertices] : faces) {
    // std::cout << "face: " << face << '\n';
    if (std::find(vertices.begin(), vertices.end(), closest_vertex) != vertices.end()) {
      connected_faces.push_back(face);
      std::vector<uint32_t> indices{};
      for (auto const& vertex : vertices) {
        if (std::find(connected_vertices.begin(), connected_vertices.end(), vertex) 
                   != connected_vertices.end()) { 
          indices.push_back(vertex);
        }
      }
      // std::cout << indices.size() << '\n';
      assert(indices.size() <= 3);
      if (indices.size() == 2) {
        edge_vertices_to_bisect.push_back(std::make_pair(indices[0], indices[1]));
      }
    }
  }

  for (auto const& vpair : edge_vertices_to_bisect) {
    const glm::dvec3 A = vertices.at(vpair.first);
    const glm::dvec3 B = vertices.at(vpair.second);
    const glm::dvec3 bisector_unit_vector = glm::normalize(A + B) / 1000.0;

    edge_bisector_vertex_map[bisector_unit_vector] = vpair;
    const glm::dvec3 position_vector = vertices.at(closest_vertex) + bisector_unit_vector;
    const double distance = sqrt(glm::dot(position_vector, position_vector));
    distance_edge_bisector_map[distance] = vpair;
    if (distance < min_face_distance) { 
      min_face_distance = distance; 
      // std::cout << "new shortest distance: " << distance << "pair: " << vpair.first << ", " << vpair.second << '\n';
    }
  }
  
  return {distance_edge_bisector_map.at(min_face_distance).first,
          distance_edge_bisector_map.at(min_face_distance).second,
          closest_vertex};
}

}
}





#endif /* vertex_functions_cpp */
